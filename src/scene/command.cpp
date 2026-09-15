// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "scene/command.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <utility>

namespace stratum::scene {

CommandStack::CommandStack(CommandStackConfig config) : m_config(config) {}

size_t CommandStack::step_bytes(const Step& step) {
    size_t total = 0;
    for (const CommandPtr& command : step.commands) {
        total += command ? command->footprint() : 0u;
    }
    return total;
}

bool CommandStack::execute(CommandPtr command) {
    if (!command) return false;

    if (!command->apply()) {
        // Refused. The history must look exactly as it did, including the redo
        // branch: an edit the user could not make is not an edit, and throwing
        // away their redo for it would be a second surprise on top of the first.
        return false;
    }

    ++m_revision;

    if (m_transaction_depth > 0) {
        // Inside a group: hold it, and let commit() decide what the step is.
        // Nothing is merged here -- merging across a transaction boundary would
        // let a group swallow an edit made before it opened.
        m_pending.push_back(std::move(command));
        return true;
    }

    // A new edit invalidates the redo branch. Do this before the merge attempt:
    // merging into the top of the undo stack is still a new edit, and leaving a
    // stale redo branch alive after it is how redo restores state from a history
    // that no longer exists.
    m_redo.clear();

    if (!m_undo.empty()) {
        Step& top = m_undo.back();
        if (!top.sealed && top.commands.size() == 1u && top.commands.front()
            && top.commands.front()->merge(*command)) {
            // Absorbed. The incoming command dies here; the survivor's revert()
            // now undoes both, which is the contract merge() signs up to.
            m_bytes -= top.bytes;
            top.bytes = step_bytes(top);
            m_bytes += top.bytes;
            return true;
        }
    }

    Step step;
    step.label = command->describe();
    step.commands.push_back(std::move(command));
    push_step(std::move(step));
    return true;
}

void CommandStack::push_step(Step step) {
    step.bytes = step_bytes(step);
    m_bytes += step.bytes;
    m_undo.push_back(std::move(step));
    trim();
}

void CommandStack::trim() {
    // Depth first, then memory. Both drop from the OLDEST end: the most recent
    // history is the part a user actually reaches for, and a stack that dropped
    // the newest step to stay under a limit would be undo that forgets what you
    // just did.
    while (m_undo.size() > m_config.max_depth) {
        m_bytes -= m_undo.front().bytes;
        m_undo.erase(m_undo.begin());
    }

    // Stop at one. A lone step over the ceiling is kept: dropping it would leave
    // the stack unable to hold the edit that was just made, which is worse than
    // being over budget by one step.
    while (m_bytes > m_config.max_bytes && m_undo.size() > 1u) {
        m_bytes -= m_undo.front().bytes;
        m_undo.erase(m_undo.begin());
    }
}

bool CommandStack::can_undo() const { return !m_undo.empty() && m_transaction_depth == 0; }
bool CommandStack::can_redo() const { return !m_redo.empty() && m_transaction_depth == 0; }

bool CommandStack::undo() {
    if (!can_undo()) return false;

    Step step = std::move(m_undo.back());
    m_undo.pop_back();
    m_bytes -= step.bytes;

    // Reverse order. A transaction that created a lot and then set its zoning
    // has to unset the zoning before the lot goes away.
    for (auto it = step.commands.rbegin(); it != step.commands.rend(); ++it) {
        if (*it) (*it)->revert();
    }

    // Sealed on the way out, so redoing and then editing cannot merge into a
    // step that was completed long ago.
    step.sealed = true;
    m_redo.push_back(std::move(step));
    ++m_revision;
    return true;
}

bool CommandStack::redo() {
    if (!can_redo()) return false;

    Step step = std::move(m_redo.back());
    m_redo.pop_back();

    // Forward order, mirroring undo. A refusal here is a real problem: the
    // command applied once already, so the scene is not in the state the history
    // says it is. Revert what went in, drop the step, and keep the stack
    // consistent rather than carrying a lie forward.
    size_t applied = 0;
    bool ok = true;
    for (CommandPtr& command : step.commands) {
        if (!command || !command->apply()) {
            ok = false;
            break;
        }
        ++applied;
    }

    if (!ok) {
        for (size_t i = applied; i-- > 0;) {
            step.commands[i]->revert();
        }
        spdlog::warn("CommandStack: redo of \"{}\" was refused; the step has been dropped",
                     step.label);
        ++m_revision;
        return false;
    }

    push_step(std::move(step));
    m_undo.back().sealed = true;
    ++m_revision;
    return true;
}

std::string CommandStack::undo_label() const {
    return m_undo.empty() ? std::string{} : m_undo.back().label;
}

std::string CommandStack::redo_label() const {
    return m_redo.empty() ? std::string{} : m_redo.back().label;
}

void CommandStack::seal() {
    if (!m_undo.empty()) m_undo.back().sealed = true;
}

void CommandStack::begin_transaction(std::string label) {
    if (m_transaction_depth == 0) {
        m_pending.clear();
        m_transaction_labels.clear();
    }
    m_transaction_labels.push_back(std::move(label));
    ++m_transaction_depth;
}

void CommandStack::commit_transaction() {
    if (m_transaction_depth == 0) {
        spdlog::warn("CommandStack: commit_transaction() with no transaction open");
        return;
    }

    --m_transaction_depth;
    if (m_transaction_depth > 0) {
        // Inner commit. The group is not a step until the outermost closes, or
        // nesting would produce one undo step per level.
        m_transaction_labels.pop_back();
        return;
    }

    if (m_pending.empty()) {
        // A group that changed nothing is not a step. Common and harmless: a
        // tool opens a transaction on mouse-down and the user clicks nothing.
        m_transaction_labels.clear();
        return;
    }

    Step step;
    step.label = m_transaction_labels.empty() ? m_pending.front()->describe()
                                              : m_transaction_labels.front();
    step.commands = std::move(m_pending);
    // Always sealed. A group is a completed action by definition, and letting
    // the next edit merge into it would extend something already finished.
    step.sealed = true;

    m_pending.clear();
    m_transaction_labels.clear();
    m_redo.clear();
    push_step(std::move(step));
}

void CommandStack::abort_transaction() {
    if (m_transaction_depth == 0) {
        spdlog::warn("CommandStack: abort_transaction() with no transaction open");
        return;
    }

    // Unwind to the outermost regardless of nesting depth. A nested abort that
    // left the outer group open would hand back a transaction whose contents had
    // already been reverted.
    for (auto it = m_pending.rbegin(); it != m_pending.rend(); ++it) {
        if (*it) (*it)->revert();
    }

    if (!m_pending.empty()) ++m_revision;

    m_pending.clear();
    m_transaction_labels.clear();
    m_transaction_depth = 0;
}

void CommandStack::clear() {
    m_undo.clear();
    m_redo.clear();
    m_pending.clear();
    m_transaction_labels.clear();
    m_transaction_depth = 0;
    m_bytes = 0;
    // Deliberately NOT reset. Revision is how A2 answers "is this document
    // dirty", and a load that reset it to zero would make a freshly loaded
    // document compare equal to an unsaved one that happened to be at zero too.
    ++m_revision;
}

} // namespace stratum::scene
