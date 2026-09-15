// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file command.hpp
 * @brief Undo and redo: the stack every edit in Stratum goes through
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists before there is anything to edit
 *
 * Stratum has no editing features yet. It has an importer, a solver and a
 * viewer. Tracks B, C, E and K of the parity plan add roughly forty features
 * that mutate a scene, and every one of them has to be undoable.
 *
 * A command stack cannot be retrofitted cheaply. Bolting it on after the fact
 * means revisiting every mutation already written, finding the state each one
 * destroyed, and reconstructing a way to put it back -- for code whose author
 * was not thinking about reversibility when they wrote it. That is the single
 * largest avoidable cost in the whole programme, which is why this lands first,
 * with no callers, rather than after the first edit tool that needs it.
 *
 * ### The model
 *
 * A Command knows how to do a thing and how to undo it. It is NOT a diff and it
 * is NOT a snapshot: at city scale a snapshot of the scene per keystroke is
 * gigabytes, and a structural diff of an EnTT registry is harder to get right
 * than the inverse operation usually is. Moving a node is undone by moving it
 * back; deleting a lot is undone by putting the lot you kept a copy of back.
 *
 * Three things follow from that, and each is a trap worth naming:
 *
 *   - **A command owns whatever its undo needs.** Delete cannot hold a
 *     reference to the thing it deleted. `footprint()` exists so the stack can
 *     bound how much that costs in total.
 *   - **Commands are applied through the stack, never directly.** A mutation
 *     that skips execute() is a hole in the history, and the symptom is an undo
 *     three steps later restoring state that was never current.
 *   - **revert() must not fail.** apply() may refuse -- the operation might be
 *     invalid -- and refusing leaves the stack untouched. But once applied, the
 *     inverse has to work, because there is nothing sensible to do with a
 *     half-undone history. A revert that cannot run is a bug in the command.
 *
 * ### Coalescing
 *
 * Dragging a node emits one command per mouse-move. Without merging, undo walks
 * back through a drag one pixel at a time, which is not undo as anyone means it.
 * merge() lets a command absorb its successor, and seal() draws the line --
 * typically on mouse-up. Merging is deliberately NOT time-based: a clock makes
 * the history depend on how fast the machine ran, and makes these tests
 * unreproducible.
 *
 * ### Transactions
 *
 * One user action often means many commands: deleting a block deletes its lots,
 * its buildings and their attributes. begin()/commit() groups them so undo takes
 * the whole thing back. An aborted transaction reverts what it had applied, so a
 * failure part-way through leaves no partial edit behind.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace stratum::scene {

/**
 * @brief One reversible edit
 *
 * Subclass this for every mutation. Keep the inverse state in the command, not
 * in the caller.
 */
class Command {
public:
    virtual ~Command() = default;

    /**
     * @brief Perform the edit
     *
     * Called once by CommandStack::execute(), and again on every redo.
     *
     * @return false to refuse. The stack then discards the command and leaves
     *         the history exactly as it was, so refusing is safe and is the
     *         right answer for an edit that turns out to be invalid.
     */
    [[nodiscard]] virtual bool apply() = 0;

    /**
     * @brief Put back what apply() changed
     *
     * Must succeed. See the note on failure in the file comment.
     */
    virtual void revert() = 0;

    /// Short label for the undo menu, e.g. "Move node". Present tense, no "Undo".
    [[nodiscard]] virtual std::string describe() const = 0;

    /**
     * @brief Bytes this command holds onto, for the stack's memory bound
     *
     * The default covers the object itself. Override when the command owns
     * heap state -- a deleted mesh, a copied attribute map -- or the bound
     * silently fails to bound anything.
     */
    [[nodiscard]] virtual size_t footprint() const { return sizeof(Command); }

    /**
     * @brief Absorb the command that would have followed this one
     *
     * Called on the command at the top of the undo stack, with the incoming one.
     * Return true to have taken it over entirely: the incoming command is then
     * dropped, and this one's revert() must undo BOTH.
     *
     * Only offered when this command is unsealed and is the most recent, so a
     * merge can never reach across an unrelated edit.
     *
     * @param next Command being pushed. Already applied when this is called.
     */
    [[nodiscard]] virtual bool merge(const Command& next) {
        (void)next;
        return false;
    }
};

using CommandPtr = std::unique_ptr<Command>;

/// Limits on how much history is kept.
struct CommandStackConfig {
    /**
     * @brief Most undo steps to keep
     *
     * Reaching it drops the OLDEST step, which then cannot be undone. That is
     * the correct trade: the alternative is refusing the edit.
     */
    size_t max_depth = 256;

    /**
     * @brief Ceiling on the total of every step's footprint(), in bytes
     *
     * Applied after max_depth. A single command larger than this is still kept
     * -- dropping it would leave the stack unable to hold the edit the user just
     * made -- but nothing else is kept alongside it.
     */
    size_t max_bytes = 64u * 1024u * 1024u;
};

/**
 * @brief Linear undo history
 *
 * Not thread-safe. Edits come from the UI thread; a background job that wants to
 * mutate the scene hands a command back to that thread rather than pushing from
 * its own.
 */
class CommandStack {
public:
    explicit CommandStack(CommandStackConfig config = {});

    /**
     * @brief Apply a command and record it
     *
     * @return false when the command refused. Nothing is recorded, the redo
     *         history is left alone, and the command is destroyed.
     */
    bool execute(CommandPtr command);

    [[nodiscard]] bool can_undo() const;
    [[nodiscard]] bool can_redo() const;

    /// @return false when there was nothing to undo.
    bool undo();

    /// @return false when there was nothing to redo, or the redo refused.
    bool redo();

    /// Label of the step undo() would take, or "" when there is none.
    [[nodiscard]] std::string undo_label() const;

    /// Label of the step redo() would take, or "" when there is none.
    [[nodiscard]] std::string redo_label() const;

    /**
     * @brief Stop the top step absorbing anything further
     *
     * Call at the end of a gesture -- mouse-up, focus loss, tool change. Without
     * it a drag and the click after it merge into one undo step.
     */
    void seal();

    /**
     * @brief Open a group that undoes as one step
     *
     * Nestable; only the outermost commit writes a step. A label is taken from
     * the outermost begin.
     */
    void begin_transaction(std::string label);

    /// Close the innermost group. Commits the step when the outermost closes.
    void commit_transaction();

    /**
     * @brief Abandon the innermost group, reverting whatever it applied
     *
     * Reverts in reverse order, exactly as undo would. Nested aborts unwind to
     * the outermost.
     */
    void abort_transaction();

    [[nodiscard]] bool in_transaction() const { return m_transaction_depth > 0; }

    /// Forget everything. Used on load, and on close.
    void clear();

    [[nodiscard]] size_t undo_depth() const { return m_undo.size(); }
    [[nodiscard]] size_t redo_depth() const { return m_redo.size(); }
    [[nodiscard]] size_t bytes() const { return m_bytes; }

    /**
     * @brief Monotonic counter, bumped on every change to the scene
     *
     * Undo bumps it too: the point is "does this differ from what was saved",
     * not "how many edits have happened". A2 compares this against the revision
     * at save time to decide whether the document is dirty.
     */
    [[nodiscard]] uint64_t revision() const { return m_revision; }

private:
    /// One undo step. Several commands when it came from a transaction.
    struct Step {
        std::vector<CommandPtr> commands;
        std::string label;
        bool sealed = false;
        size_t bytes = 0;
    };

    void push_step(Step step);
    void trim();
    static size_t step_bytes(const Step& step);

    CommandStackConfig m_config;
    std::vector<Step> m_undo;
    std::vector<Step> m_redo;

    /// Commands applied inside the open transaction, innermost last.
    std::vector<CommandPtr> m_pending;
    std::vector<std::string> m_transaction_labels;
    size_t m_transaction_depth = 0;

    size_t m_bytes = 0;
    uint64_t m_revision = 0;
};

} // namespace stratum::scene
