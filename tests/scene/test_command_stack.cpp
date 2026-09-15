// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_command_stack.cpp
 * @brief CommandStack: ordering, merging, transactions, bounds and refusal
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The stack has no callers yet -- it lands before the edit features that will
 * use it, which is the whole point of A4 -- so these tests are the only thing
 * holding its contract. They are written against the guarantees the header
 * makes, not against the implementation, so a rewrite of the internals should
 * leave them alone.
 *
 * The fixture is a command over an int. That is not laziness: every property
 * worth checking here is about ORDER and BOOKKEEPING, and an int makes an
 * out-of-order revert visible as a wrong number rather than as a plausible-
 * looking mesh.
 */

#include "framework.hpp"

#include "scene/command.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using stratum::scene::Command;
using stratum::scene::CommandPtr;
using stratum::scene::CommandStack;
using stratum::scene::CommandStackConfig;

namespace {

/// Appends a character on apply and removes it on revert.
///
/// A log rather than a counter, because a counter cannot tell "undone in the
/// wrong order" from "undone correctly".
class Append : public Command {
public:
    Append(std::string* log, char token, bool succeeds = true)
        : m_log(log), m_token(token), m_succeeds(succeeds) {}

    bool apply() override {
        if (!m_succeeds) return false;
        m_log->push_back(m_token);
        ++m_applied;
        return true;
    }

    void revert() override {
        if (!m_log->empty()) m_log->pop_back();
        ++m_reverted;
    }

    std::string describe() const override { return std::string("Append ") + m_token; }

    int applied() const { return m_applied; }
    int reverted() const { return m_reverted; }

private:
    std::string* m_log;
    char m_token;
    bool m_succeeds;
    int m_applied = 0;
    int m_reverted = 0;
};

/// Absorbs any other Slide, as a drag does.
class Slide : public Command {
public:
    Slide(int* value, int delta) : m_value(value), m_delta(delta) {}

    bool apply() override {
        *m_value += m_delta;
        return true;
    }
    void revert() override { *m_value -= m_delta; }
    std::string describe() const override { return "Slide"; }

    bool merge(const Command& next) override {
        const auto* other = dynamic_cast<const Slide*>(&next);
        if (other == nullptr) return false;
        // The successor already applied, so its delta is already in the value.
        // Taking it over means this command's revert must now remove both.
        m_delta += other->m_delta;
        return true;
    }

private:
    int* m_value;
    int m_delta;
};

/// Reports a fixed footprint, for the memory bound.
class Heavy : public Command {
public:
    explicit Heavy(size_t bytes) : m_bytes(bytes) {}
    bool apply() override { return true; }
    void revert() override {}
    std::string describe() const override { return "Heavy"; }
    size_t footprint() const override { return m_bytes; }

private:
    size_t m_bytes;
};

CommandPtr append(std::string* log, char token, bool succeeds = true) {
    return std::make_unique<Append>(log, token, succeeds);
}

} // namespace

// ============================================================================
// The basics
// ============================================================================

TEST(CommandStack, an_executed_command_is_applied_once_and_recorded) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'a')));

    CHECK_EQ(log, std::string("a"));
    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_TRUE(stack.can_undo());
    CHECK_FALSE(stack.can_redo());
}

TEST(CommandStack, undo_then_redo_returns_the_same_state) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    CHECK_EQ(log, std::string("ab"));

    CHECK_TRUE(stack.undo());
    CHECK_EQ(log, std::string("a"));
    CHECK_TRUE(stack.undo());
    CHECK_EQ(log, std::string(""));
    CHECK_FALSE(stack.can_undo());

    CHECK_TRUE(stack.redo());
    CHECK_TRUE(stack.redo());
    CHECK_EQ(log, std::string("ab"));
    CHECK_FALSE(stack.can_redo());
}

TEST(CommandStack, undo_and_redo_on_an_empty_stack_are_no_ops) {
    CommandStack stack;

    CHECK_FALSE(stack.undo());
    CHECK_FALSE(stack.redo());
    CHECK_EQ(stack.undo_label(), std::string(""));
    CHECK_EQ(stack.redo_label(), std::string(""));
}

TEST(CommandStack, labels_name_the_step_each_direction_would_take) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_EQ(stack.undo_label(), std::string("Append a"));

    CHECK_TRUE(stack.undo());
    CHECK_EQ(stack.redo_label(), std::string("Append a"));
    CHECK_EQ(stack.undo_label(), std::string(""));
}

// ============================================================================
// Refusal
// ============================================================================
//
// apply() returning false has to leave the history untouched -- INCLUDING the
// redo branch. An edit the user could not make is not an edit, and silently
// discarding their redo for it would be a second surprise on top of the first.

TEST(CommandStack, a_refused_command_records_nothing) {
    std::string log;
    CommandStack stack;

    CHECK_FALSE(stack.execute(append(&log, 'x', /*succeeds=*/false)));

    CHECK_EQ(log, std::string(""));
    CHECK_EQ(stack.undo_depth(), size_t{0});
    CHECK_FALSE(stack.can_undo());
}

TEST(CommandStack, a_refused_command_does_not_discard_the_redo_branch) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.undo());
    CHECK_TRUE(stack.can_redo());

    CHECK_FALSE(stack.execute(append(&log, 'x', /*succeeds=*/false)));

    CHECK_TRUE(stack.can_redo());
    CHECK_TRUE(stack.redo());
    CHECK_EQ(log, std::string("a"));
}

TEST(CommandStack, a_null_command_is_refused) {
    CommandStack stack;
    CHECK_FALSE(stack.execute(nullptr));
    CHECK_EQ(stack.undo_depth(), size_t{0});
}

// ============================================================================
// The redo branch
// ============================================================================

TEST(CommandStack, a_new_edit_after_undo_discards_the_redo_branch) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    CHECK_TRUE(stack.undo());
    CHECK_TRUE(stack.can_redo());

    CHECK_TRUE(stack.execute(append(&log, 'c')));

    CHECK_FALSE(stack.can_redo());
    CHECK_EQ(log, std::string("ac"));
}

// A merge is still a new edit. Leaving a stale redo branch alive behind one is
// how redo comes to restore state from a history that no longer exists.
TEST(CommandStack, a_merged_edit_after_undo_also_discards_the_redo_branch) {
    int value = 0;
    CommandStack stack;

    CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 1)));
    CHECK_TRUE(stack.undo());
    CHECK_TRUE(stack.can_redo());

    CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 5)));

    CHECK_FALSE(stack.can_redo());
}

// ============================================================================
// Coalescing
// ============================================================================

TEST(CommandStack, a_drag_collapses_into_one_undo_step) {
    int value = 0;
    CommandStack stack;

    for (int i = 0; i < 20; ++i) {
        CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 1)));
    }

    CHECK_EQ(value, 20);
    CHECK_EQ(stack.undo_depth(), size_t{1});

    CHECK_TRUE(stack.undo());
    CHECK_EQ(value, 0);
}

TEST(CommandStack, seal_ends_the_gesture) {
    int value = 0;
    CommandStack stack;

    CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 3)));
    stack.seal();
    CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 4)));

    CHECK_EQ(stack.undo_depth(), size_t{2});
    CHECK_TRUE(stack.undo());
    CHECK_EQ(value, 3);
}

TEST(CommandStack, a_command_that_refuses_to_merge_starts_its_own_step) {
    std::string log;
    int value = 0;
    CommandStack stack;

    CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 2)));
    CHECK_TRUE(stack.execute(append(&log, 'a')));

    CHECK_EQ(stack.undo_depth(), size_t{2});
}

// Merging must never reach past the most recent step into an older one.
TEST(CommandStack, a_merge_cannot_reach_across_an_intervening_edit) {
    std::string log;
    int value = 0;
    CommandStack stack;

    CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 2)));
    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.execute(std::make_unique<Slide>(&value, 5)));

    CHECK_EQ(stack.undo_depth(), size_t{3});
    CHECK_TRUE(stack.undo());
    CHECK_EQ(value, 2);
}

// ============================================================================
// Transactions
// ============================================================================

TEST(CommandStack, a_transaction_undoes_as_one_step) {
    std::string log;
    CommandStack stack;

    stack.begin_transaction("Delete block");
    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    CHECK_TRUE(stack.execute(append(&log, 'c')));
    stack.commit_transaction();

    CHECK_EQ(log, std::string("abc"));
    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_EQ(stack.undo_label(), std::string("Delete block"));

    CHECK_TRUE(stack.undo());
    CHECK_EQ(log, std::string(""));

    CHECK_TRUE(stack.redo());
    CHECK_EQ(log, std::string("abc"));
}

TEST(CommandStack, nesting_produces_one_step_labelled_by_the_outermost) {
    std::string log;
    CommandStack stack;

    stack.begin_transaction("Outer");
    CHECK_TRUE(stack.execute(append(&log, 'a')));
    stack.begin_transaction("Inner");
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    stack.commit_transaction();
    CHECK_TRUE(stack.in_transaction());
    CHECK_TRUE(stack.execute(append(&log, 'c')));
    stack.commit_transaction();

    CHECK_FALSE(stack.in_transaction());
    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_EQ(stack.undo_label(), std::string("Outer"));
}

TEST(CommandStack, an_aborted_transaction_leaves_no_trace) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'z')));

    stack.begin_transaction("Doomed");
    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    stack.abort_transaction();

    CHECK_EQ(log, std::string("z"));
    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_EQ(stack.undo_label(), std::string("Append z"));
}

TEST(CommandStack, an_empty_transaction_is_not_a_step) {
    CommandStack stack;

    stack.begin_transaction("Clicked nothing");
    stack.commit_transaction();

    CHECK_EQ(stack.undo_depth(), size_t{0});
    CHECK_FALSE(stack.can_undo());
}

// Undo mid-transaction would take back a step the open group is building on.
TEST(CommandStack, undo_is_refused_while_a_transaction_is_open) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'a')));

    stack.begin_transaction("Open");
    CHECK_FALSE(stack.can_undo());
    CHECK_FALSE(stack.undo());
    CHECK_FALSE(stack.can_redo());
    stack.commit_transaction();

    CHECK_TRUE(stack.can_undo());
}

TEST(CommandStack, a_nested_abort_unwinds_the_whole_group) {
    std::string log;
    CommandStack stack;

    stack.begin_transaction("Outer");
    CHECK_TRUE(stack.execute(append(&log, 'a')));
    stack.begin_transaction("Inner");
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    stack.abort_transaction();

    CHECK_FALSE(stack.in_transaction());
    CHECK_EQ(log, std::string(""));
    CHECK_EQ(stack.undo_depth(), size_t{0});
}

// ============================================================================
// Ordering
// ============================================================================
//
// The one property that cannot be recovered once it is wrong: a transaction that
// created a lot and then set its zoning must unset the zoning BEFORE the lot
// goes away.

TEST(CommandStack, a_transaction_reverts_in_reverse_order) {
    std::string log;
    CommandStack stack;

    stack.begin_transaction("Three");
    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    CHECK_TRUE(stack.execute(append(&log, 'c')));
    stack.commit_transaction();

    // Append removes the LAST character, so reverting out of order would leave
    // the wrong letters standing rather than an empty string.
    CHECK_TRUE(stack.undo());
    CHECK_EQ(log, std::string(""));
}

TEST(CommandStack, a_transaction_redoes_in_forward_order) {
    std::string log;
    CommandStack stack;

    stack.begin_transaction("Three");
    CHECK_TRUE(stack.execute(append(&log, 'a')));
    CHECK_TRUE(stack.execute(append(&log, 'b')));
    CHECK_TRUE(stack.execute(append(&log, 'c')));
    stack.commit_transaction();

    CHECK_TRUE(stack.undo());
    CHECK_TRUE(stack.redo());
    CHECK_EQ(log, std::string("abc"));
}

// ============================================================================
// Bounds
// ============================================================================

TEST(CommandStack, depth_is_bounded_and_drops_the_oldest) {
    std::string log;
    CommandStackConfig cfg;
    cfg.max_depth = 3;
    CommandStack stack(cfg);

    for (char c : {'a', 'b', 'c', 'd', 'e'}) {
        CHECK_TRUE(stack.execute(append(&log, c)));
        stack.seal();
    }

    CHECK_EQ(log, std::string("abcde"));
    CHECK_EQ(stack.undo_depth(), size_t{3});

    // The three newest survive; 'a' and 'b' can no longer be undone.
    CHECK_TRUE(stack.undo());
    CHECK_TRUE(stack.undo());
    CHECK_TRUE(stack.undo());
    CHECK_FALSE(stack.can_undo());
    CHECK_EQ(log, std::string("ab"));
}

TEST(CommandStack, memory_is_bounded_and_drops_the_oldest) {
    CommandStackConfig cfg;
    cfg.max_depth = 1000;
    cfg.max_bytes = 1000;
    CommandStack stack(cfg);

    for (int i = 0; i < 6; ++i) {
        CHECK_TRUE(stack.execute(std::make_unique<Heavy>(300)));
        stack.seal();
    }

    CHECK((stack.bytes()) <= (size_t{1000}));
    CHECK((stack.undo_depth()) <= (size_t{3}));
    CHECK((stack.undo_depth()) > (size_t{0}));
}

// Dropping the only step to stay under the ceiling would leave the stack unable
// to hold the edit that was just made, which is worse than being over budget.
TEST(CommandStack, a_single_oversized_command_is_still_kept) {
    CommandStackConfig cfg;
    cfg.max_bytes = 100;
    CommandStack stack(cfg);

    CHECK_TRUE(stack.execute(std::make_unique<Heavy>(50'000)));

    CHECK_EQ(stack.undo_depth(), size_t{1});
    CHECK_TRUE(stack.can_undo());
}

// ============================================================================
// Revision
// ============================================================================

TEST(CommandStack, the_revision_advances_on_every_change_including_undo) {
    std::string log;
    CommandStack stack;

    const uint64_t start = stack.revision();

    CHECK_TRUE(stack.execute(append(&log, 'a')));
    const uint64_t after_edit = stack.revision();
    CHECK((after_edit) > (start));

    CHECK_TRUE(stack.undo());
    const uint64_t after_undo = stack.revision();
    CHECK((after_undo) > (after_edit));

    CHECK_TRUE(stack.redo());
    CHECK((stack.revision()) > (after_undo));
}

TEST(CommandStack, a_refused_command_does_not_advance_the_revision) {
    std::string log;
    CommandStack stack;

    const uint64_t before = stack.revision();
    CHECK_FALSE(stack.execute(append(&log, 'x', /*succeeds=*/false)));

    CHECK_EQ(stack.revision(), before);
}

TEST(CommandStack, clear_forgets_the_history_but_not_the_revision) {
    std::string log;
    CommandStack stack;

    CHECK_TRUE(stack.execute(append(&log, 'a')));
    const uint64_t before = stack.revision();

    stack.clear();

    CHECK_EQ(stack.undo_depth(), size_t{0});
    CHECK_EQ(stack.redo_depth(), size_t{0});
    CHECK_EQ(stack.bytes(), size_t{0});
    CHECK_FALSE(stack.can_undo());
    // A load that reset this to zero would make a freshly loaded document
    // compare equal to an unsaved one that happened to sit at zero too.
    CHECK((stack.revision()) > (before));
}
