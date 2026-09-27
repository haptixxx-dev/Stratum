// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#pragma once

#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <spdlog/sinks/base_sink.h>

namespace stratum {

/// One line of the in-editor console. Deliberately spdlog-free so the UI side
/// (console_panel.cpp) never needs to touch spdlog headers or types.
struct LogLine {
    enum Level { Info, Warn, Error };

    Level level = Info;
    std::string time;  // "HH:MM:SS", formatted once at push time
    std::string text;  // the formatted message, no trailing newline
};

/// Fixed-capacity ring buffer of LogLine.
///
/// Push happens on whichever thread logs -- the spdlog sink below runs on the
/// logging call's own thread, and several worker threads in this codebase log
/// directly (road build, export, GPU pool). Iteration happens on the UI
/// thread inside Editor::draw_console(). Both go through m_mutex, so a push
/// mid-frame cannot hand the UI a half-written line or a torn read of
/// m_head/m_count.
class LogRing {
public:
    explicit LogRing(size_t capacity = 2000);

    void push(LogLine line);
    void clear();

    /// Calls fn(line) for every stored line, oldest first. fn runs under the
    /// ring's lock, so it must not call back into this LogRing.
    void for_each(const std::function<void(const LogLine&)>& fn) const;

    size_t size() const;
    size_t capacity() const { return m_capacity; }

private:
    mutable std::mutex m_mutex;
    std::vector<LogLine> m_lines;  // m_capacity slots, used as a circular buffer
    size_t m_capacity;
    size_t m_head = 0;   // slot holding the oldest line
    size_t m_count = 0;  // number of valid lines currently stored
};

/// spdlog sink that pushes every logged record into a LogRing, so the console
/// panel shows exactly what went to the logger rather than a parallel stream
/// someone has to remember to update by hand.
///
/// The LogRing is owned elsewhere (Editor) and must outlive this sink; the
/// sink only keeps a reference. Mutex-templated like every other spdlog sink,
/// but this codebase only ever registers it on the default (multithreaded)
/// logger, so only the std::mutex instantiation is compiled.
template <typename Mutex>
class RingSink : public spdlog::sinks::base_sink<Mutex> {
public:
    explicit RingSink(LogRing& ring) : m_ring(ring) {}

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override;
    void flush_() override {}

private:
    LogRing& m_ring;
};

extern template class RingSink<std::mutex>;
using RingSinkMt = RingSink<std::mutex>;

}  // namespace stratum
