// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/log_sink.hpp"

#include <spdlog/details/os.h>

#include <cstdio>
#include <ctime>
#include <utility>

namespace stratum {

LogRing::LogRing(size_t capacity)
    : m_lines(capacity > 0 ? capacity : 1), m_capacity(capacity > 0 ? capacity : 1) {}

void LogRing::push(LogLine line) {
    std::lock_guard<std::mutex> lock(m_mutex);
    const size_t tail = (m_head + m_count) % m_capacity;
    m_lines[tail] = std::move(line);
    if (m_count < m_capacity) {
        ++m_count;
    } else {
        // Already full: the slot just overwritten WAS the oldest line, so the
        // new oldest is the next one around.
        m_head = (m_head + 1) % m_capacity;
    }
}

void LogRing::clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_head = 0;
    m_count = 0;
}

void LogRing::for_each(const std::function<void(const LogLine&)>& fn) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (size_t i = 0; i < m_count; ++i) {
        fn(m_lines[(m_head + i) % m_capacity]);
    }
}

size_t LogRing::size() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_count;
}

namespace {

LogLine::Level to_line_level(spdlog::level::level_enum lvl) {
    switch (lvl) {
        case spdlog::level::warn:
            return LogLine::Warn;
        case spdlog::level::err:
        case spdlog::level::critical:
            return LogLine::Error;
        default:
            // trace, debug, info, off (should never log) all read as Info --
            // there is no dimmer tier in the console panel.
            return LogLine::Info;
    }
}

// spdlog's own pattern formatter can do this too, but it means constructing a
// pattern_formatter per sink just to get "%H:%M:%S", and os::localtime() is
// already the thread-safe (localtime_r/localtime_s) primitive that formatter
// uses internally. Calling it directly avoids the extra object with no loss
// of thread safety.
std::string format_time(spdlog::log_clock::time_point tp) {
    const std::time_t tt = spdlog::log_clock::to_time_t(tp);
    const std::tm tm = spdlog::details::os::localtime(tt);
    char buf[16];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm);
    return std::string(buf);
}

}  // namespace

template <typename Mutex>
void RingSink<Mutex>::sink_it_(const spdlog::details::log_msg& msg) {
    LogLine line;
    line.level = to_line_level(msg.level);
    line.time = format_time(msg.time);
    line.text.assign(msg.payload.data(), msg.payload.size());
    m_ring.push(std::move(line));
}

template class RingSink<std::mutex>;

}  // namespace stratum
