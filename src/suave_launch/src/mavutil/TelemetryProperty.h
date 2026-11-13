//
// Created by suave on 4/18/24.
//

#ifndef TELEMETRYPROPERTY_H
#define TELEMETRYPROPERTY_H
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <plugins/telemetry/telemetry.h>

#include "../util/Timer.h"
#include "../util/Result.h"

// Caches the latest value of one MAVSDK telemetry stream.
//
// MAVSDK 1.x allows only ONE callback per stream, so anything else that wants the
// stream (e.g. the flight logger) registers here with add_listener() instead of
// calling Telemetry::subscribe_*() again, which would silently replace this one.
template<typename T>
class TelemetryProperty {
public:
    using TCallback = std::function<void(T)>;
    using SubscribeFn = void(mavsdk::Telemetry::*)(TCallback);
    TelemetryProperty(mavsdk::Telemetry& telemetry, SubscribeFn subscribe_fn)
    {
        (telemetry.*subscribe_fn)(std::bind(&TelemetryProperty::set, this, std::placeholders::_1));
    }

    // Called on the MAVSDK thread for every new value.
    void add_listener(TCallback callback)
    {
        std::lock_guard<std::mutex> lock{ m_mutex };
        m_listeners.push_back(std::move(callback));
    }

    // Kept for compatibility with older controllers.
    void set_callback(TCallback callback)
    {
        add_listener(std::move(callback));
    }

    [[nodiscard]]
    Result<T> get() const
    {
        std::lock_guard<std::mutex> lock{ m_mutex };
        if (!m_value)
        {
            return Result<T>("TelemetryProperty::get(), does not have the value yet!");
        }
        return *m_value;
    }

    Result<T> get_within(double elapse_sec = 1.0) const
    {
        if (has_been(elapse_sec))
        {
            return Result<T>("TelemetryProperty::get_within(), does not have the value within " + std::to_string(elapse_sec) + " seconds! Returning the last value.");
        }
        return get();
    }

    Result<T> wait_for_next(const double timeout_sec = 60, const double refresh_rate_hz = 1) const
    {
        Timer timer{};
        timer.start();
        while (has_been(1/refresh_rate_hz))
        {
            if (timer.has_been(timeout_sec))
            {
                return Result<T>("TelemetryProperty::wait_for_next(), timeout after " + std::to_string(timeout_sec) + " seconds!");
            }
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(1/refresh_rate_hz * 1e6 / 10)));
        }
        return get();
    }

private:
    bool has_been(double sec) const
    {
        std::lock_guard<std::mutex> lock{ m_mutex };
        return !m_timer_since_last_update.is_running() || m_timer_since_last_update.has_been(sec);
    }

    void set(T value)
    {
        std::vector<TCallback> listeners;
        {
            std::lock_guard<std::mutex> lock{ m_mutex };
            m_value = value;
            m_timer_since_last_update.start();
            listeners = m_listeners;
        }
        for (auto& listener : listeners)
        {
            listener(value);
        }
    }

    mutable std::mutex m_mutex{};
    Timer m_timer_since_last_update{};
    std::optional<T> m_value{};
    std::vector<TCallback> m_listeners{};
};

#endif //TELEMETRYPROPERTY_H
