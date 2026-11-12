#ifndef UR_ROBOT_DRIVER_ATI_STOP_MONITOR_HPP
#define UR_ROBOT_DRIVER_ATI_STOP_MONITOR_HPP

#include <chrono>
#include <cstdint>
#include <mutex>

// DI0 is wired to the ATI threshold relay. The UR controller performs the
// physical stop; this class only attributes the observed event to a ROS goal.
class AtiStopMonitor
{
public:
    using Clock = std::chrono::steady_clock;

    struct Goal
    {
        std::uint64_t rising_edges;
        bool started_high;
        Clock::time_point started_at;
    };

    void updateInputs(std::uint64_t input_bits)
    {
        const bool high = (input_bits & 1ULL) != 0;
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_ && !high_ && high)
            ++rising_edges_;
        high_ = high;
        initialized_ = true;
    }

    Goal startGoal(Clock::time_point now = Clock::now()) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return {rising_edges_, high_, now};
    }

    bool shouldStop(const Goal &goal, std::chrono::duration<double> high_timeout,
                    Clock::time_point now = Clock::now()) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return rising_edges_ != goal.rising_edges ||
               (goal.started_high && high_ && now - goal.started_at >= high_timeout);
    }

private:
    mutable std::mutex mutex_;
    bool initialized_{false};
    bool high_{false};
    std::uint64_t rising_edges_{0};
};

#endif // UR_ROBOT_DRIVER_ATI_STOP_MONITOR_HPP
