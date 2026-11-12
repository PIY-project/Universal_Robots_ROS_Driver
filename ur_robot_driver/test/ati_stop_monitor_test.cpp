#include <ur_robot_driver/ati_stop_monitor.hpp>

#include <chrono>
#include <iostream>

int main()
{
    using Clock = AtiStopMonitor::Clock;
    const auto t0 = Clock::now();
    const std::chrono::duration<double> timeout(2.0);
    AtiStopMonitor monitor;

    monitor.updateInputs(0);
    const auto first = monitor.startGoal(t0);
    monitor.updateInputs(1);
    monitor.updateInputs(0);
    if (!monitor.shouldStop(first, timeout, t0 + std::chrono::milliseconds(100)))
    {
        std::cerr << "A short ATI pulse was lost" << std::endl;
        return 1;
    }

    const auto next = monitor.startGoal(t0);
    if (monitor.shouldStop(next, timeout, t0 + std::chrono::seconds(3)))
    {
        std::cerr << "The previous ATI event leaked into a new goal" << std::endl;
        return 1;
    }

    monitor.updateInputs(1);
    const auto high_goal = monitor.startGoal(t0);
    if (monitor.shouldStop(high_goal, timeout, t0 + std::chrono::milliseconds(1999)) ||
        !monitor.shouldStop(high_goal, timeout, t0 + std::chrono::seconds(2)))
    {
        std::cerr << "Persistent HIGH timeout is incorrect" << std::endl;
        return 1;
    }

    monitor.updateInputs(0);
    if (monitor.shouldStop(high_goal, timeout, t0 + std::chrono::seconds(3)))
    {
        std::cerr << "A cleared input still triggered the HIGH timeout" << std::endl;
        return 1;
    }
    monitor.updateInputs(1);
    if (!monitor.shouldStop(high_goal, timeout, t0 + std::chrono::seconds(3)))
    {
        std::cerr << "A new rising edge was not detected" << std::endl;
        return 1;
    }

    return 0;
}
