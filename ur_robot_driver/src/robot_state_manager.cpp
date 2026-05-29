#include <ur_robot_driver/robot_state_manager.hpp>

#include <ur_client_library/types.h>
#include <ur_client_library/ur/datatypes.h>

RobotStateManager::RobotStateManager(ros::NodeHandle &nh, std::shared_ptr<urcl::UrDriver> driver, std::shared_ptr<urcl::DashboardClient> dashboard, bool auto_recover_protective_stop, double recovery_timeout_s, int recovery_retries)
    : nh_(nh), driver_(std::move(driver)), dashboard_(std::move(dashboard)), auto_recover_protective_stop_(auto_recover_protective_stop), recovery_timeout_s_(recovery_timeout_s), recovery_retries_(recovery_retries)
{
}

RobotStateManager::~RobotStateManager()
{
    stop();
}

void RobotStateManager::start()
{
    running_ = true;
    monitor_thread_ = std::thread(&RobotStateManager::monitorThread, this);
}

void RobotStateManager::stop()
{
    running_ = false;
    if (monitor_thread_.joinable())
        monitor_thread_.join();
}

void RobotStateManager::updateRtdeState(uint32_t runtime_state, int32_t robot_mode, int32_t safety_mode)
{
    rtde_runtime_state_.store(runtime_state, std::memory_order_relaxed);
    rtde_robot_mode_.store(robot_mode, std::memory_order_relaxed);
    rtde_safety_mode_.store(safety_mode, std::memory_order_relaxed);
    rtde_initialized_.store(true, std::memory_order_relaxed);
}

void RobotStateManager::setOnBlockedCallback(std::function<void()> callback)
{
    on_blocked_callback_ = std::move(callback);
}

void RobotStateManager::onProgramStateChanged(bool running)
{
    std::string robot_mode_str, safety_mode_str;
    dashboard_->commandRobotMode(robot_mode_str);
    dashboard_->commandSafetyMode(safety_mode_str);
    ROS_INFO_STREAM("[StateManager]: Program state confirmation: " << (running ? "RUNNING" : "STOPPED") << " | robot_mode=" << robot_mode_str << " | safety_mode=" << safety_mode_str);
}

bool RobotStateManager::isSafeguardMode(int32_t safety_mode)
{
    return safety_mode == urcl::toUnderlying(urcl::SafetyMode::SAFEGUARD_STOP) ||
           safety_mode == urcl::toUnderlying(urcl::SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP) ||
           safety_mode == urcl::toUnderlying(urcl::SafetyMode::SAFETY_API_SAFEGUARD_STOP);
}

bool RobotStateManager::isUnrecoverableSafetyMode(int32_t safety_mode)
{
    return safety_mode == urcl::toUnderlying(urcl::SafetyMode::SYSTEM_EMERGENCY_STOP) ||
           safety_mode == urcl::toUnderlying(urcl::SafetyMode::ROBOT_EMERGENCY_STOP) ||
           safety_mode == urcl::toUnderlying(urcl::SafetyMode::VIOLATION) ||
           safety_mode == urcl::toUnderlying(urcl::SafetyMode::FAULT);
}

bool RobotStateManager::attemptRecovery()
{
    int32_t safety_mode = rtde_safety_mode_.load(std::memory_order_relaxed);

    if (safety_mode == urcl::toUnderlying(urcl::SafetyMode::PROTECTIVE_STOP))
    {
        if (!auto_recover_protective_stop_)
        {
            ROS_ERROR("[StateManager]: Protective stop active, auto-recovery disabled (set auto_recover_protective_stop:=true to enable)");
            return false;
        }
        ROS_WARN("[StateManager]: Protective stop active, attempting to unlock");
        if (!dashboard_->commandUnlockProtectiveStop())
        {
            ROS_ERROR("[StateManager]: Failed to unlock protective stop");
            return false;
        }
    }

    if (isSafeguardMode(safety_mode))
    {
        ROS_WARN_STREAM("[StateManager]: Safeguard stop active (" << urcl::safetyModeString(static_cast<urcl::SafetyMode>(safety_mode)) << "), waiting for clearance");
        while (running_ && ros::ok())
        {
            int32_t current = rtde_safety_mode_.load(std::memory_order_relaxed);
            if (!isSafeguardMode(current))
                break;
            ros::Duration(0.2).sleep();
        }
        ROS_INFO("[StateManager]: Safeguard cleared, continuing recovery");

        ros::Duration(0.5).sleep();

        if (rtde_runtime_state_.load(std::memory_order_relaxed) == 2)
            return true;

        // Re-check safety mode — PLC may have asserted a new condition during the transition
        int32_t post_safeguard_safety = rtde_safety_mode_.load(std::memory_order_relaxed);
        if (isUnrecoverableSafetyMode(post_safeguard_safety) || isSafeguardMode(post_safeguard_safety))
        {
            ROS_WARN_STREAM("[StateManager]: Safety mode changed during safeguard recovery: "
                << urcl::safetyModeString(static_cast<urcl::SafetyMode>(post_safeguard_safety))
                << " — aborting attempt, will re-evaluate");
            return false;
        }
    }

    // Verify motors are on before sending the program
    int32_t robot_mode = rtde_robot_mode_.load(std::memory_order_relaxed);

    if (robot_mode == urcl::toUnderlying(urcl::RobotMode::POWER_OFF))
    {
        ROS_WARN("[StateManager]: Robot is powered off, attempting to power on");
        if (!dashboard_->commandPowerOn())
        {
            ROS_ERROR("[StateManager]: Failed to power on robot");
            return false;
        }
        ros::Time wait_start = ros::Time::now();
        while ((ros::Time::now() - wait_start).toSec() < recovery_timeout_s_)
        {
            robot_mode = rtde_robot_mode_.load(std::memory_order_relaxed);
            if (robot_mode == urcl::toUnderlying(urcl::RobotMode::IDLE))
                break;
            ros::Duration(0.2).sleep();
        }
    }

    if (robot_mode == urcl::toUnderlying(urcl::RobotMode::IDLE))
    {
        ROS_WARN("[StateManager]: Robot is idle, releasing brakes");
        if (!dashboard_->commandBrakeRelease())
        {
            ROS_ERROR("[StateManager]: Failed to release brakes");
            return false;
        }
        ros::Time wait_start = ros::Time::now();
        while ((ros::Time::now() - wait_start).toSec() < recovery_timeout_s_)
        {
            robot_mode = rtde_robot_mode_.load(std::memory_order_relaxed);
            if (robot_mode == urcl::toUnderlying(urcl::RobotMode::RUNNING))
                break;
            int32_t cur_safety = rtde_safety_mode_.load(std::memory_order_relaxed);
            if (cur_safety != urcl::toUnderlying(urcl::SafetyMode::NORMAL) &&
                cur_safety != urcl::toUnderlying(urcl::SafetyMode::REDUCED))
                break;
            ros::Duration(0.2).sleep();
        }
    }

    if (robot_mode != urcl::toUnderlying(urcl::RobotMode::RUNNING))
    {
        ROS_ERROR_STREAM("[StateManager]: Robot in unexpected mode: " << urcl::robotModeString(static_cast<urcl::RobotMode>(robot_mode)) << " — cannot restart program");
        return false;
    }

    for (int attempt = 1; attempt <= recovery_retries_; ++attempt)
    {
        ROS_INFO_STREAM("[StateManager]: Sending robot program (attempt " << attempt << "/" << recovery_retries_ << ")");
        if (!driver_->sendRobotProgram())
        {
            ROS_WARN("[StateManager]: sendRobotProgram failed");
            continue;
        }

        ros::Time wait_start = ros::Time::now();
        while ((ros::Time::now() - wait_start).toSec() < recovery_timeout_s_)
        {
            if (rtde_runtime_state_.load(std::memory_order_relaxed) == 2)
                return true;
            ros::Duration(0.05).sleep();
        }

        ROS_WARN_STREAM("[StateManager]: Program did not start within " << recovery_timeout_s_ << "s (attempt " << attempt << "/" << recovery_retries_ << ")");
    }

    return false;
}

void RobotStateManager::monitorThread()
{
    ros::Rate rate(5.0);

    while (running_ && ros::ok())
    {
        if (!rtde_initialized_.load(std::memory_order_relaxed))
        {
            rate.sleep();
            continue;
        }

        uint32_t rt_state = rtde_runtime_state_.load(std::memory_order_relaxed);
        int32_t safety_mode = rtde_safety_mode_.load(std::memory_order_relaxed);

        if (rt_state == 2)
        {
            if (!robot_program_ready_.load(std::memory_order_relaxed))
            {
                ROS_INFO("[StateManager]: Robot program running, bridge ready");
                robot_program_ready_.store(true, std::memory_order_relaxed);
            }
            rate.sleep();
            continue;
        }

        if (robot_program_ready_.load(std::memory_order_relaxed))
        {
            ROS_WARN("[StateManager]: Robot program stopped, bridge blocked");
            robot_program_ready_.store(false, std::memory_order_relaxed);

            bool is_safeguard = isSafeguardMode(safety_mode);
            if (!is_safeguard && on_blocked_callback_)
                on_blocked_callback_();
        }

        if (isUnrecoverableSafetyMode(safety_mode))
        {
            ROS_ERROR_STREAM_THROTTLE(5.0, "[StateManager]: Safety mode requires manual intervention: " << urcl::safetyModeString(static_cast<urcl::SafetyMode>(safety_mode)));
            ros::Duration(1.0).sleep();
            continue;
        }

        if (!attemptRecovery())
        {
            int32_t cur_safety = rtde_safety_mode_.load(std::memory_order_relaxed);
            if (cur_safety != urcl::toUnderlying(urcl::SafetyMode::NORMAL) &&
                cur_safety != urcl::toUnderlying(urcl::SafetyMode::REDUCED))
            {
                rate.sleep();
                continue;
            }
            ROS_FATAL("[StateManager]: All recovery attempts failed, shutting down");
            ros::requestShutdown();
            return;
        }
    }
}
