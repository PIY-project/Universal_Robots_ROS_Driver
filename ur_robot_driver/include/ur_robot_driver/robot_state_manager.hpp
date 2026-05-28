#pragma once
#include <atomic>
#include <thread>
#include <memory>
#include <ros/ros.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/datatypes.h>

class RobotStateManager
{
public:
    /// @brief Constructs the state manager with driver references and recovery parameters.
    RobotStateManager(ros::NodeHandle &nh, std::shared_ptr<urcl::UrDriver> driver, std::shared_ptr<urcl::DashboardClient> dashboard, bool auto_recover_protective_stop, double recovery_timeout_s, int recovery_retries);

    ~RobotStateManager();

    /// @brief Starts the monitor thread. Call after the RTDE thread is running.
    void start();

    /// @brief Stops the monitor thread. Called on node shutdown.
    void stop();

    /// @brief Returns true when the robot program is confirmed running and the bridge may send commands.
    bool isReady() const { return robot_program_ready_.load(std::memory_order_relaxed); }

    /// @brief Returns true when the current safety mode is a safeguard stop variant.
    bool isSafeguardActive() const;

    /// @brief Called from thread_handle_rtde on every RTDE packet to update internal state.
    void updateRtdeState(uint32_t runtime_state, int32_t robot_mode, int32_t safety_mode);

    /// @brief Called from handleRobotProgramState for logging/confirmation only.
    void onProgramStateChanged(bool running);

    /// @brief Registers a callback invoked the moment the bridge becomes blocked. Use to cancel in-flight motion.
    void setOnBlockedCallback(std::function<void()> callback);

private:
    void monitorThread();
    bool attemptRecovery();
    static bool isUnrecoverableSafetyMode(int32_t safety_mode);

    ros::NodeHandle &nh_;
    std::shared_ptr<urcl::UrDriver> driver_;
    std::shared_ptr<urcl::DashboardClient> dashboard_;

    std::atomic<bool> robot_program_ready_{false};
    std::atomic<bool> rtde_initialized_{false};
    std::atomic<uint32_t> rtde_runtime_state_{0};
    std::atomic<int32_t> rtde_robot_mode_{0};
    std::atomic<int32_t> rtde_safety_mode_{0};

    std::atomic<bool> running_{false};
    std::thread monitor_thread_;
    std::function<void()> on_blocked_callback_;

    bool auto_recover_protective_stop_;
    double recovery_timeout_s_;
    int recovery_retries_;
};
