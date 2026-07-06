#ifndef UR_ROBOT_STATE_MANAGER_HPP
#define UR_ROBOT_STATE_MANAGER_HPP

// -----------------------------------------
//                Includes
// -----------------------------------------
// System includes
#include <atomic>
#include <functional>
#include <memory>
#include <thread>

// ROS includes
#include <ros/ros.h>

// UR Client Library includes
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/ur/dashboard_client.h>

// -----------------------------------------
//                  Class
// -----------------------------------------

class RobotStateManager
{
public:
    /**
     * @brief Constructs the state manager with driver references and recovery parameters.
     *
     * The object is ready to use immediately; call start() after the RTDE thread
     * is running to begin monitoring.
     *
     * @param nh                           ROS node handle used for parameter reads.
     * @param driver                       Shared UR driver instance; must outlive this object.
     * @param dashboard                    Shared dashboard client; must outlive this object.
     * @param auto_recover_protective_stop If true, automatically unlock and recover from protective stops.
     * @param recovery_timeout_s           Seconds to wait for the program to start after each send attempt.
     * @param recovery_retries             Number of program-send attempts before giving up and shutting down.
     */
    RobotStateManager(ros::NodeHandle &nh, std::shared_ptr<urcl::UrDriver> driver, std::shared_ptr<urcl::DashboardClient> dashboard, bool auto_recover_protective_stop, double recovery_timeout_s, int recovery_retries);

    /** @brief Calls stop(), joining the monitor thread. */
    ~RobotStateManager();

    /**
     * @brief Starts the state monitor thread.
     *
     * Must be called after the RTDE thread is running so that data is
     * available when the monitor first evaluates robot state.
     */
    void start();

    /**
     * @brief Stops the monitor thread, blocking until it has joined.
     *
     * Safe to call multiple times and from any thread.
     */
    void stop();

    /** @brief Returns true when the robot program is running and the bridge may send commands. */
    bool isReady() const { return robot_program_ready_.load(std::memory_order_relaxed); }

    /** @brief Returns true when the current safety mode is a safeguard stop variant. */
    bool isSafeguardActive() const { return isSafeguardMode(rtde_safety_mode_.load(std::memory_order_relaxed)); }

    /**
     * @brief Forces the currently running robot program to stop via the Dashboard Server,
     * even during an active safeguard stop.
     *
     * Unlike @c cancelMotion(), which relies on the (possibly paused) script interpreter
     * reading a reverse-socket message, this goes over the dashboard channel and is honored
     * even while the interpreter is halted. The monitor thread's existing recovery logic
     * will re-upload and restart the program once conditions allow.
     *
     * @return true if the Dashboard Server confirmed the program stopped.
     */
    bool forceProgramStop();

    /**
     * @brief Updates internal RTDE state. Called from @c thread_handle_rtde on every data packet.
     *
     * Thread-safe; writes are @c memory_order_relaxed atomics.
     *
     * @param runtime_state  Value of the RTDE @c runtime_state field (2 = PLAYING).
     * @param robot_mode     Value of the RTDE @c robot_mode field (cast of @c urcl::RobotMode).
     * @param safety_mode    Value of the RTDE @c safety_mode field (cast of @c urcl::SafetyMode).
     */
    void updateRtdeState(uint32_t runtime_state, int32_t robot_mode, int32_t safety_mode);

    /** @brief Confirmation callback from @c handleRobotProgramState — logs robot and safety mode. */
    void onProgramStateChanged(bool running);

    /**
     * @brief Registers a callback invoked the moment the bridge becomes blocked.
     *
     * Not fired on safeguard stops since the robot halts naturally in that case.
     * Typical use: call @c ur_instruction_executor_->cancelMotion() to unblock
     * any in-flight @c executeMotion() and allow the active action goal to abort.
     *
     * @param callback  Callable with signature @c void().
     */
    void setOnBlockedCallback(std::function<void()> callback);

private:
    // --- Methods ---

    void monitorThread();
    bool attemptRecovery();
    static bool isUnrecoverableSafetyMode(int32_t safety_mode);
    static bool isSafeguardMode(int32_t safety_mode);

    // --- Variables ---

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

#endif // UR_ROBOT_STATE_MANAGER_HPP
