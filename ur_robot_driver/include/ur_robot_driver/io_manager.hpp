#ifndef UR_IO_MANAGER_HPP
#define UR_IO_MANAGER_HPP

#include <atomic>
#include <array>
#include <string>
#include <thread>
#include <unordered_map>

#include <ros/ros.h>

#include <rpwc_msgs/robotIOSignals.h>
#include <rpwc_msgs/setDigitalIOSignal.h>
#include <rpwc_msgs/getDigitalIOSignal.h>

#include <ur_client_library/ur/ur_driver.h>

class IOManager
{
public:
    /**
     * @brief Atomic snapshot of the UR RTDE `actual_digital_input_bits` field.
     *
     * Bit layout (mirrors the RTDE spec, identical for input and output fields):
     *  - bits  0– 7  standard inputs     (pin = bit)
     *  - bits  8–15  configurable inputs (pin = bit − 8)
     *  - bits 16–17  tool inputs         (pin = bit − 16)
     *
     * Written by @c thread_handle_rtde with @c memory_order_relaxed.
     * Read by the publisher thread and the @c get_digital_io_signal service callback.
     * No external mutex is required — all accesses are atomic.
     */
    std::atomic<std::uint64_t> digital_input_bits{0};

    /**
     * @brief Atomic snapshot of the UR RTDE `actual_digital_output_bits` field.
     *
     * Same bit layout as @ref digital_input_bits.
     * Valid only after RTDE communication has started and the first data package has arrived.
     */
    std::atomic<std::uint64_t> digital_output_bits{0};

    /**
     * @brief Constructs the IOManager and loads signal name configuration.
     *
     * If @p json_path is empty or the file cannot be opened or parsed, default names
     * are applied (`standard_in_0` … `tool_out_1`) and a @c ROS_WARN is emitted.
     * The object is ready to use immediately; call start() to begin publishing.
     *
     * @param nh        ROS node handle used to advertise the publisher and services.
     * @param driver    Shared UR driver instance; must outlive this IOManager.
     * @param json_path Absolute path to the IO signal names JSON file (may be empty).
     */
    IOManager(ros::NodeHandle &nh, std::shared_ptr<urcl::UrDriver> driver, const std::string &json_path);

    /** @brief Calls stop(), joining the publisher thread and shutting down all ROS entities. */
    ~IOManager();

    /**
     * @brief Starts the IO publisher thread and advertises the ROS publisher and services.
     *
     * Advertises:
     *  - Latched publisher  @c io_signals_state      (`rpwc_msgs/robotIOSignals`)
     *  - Service            @c set_digital_io_signal (`rpwc_msgs/setDigitalIOSignal`)
     *  - Service            @c get_digital_io_signal (`rpwc_msgs/getDigitalIOSignal`)
     *
     * The publisher fires on every bit-state change, at a maximum rate of @p rate_hz.
     * Calling start() on an already-running manager is a no-op.
     *
     * @param rate_hz  Maximum publish rate in Hz (typically the RTDE loop rate).
     */
    void start(double rate_hz);

    /**
     * @brief Stops the publisher thread and shuts down the publisher and services.
     *
     * Blocks until the publisher thread has joined.  Safe to call multiple times
     * and from any thread.  After stop() returns, start() may be called again.
     */
    void stop();

private:
    // --- Constants ---

    static constexpr size_t kStdCount  = 8;
    static constexpr size_t kCfgCount  = 8;
    static constexpr size_t kToolCount = 2;
    static constexpr size_t kTotal     = kStdCount + kCfgCount + kToolCount; // 18

    // --- Types ---

    // Bit layout (mirrors UR RTDE spec, identical for input and output fields):
    //   bits  0– 7: standard     (pin = bit)
    //   bits  8–15: configurable (pin = bit − 8)
    //   bits 16–17: tool         (pin = bit − 16)

    // Role of a single configurable pin within its side (input or output).
    enum class CfgPinRole { Normal, Safety };

    enum class DigitalKind { StdIn, StdOut, CfgIn, CfgOut, SafetyIn, SafetyOut, ToolIn, ToolOut };

    struct DigitalSignal
    {
        DigitalKind kind;
        uint8_t     pin; // index within its group (0–7 or 0–1)
        size_t      bit; // index in the uint64 field (0–17)
    };

    // --- Functions ---

    bool callbackSetDigital(rpwc_msgs::setDigitalIOSignal::Request &req,
                            rpwc_msgs::setDigitalIOSignal::Response &res);
    bool callbackGetDigital(rpwc_msgs::getDigitalIOSignal::Request &req,
                            rpwc_msgs::getDigitalIOSignal::Response &res);
    void publisherThread();

    void loadDefaults();
    bool loadFromJson(const std::string &path); // false + log on error → caller falls back to defaults
    void buildLookup(const std::array<CfgPinRole, kCfgCount> &cfg_in,
                     const std::array<CfgPinRole, kCfgCount> &cfg_out);

    static std::string defaultInputName(size_t bit);
    static std::string defaultOutputName(size_t bit);

    // --- Variables ---

    std::unordered_map<std::string, DigitalSignal> digital_lookup_;

    // Indexed by bit (0–17); used by publisherThread to build message signal names
    std::array<std::string, kTotal> digital_input_names_, digital_output_names_;

    ros::NodeHandle                 &nh_;
    std::shared_ptr<urcl::UrDriver>  driver_;
    ros::Publisher                   pub_;
    ros::ServiceServer               srv_set_digital_, srv_get_digital_;
    std::thread                      pub_thread_;
    std::atomic<bool>                running_{false};
    double                           rate_hz_{50.0};
};

#endif // UR_IO_MANAGER_HPP
