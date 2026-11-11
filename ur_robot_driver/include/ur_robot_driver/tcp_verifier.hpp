#ifndef UR_ROBOT_DRIVER_TCP_VERIFIER_HPP
#define UR_ROBOT_DRIVER_TCP_VERIFIER_HPP

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

// This class deliberately has no ROS or URCL dependencies so controller-state
// transitions and TCP comparison can be tested without a robot.
struct BridgeRtdeSample
{
    using Vector6 = std::array<double, 6>;

    double controller_timestamp = 0.0;
    Vector6 actual_q{};
    Vector6 actual_qd{};
    Vector6 tcp_offset{};
    std::uint32_t runtime_state = 0;
    std::int32_t robot_mode = 0;
    std::int32_t safety_mode = 0;
    std::uint64_t sequence = 0;
    std::uint64_t program_epoch = 0;
    std::chrono::steady_clock::time_point received_at{};
};

class TcpVerifier
{
public:
    using Vector6 = BridgeRtdeSample::Vector6;
    enum class Event { NONE, PROGRAM_STARTED, PROGRAM_STOPPED, TCP_CHANGED, TIMESTAMP_RESET,
                       DUPLICATE_SAMPLE, INVALID_SAMPLE };
    enum class WaitResult { VERIFIED, INTERRUPTED, FAULT, TIMEOUT };

    explicit TcpVerifier(const Vector6& expected, std::chrono::milliseconds max_sample_age);

    static Vector6 quantizeForUrScript(const Vector6& tcp);
    static bool equivalent(const Vector6& expected, const Vector6& actual,
                           double translation_tolerance_m = 3e-6,
                           double rotation_tolerance_rad = 3e-6);

    Event observe(BridgeRtdeSample& sample);
    bool onNoPackage();
    void onProgramDisconnected();
    std::uint64_t claimPendingEpoch();
    bool prepareReconfiguration(const Vector6& expected, std::uint64_t epoch);
    bool beginVerification(std::uint64_t epoch, std::uint64_t after_sequence);
    WaitResult waitForVerification(std::uint64_t epoch, std::chrono::milliseconds timeout);
    void fail(std::uint64_t epoch, const std::string& reason);
    void failReconfiguration(std::uint64_t epoch, const Vector6& previous_expected,
                             const std::string& reason);
    void waitForChange(std::chrono::milliseconds timeout);
    bool motionReady() const;
    bool isFault() const;
    bool isVerified() const;
    bool programRunning() const;
    bool snapshot(BridgeRtdeSample& sample) const;
    std::uint64_t sequence() const;
    std::uint64_t epoch() const;
    std::string faultReason() const;
    Vector6 expected() const;

private:
    void invalidateLocked();
    bool freshLocked(std::chrono::steady_clock::time_point now) const;

    Vector6 expected_;
    const std::chrono::milliseconds max_sample_age_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    BridgeRtdeSample latest_;
    bool have_sample_ = false;
    bool program_running_ = false;
    bool verified_ = false;
    bool verifying_ = false;
    bool fault_ = false;
    std::string fault_reason_;
    std::uint64_t sequence_ = 0;
    std::uint64_t epoch_ = 0;
    std::uint64_t claimed_epoch_ = 0;
    std::uint64_t verify_after_sequence_ = 0;
    unsigned consecutive_matches_ = 0;
};

#endif
