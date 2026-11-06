#ifndef UR_ROBOT_DRIVER_BRIDGE_TRACE_RECORDER_HPP
#define UR_ROBOT_DRIVER_BRIDGE_TRACE_RECORDER_HPP

#include <ur_robot_driver/tcp_verifier.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

// Disk I/O is confined to the writer thread. The RTDE reader only copies one
// complete DataPackage into a bounded queue and a bounded pre-roll buffer.
class BridgeTraceRecorder
{
public:
    BridgeTraceRecorder(std::string root, std::string robot_name,
                        std::size_t queue_capacity = 8192,
                        std::size_t preroll_samples = 500,
                        std::size_t max_samples_per_attempt = 45000,
                        std::chrono::seconds max_capture_duration = std::chrono::minutes(20));
    ~BridgeTraceRecorder();
    BridgeTraceRecorder(const BridgeTraceRecorder&) = delete;
    BridgeTraceRecorder& operator=(const BridgeTraceRecorder&) = delete;

    void sample(const BridgeRtdeSample& value);
    void beginGoal(const std::string& attempt_id, std::uint32_t segment_id,
                   const std::string& goal_json);
    void endGoal(const std::string& attempt_id, std::uint32_t segment_id,
                 const std::string& result_json);
    void event(const std::string& name, const std::string& details_json = "{}");
    void stop();
    bool healthy() const { return disk_ok_.load(); }
    std::uint64_t droppedSamples() const { return dropped_samples_.load(); }
    const std::string& sessionId() const { return session_id_; }

    static std::string jsonEscape(const std::string& value);

private:
    enum class Kind { SAMPLE, EVENT, CHECKPOINT, STATUS };
    struct Record
    {
        Kind kind = Kind::EVENT;
        std::string attempt_id;
        BridgeRtdeSample sample;
        std::string line;
        std::chrono::steady_clock::time_point enqueued_at = std::chrono::steady_clock::now();
    };

    static bool validAttemptId(const std::string& value);
    static std::string safeName(const std::string& value);
    std::string eventLine(const std::string& name, const std::string& details_json,
                          std::chrono::steady_clock::time_point at = std::chrono::steady_clock::now()) const;
    void enqueueLocked(Record record);
    void worker();

    const std::string root_;
    const std::string robot_name_;
    const std::string session_id_;
    const std::size_t queue_capacity_;
    const std::size_t preroll_samples_;
    const std::size_t max_samples_per_attempt_;
    const std::chrono::seconds max_capture_duration_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Record> queue_;
    std::deque<BridgeRtdeSample> preroll_;
    std::deque<std::string> event_history_;
    std::string active_attempt_;
    std::chrono::steady_clock::time_point active_until_{};
    std::size_t active_samples_ = 0;
    std::uint64_t active_last_sequence_ = 0;
    bool active_capture_expired_ = false;
    bool active_cap_reported_ = false;
    std::unordered_map<std::string, std::uint64_t> dropped_by_attempt_;
    bool stopping_ = false;
    std::thread writer_thread_;
    std::atomic<bool> disk_ok_{true};
    std::atomic<std::uint64_t> dropped_samples_{0};
};

#endif
