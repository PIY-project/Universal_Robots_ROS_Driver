#include <ur_robot_driver/bridge_trace_recorder.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unistd.h>

namespace
{
std::int64_t monotonicNanoseconds(std::chrono::steady_clock::time_point tp)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

void writeVector(std::ostream& out, const BridgeRtdeSample::Vector6& value)
{
    for (double x : value)
        out << ',' << x;
}

const char* csvHeader()
{
    return "bridge_session_id,host_monotonic_ns,controller_timestamp,sequence,program_epoch,runtime_state,robot_mode,safety_mode,"
           "actual_q_0,actual_q_1,actual_q_2,actual_q_3,actual_q_4,actual_q_5,"
           "actual_qd_0,actual_qd_1,actual_qd_2,actual_qd_3,actual_qd_4,actual_qd_5,"
           "target_q_0,target_q_1,target_q_2,target_q_3,target_q_4,target_q_5,"
           "target_TCP_pose_0,target_TCP_pose_1,target_TCP_pose_2,target_TCP_pose_3,target_TCP_pose_4,target_TCP_pose_5,"
           "actual_TCP_pose_0,actual_TCP_pose_1,actual_TCP_pose_2,actual_TCP_pose_3,actual_TCP_pose_4,actual_TCP_pose_5,"
           "tcp_offset_0,tcp_offset_1,tcp_offset_2,tcp_offset_3,tcp_offset_4,tcp_offset_5\n";
}

std::string makeSessionId()
{
    std::ostringstream out;
    out << std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() << '_' << getpid();
    return out.str();
}
}

BridgeTraceRecorder::BridgeTraceRecorder(std::string root, std::string robot_name,
                                         std::size_t queue_capacity, std::size_t preroll_samples,
                                         std::size_t max_samples_per_attempt,
                                         std::chrono::seconds max_capture_duration)
    : root_(std::move(root)), robot_name_(safeName(robot_name)), session_id_(makeSessionId()),
      queue_capacity_(queue_capacity), preroll_samples_(preroll_samples),
      max_samples_per_attempt_(max_samples_per_attempt), max_capture_duration_(max_capture_duration)
{
    if (root_.empty())
    {
        disk_ok_.store(false);
        std::cerr << "[BridgeTraceRecorder] Empty trace root; recording disabled\n";
    }
    else
    {
        writer_thread_ = std::thread(&BridgeTraceRecorder::worker, this);
    }
}

BridgeTraceRecorder::~BridgeTraceRecorder() { stop(); }

std::string BridgeTraceRecorder::safeName(const std::string& value)
{
    std::string result;
    for (char c : value)
        result += ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                   (c >= '0' && c <= '9') || c == '_' || c == '-') ? c : '_';
    return result.empty() ? "robot" : result;
}

bool BridgeTraceRecorder::validAttemptId(const std::string& value)
{
    if (value.empty() || value.size() > 128)
        return false;
    for (char c : value)
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-'))
            return false;
    return true;
}

std::string BridgeTraceRecorder::jsonEscape(const std::string& value)
{
    std::ostringstream out;
    for (unsigned char c : value)
    {
        switch (c)
        {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 0x20)
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
            else
                out << static_cast<char>(c);
        }
    }
    return out.str();
}

std::string BridgeTraceRecorder::eventLine(const std::string& name, const std::string& details_json,
                                           std::chrono::steady_clock::time_point at) const
{
    std::ostringstream out;
    out << "{\"bridge_session_id\":\"" << jsonEscape(session_id_) << "\",\"robot\":\""
        << jsonEscape(robot_name_) << "\",\"host_monotonic_ns\":"
        << monotonicNanoseconds(at)
        << ",\"event\":\"" << jsonEscape(name) << "\",\"details\":" << details_json << "}\n";
    return out.str();
}

void BridgeTraceRecorder::enqueueLocked(Record record)
{
    if (stopping_ || root_.empty())
    {
        if (record.kind == Kind::SAMPLE)
        {
            ++dropped_samples_;
            ++dropped_by_attempt_[record.attempt_id];
        }
        return;
    }
    if (queue_capacity_ == 0)
    {
        if (record.kind == Kind::SAMPLE)
        {
            ++dropped_samples_;
            ++dropped_by_attempt_[record.attempt_id];
        }
        return;
    }
    if (queue_.size() >= queue_capacity_)
    {
        // Preserve control events and make every loss observable.
        auto it = queue_.begin();
        while (it != queue_.end() && it->kind != Kind::SAMPLE) ++it;
        if (it != queue_.end())
        {
            ++dropped_by_attempt_[it->attempt_id];
            queue_.erase(it);
            ++dropped_samples_;
        }
        else if (record.kind == Kind::SAMPLE)
        {
            ++dropped_samples_;
            ++dropped_by_attempt_[record.attempt_id];
            return;
        }
        else
        {
            queue_.pop_front();
        }
    }
    queue_.push_back(std::move(record));
    cv_.notify_one();
}

void BridgeTraceRecorder::sample(const BridgeRtdeSample& value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || root_.empty()) return;
    if (preroll_samples_ != 0)
    {
        preroll_.push_back(value);
        while (preroll_.size() > preroll_samples_) preroll_.pop_front();
    }
    if (active_attempt_.empty())
        return;
    if (std::chrono::steady_clock::now() > active_until_)
    {
        if (!active_capture_expired_)
        {
            active_capture_expired_ = true;
            ++dropped_samples_;
            ++dropped_by_attempt_[active_attempt_];
            enqueueLocked({Kind::EVENT, active_attempt_, {},
                eventLine("trace_incomplete", "{\"reason\":\"capture_timeout\"}")});
            enqueueLocked({Kind::STATUS, active_attempt_, {}, {}});
        }
        return;
    }
    if (active_samples_ >= max_samples_per_attempt_)
    {
        ++dropped_samples_;
        ++dropped_by_attempt_[active_attempt_];
        if (!active_cap_reported_)
        {
            active_cap_reported_ = true;
            enqueueLocked({Kind::EVENT, active_attempt_, {},
                eventLine("trace_incomplete", "{\"reason\":\"sample_cap\"}")});
        }
        return;
    }
    Record record;
    record.kind = Kind::SAMPLE;
    record.attempt_id = active_attempt_;
    record.sample = value;
    ++active_samples_;
    active_last_sequence_ = value.sequence;
    enqueueLocked(std::move(record));
}

void BridgeTraceRecorder::beginGoal(const std::string& attempt_id, std::uint32_t segment_id,
                                    const std::string& goal_json)
{
    if (!validAttemptId(attempt_id))
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_attempt_.empty())
            enqueueLocked({Kind::STATUS, active_attempt_, {}, {}});
        active_attempt_.clear();
        active_capture_expired_ = false;
        if (!attempt_id.empty())
            std::cerr << "[BridgeTraceRecorder] Invalid attempt_id; trace cannot be correlated\n";
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || root_.empty()) return;
    const bool new_attempt = active_attempt_ != attempt_id;
    const bool gap_since_previous_goal = std::chrono::steady_clock::now() > active_until_;
    if (new_attempt || gap_since_previous_goal)
    {
        if (new_attempt && !active_attempt_.empty())
            enqueueLocked({Kind::STATUS, active_attempt_, {}, {}});
        active_attempt_ = attempt_id;
        if (new_attempt)
        {
            active_samples_ = 0;
            active_last_sequence_ = 0;
            active_cap_reported_ = false;
        }
        active_capture_expired_ = false;
        for (const auto& historic : event_history_)
            enqueueLocked({Kind::EVENT, attempt_id, {}, historic});
        for (const auto& sample_value : preroll_)
        {
            if (sample_value.sequence > active_last_sequence_)
            {
                enqueueLocked({Kind::SAMPLE, attempt_id, sample_value, {}});
                active_last_sequence_ = sample_value.sequence;
                ++active_samples_;
            }
        }
    }
    active_until_ = std::chrono::steady_clock::now() + max_capture_duration_;
    std::ostringstream details;
    details << "{\"attempt_id\":\"" << jsonEscape(attempt_id) << "\",\"segment_id\":"
            << segment_id << ",\"goal\":" << goal_json << '}';
    enqueueLocked({Kind::EVENT, attempt_id, {}, eventLine("goal_received", details.str())});
}

void BridgeTraceRecorder::endGoal(const std::string& attempt_id, std::uint32_t segment_id,
                                  const std::string& result_json)
{
    if (!validAttemptId(attempt_id)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || root_.empty()) return;
    std::ostringstream details;
    details << "{\"attempt_id\":\"" << jsonEscape(attempt_id) << "\",\"segment_id\":"
            << segment_id << ",\"result\":" << result_json << '}';
    enqueueLocked({Kind::EVENT, attempt_id, {}, eventLine("goal_finished", details.str())});
    // More segments and a bridge restart can follow this goal. Capture stays
    // active until the attempt changes, expires, reaches its cap, or shutdown.
    enqueueLocked({Kind::CHECKPOINT, attempt_id, {}, {}});
}

void BridgeTraceRecorder::event(const std::string& name, const std::string& details_json)
{
    const std::string line = eventLine(name, details_json);
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || root_.empty()) return;
    const bool replay_on_next_goal = name == "bridge_started" || name == "calibration_checked" ||
        name == "controller_timestamp_reset" || name.rfind("tcp_", 0) == 0 ||
        name.rfind("rtde_", 0) == 0 || name.rfind("program_", 0) == 0 ||
        name.rfind("script_", 0) == 0;
    if (replay_on_next_goal)
    {
        event_history_.push_back(line);
        while (event_history_.size() > 1000) event_history_.pop_front();
    }
    enqueueLocked({Kind::EVENT, "_bridge_sessions", {}, line});
    if (!active_attempt_.empty() && std::chrono::steady_clock::now() <= active_until_)
        enqueueLocked({Kind::EVENT, active_attempt_, {}, line});
}

void BridgeTraceRecorder::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) return;
        if (!active_attempt_.empty())
            enqueueLocked({Kind::STATUS, active_attempt_, {}, {}});
        stopping_ = true;
        cv_.notify_all();
    }
    if (writer_thread_.joinable()) writer_thread_.join();
}

void BridgeTraceRecorder::worker()
{
    std::string current_attempt;
    std::ofstream csv, events;
    auto openFor = [&](const std::string& id) -> bool {
        if (id == current_attempt && events.is_open()) return true;
        if (csv.is_open()) csv.close();
        if (events.is_open()) events.close();
        current_attempt.clear();
        try
        {
            const std::filesystem::path dir = id == "_bridge_sessions"
                ? std::filesystem::path(root_) / "_bridge_sessions"
                : std::filesystem::path(root_) / id / ("bridge_" + robot_name_);
            std::filesystem::create_directories(dir);
            const auto events_path = dir / (id == "_bridge_sessions" ? session_id_ + ".jsonl" : "bridge_events.jsonl");
            events.open(events_path, std::ios::app);
            if (!events) throw std::runtime_error("Cannot open " + events_path.string());
            if (id != "_bridge_sessions")
            {
                const auto csv_path = dir / "rtde.csv";
                const bool write_header = !std::filesystem::exists(csv_path) || std::filesystem::file_size(csv_path) == 0;
                csv.open(csv_path, std::ios::app);
                if (!csv) throw std::runtime_error("Cannot open " + csv_path.string());
                csv << std::setprecision(17);
                if (write_header) csv << csvHeader();
            }
            current_attempt = id;
            return true;
        }
        catch (const std::exception& e)
        {
            disk_ok_.store(false);
            std::cerr << "[BridgeTraceRecorder] " << e.what() << '\n';
            return false;
        }
    };

    for (;;)
    {
        Record record;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
            if (queue_.empty() && stopping_) break;
            record = std::move(queue_.front());
            queue_.pop_front();
        }
        if (!openFor(record.attempt_id)) continue;
        if (record.kind == Kind::SAMPLE)
        {
            const auto& s = record.sample;
            csv << session_id_ << ',' << monotonicNanoseconds(s.received_at) << ','
                << s.controller_timestamp << ',' << s.sequence << ',' << s.program_epoch << ','
                << s.runtime_state << ',' << s.robot_mode << ',' << s.safety_mode;
            writeVector(csv, s.actual_q);
            writeVector(csv, s.actual_qd);
            writeVector(csv, s.target_q);
            writeVector(csv, s.target_tcp_pose);
            writeVector(csv, s.actual_tcp_pose);
            writeVector(csv, s.tcp_offset);
            csv << '\n';
        }
        else if (record.kind == Kind::EVENT)
        {
            events << record.line;
        }
        else
        {
            std::uint64_t attempt_drops = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                attempt_drops = dropped_by_attempt_[record.attempt_id];
            }
            events << eventLine(record.kind == Kind::STATUS ? "trace_status" : "trace_checkpoint",
                std::string("{\"trace_complete\":") +
                (disk_ok_.load() && attempt_drops == 0 ? "true" : "false") +
                ",\"dropped_samples\":" + std::to_string(attempt_drops) +
                ",\"final\":" + (record.kind == Kind::STATUS ? "true" : "false") + "}",
                record.enqueued_at);
            events.flush();
            if (csv.is_open()) csv.flush();
        }
        if (!events.good() || (record.kind == Kind::SAMPLE && !csv.good()))
        {
            disk_ok_.store(false);
            std::cerr << "[BridgeTraceRecorder] Write failed for " << record.attempt_id << '\n';
            events.clear();
            csv.clear();
        }
    }
    if (events.is_open()) events.flush();
    if (csv.is_open()) csv.flush();
}
