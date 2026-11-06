#include <ur_robot_driver/bridge_trace_recorder.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    require(input.good(), "Cannot open " + path.string());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::size_t lineCount(const std::string& content)
{
    std::size_t count = 0;
    for (char c : content) if (c == '\n') ++count;
    return count;
}
}

int main()
{
    try
    {
        const auto root = std::filesystem::temp_directory_path() /
            ("ur_bridge_trace_test_" + std::to_string(getpid()) + "_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(root);

        BridgeTraceRecorder recorder(root.string(), "robot/ur20", 64, 8, 16);
        BridgeRtdeSample before;
        before.controller_timestamp = 12.0;
        before.sequence = 10;
        before.program_epoch = 2;
        before.runtime_state = 2;
        before.received_at = std::chrono::steady_clock::now();
        before.actual_q[0] = 1.25;
        before.tcp_offset[2] = 0.125;

        recorder.event("tcp_verified", "{\"expected_z_m\":0.125}");
        recorder.sample(before);  // Should survive as pre-roll when the first goal arrives.
        recorder.beginGoal("attempt_1", 7, "{\"type\":\"MoveL\"}");

        BridgeRtdeSample during = before;
        during.controller_timestamp = 12.02;
        during.sequence = 11;
        during.received_at = std::chrono::steady_clock::now();
        during.actual_q[0] = 1.30;
        recorder.sample(during);
        recorder.endGoal("attempt_1", 7, "{\"success\":true}");

        BridgeRtdeSample between_segments = during;
        between_segments.controller_timestamp = 12.04;
        between_segments.sequence = 12;
        between_segments.received_at = std::chrono::steady_clock::now();
        recorder.sample(between_segments);  // Keep the tool-switch interval.
        recorder.beginGoal("attempt_1", 8, "{\"type\":\"MoveJ\"}");

        BridgeRtdeSample second_goal = between_segments;
        second_goal.controller_timestamp = 12.06;
        second_goal.sequence = 13;
        second_goal.received_at = std::chrono::steady_clock::now();
        recorder.sample(second_goal);
        recorder.endGoal("attempt_1", 8, "{\"success\":true}");

        recorder.beginGoal("", 0, "{\"type\":\"MoveJ\"}");  // Ordinary motion ends attribution.
        BridgeRtdeSample unrelated = second_goal;
        unrelated.controller_timestamp = 12.08;
        unrelated.sequence = 14;
        unrelated.received_at = std::chrono::steady_clock::now();
        recorder.sample(unrelated);
        recorder.stop();

        const auto attempt_dir = root / "attempt_1" / "bridge_robot_ur20";
        const std::string csv = readFile(attempt_dir / "rtde.csv");
        const std::string events = readFile(attempt_dir / "bridge_events.jsonl");
        require(lineCount(csv) == 5,
                "Expected header, pre-roll, first goal, segment gap and second goal; ordinary motion excluded");
        require(csv.find("target_TCP_pose_0") != std::string::npos, "Missing RTDE column");
        std::istringstream csv_lines(csv);
        std::string line;
        std::getline(csv_lines, line);  // Header.
        std::getline(csv_lines, line);  // Pre-roll sample.
        std::getline(csv_lines, line);  // In-goal sample.
        std::istringstream fields(line);
        std::string field;
        std::getline(fields, field, ',');  // Session ID.
        std::getline(fields, field, ',');  // Host monotonic time.
        std::getline(fields, field, ',');  // Controller timestamp.
        require(std::stod(field) == 12.02, "Wrong in-goal controller timestamp");
        std::getline(fields, field, ',');  // Sequence.
        require(field == "11", "Wrong in-goal sequence");
        std::getline(fields, field, ',');  // Program epoch.
        require(field == "2", "Wrong in-goal epoch");
        require(events.find("tcp_verified") != std::string::npos,
                "Pre-goal TCP verification event was not attached to the attempt");
        require(events.find("goal_received") != std::string::npos, "Missing goal event");
        require(events.find("goal_finished") != std::string::npos, "Missing result event");
        require(events.find("\"segment_id\":8") != std::string::npos, "Missing second correlated segment");
        require(events.find("trace_status") != std::string::npos, "Missing integrity status");

        BridgeTraceRecorder capped(root.string(), "cap", 64, 0, 1);
        capped.beginGoal("attempt_capped", 0, "{\"type\":\"MoveL\"}");
        capped.sample(before);
        capped.sample(during);  // Exceeds the one-sample limit.
        capped.beginGoal("attempt_clean", 0, "{\"type\":\"MoveL\"}");
        capped.sample(second_goal);
        capped.stop();

        const std::string capped_events = readFile(root / "attempt_capped" / "bridge_cap" / "bridge_events.jsonl");
        const std::string clean_events = readFile(root / "attempt_clean" / "bridge_cap" / "bridge_events.jsonl");
        require(capped_events.find("sample_cap") != std::string::npos,
                "Sample cap must be recorded as incomplete");
        require(capped_events.find("\"trace_complete\":false") != std::string::npos,
                "Capped attempt must not claim a complete trace");
        require(clean_events.find("\"trace_complete\":true") != std::string::npos,
                "Drops in an earlier attempt must not taint a later attempt");
        std::cout << "bridge_trace_recorder_test passed; artifacts: " << root << '\n';
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "bridge_trace_recorder_test failed: " << e.what() << '\n';
        return 1;
    }
}
