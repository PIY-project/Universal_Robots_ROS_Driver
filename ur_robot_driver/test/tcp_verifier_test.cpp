#include <ur_robot_driver/tcp_verifier.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

BridgeRtdeSample packet(double timestamp, const TcpVerifier::Vector6& tcp, std::uint32_t runtime = 2)
{
    BridgeRtdeSample result;
    result.controller_timestamp = timestamp;
    result.tcp_offset = tcp;
    result.runtime_state = runtime;
    return result;
}
}

int main()
{
    try
    {
        const TcpVerifier::Vector6 raw{0.0, -0.0000006, 0.1250004, 0.0, 0.0, 0.0};
        const auto expected = TcpVerifier::quantizeForUrScript(raw);
        require(expected[0] == 0.0, "Zero must remain exactly zero after URScript quantization");
        require(expected[1] == -0.000001 && expected[2] == 0.125,
                "Expected 1e-6 TCP quantization");
        auto invalid_expected = raw;
        invalid_expected[0] = std::numeric_limits<double>::infinity();
        bool rejected_invalid = false;
        try { TcpVerifier::quantizeForUrScript(invalid_expected); }
        catch (const std::invalid_argument&) { rejected_invalid = true; }
        require(rejected_invalid, "Nonfinite configuration must fail before URCL serialization");
        invalid_expected[0] = 10000.0;
        rejected_invalid = false;
        try { TcpVerifier::quantizeForUrScript(invalid_expected); }
        catch (const std::invalid_argument&) { rejected_invalid = true; }
        require(rejected_invalid, "Out-of-range configuration must fail before URCL serialization");
        require(TcpVerifier::equivalent(expected, expected), "Exact TCP must match");
        auto offset = expected;
        offset[0] += 0.002;
        require(!TcpVerifier::equivalent(expected, offset), "2 mm translation error must be rejected");
        offset = expected;
        offset[5] = 0.002;
        require(!TcpVerifier::equivalent(expected, offset), "2 mrad rotation error must be rejected");
        offset[5] = std::numeric_limits<double>::quiet_NaN();
        require(!TcpVerifier::equivalent(expected, offset), "Nonfinite TCP must be rejected");

        TcpVerifier verifier(raw, std::chrono::seconds(1));
        auto first = packet(1.0, expected);
        require(verifier.observe(first) == TcpVerifier::Event::PROGRAM_STARTED, "Missing program start");
        require(!verifier.motionReady(), "Program start alone cannot authorize motion");
        const auto epoch = verifier.claimPendingEpoch();
        require(epoch != 0 && epoch == first.program_epoch, "Expected a pending program epoch");
        require(verifier.claimPendingEpoch() == 0, "Only one configuration claim per epoch");
        require(verifier.beginVerification(epoch, first.sequence), "Could not begin readback verification");

        auto second = packet(1.02, expected);
        require(verifier.observe(second) == TcpVerifier::Event::NONE, "Unexpected RTDE event");
        require(verifier.waitForVerification(epoch, std::chrono::milliseconds(0)) ==
                TcpVerifier::WaitResult::TIMEOUT, "One matching packet is insufficient");
        auto duplicate = packet(1.02, expected);
        require(verifier.observe(duplicate) == TcpVerifier::Event::DUPLICATE_SAMPLE,
                "Repeated controller timestamp must be treated as duplicate");
        require(duplicate.sequence == second.sequence, "Duplicate packet advanced sequence");
        require(verifier.waitForVerification(epoch, std::chrono::milliseconds(0)) ==
                TcpVerifier::WaitResult::TIMEOUT, "Duplicate packet must not count as second match");
        auto third = packet(1.04, expected);
        require(verifier.observe(third) == TcpVerifier::Event::NONE, "Unexpected RTDE event");
        require(verifier.waitForVerification(epoch, std::chrono::milliseconds(0)) ==
                TcpVerifier::WaitResult::VERIFIED, "Two fresh matching packets must verify TCP");
        require(verifier.motionReady(), "Verified fresh TCP should authorize motion");

        auto changed_tcp = expected;
        changed_tcp[1] += 0.002;
        auto changed = packet(1.06, changed_tcp);
        require(verifier.observe(changed) == TcpVerifier::Event::TCP_CHANGED,
                "Unexpected TCP change was not detected");
        require(verifier.isFault() && !verifier.motionReady(), "TCP change must fail closed");
        require(verifier.claimPendingEpoch() == 0, "Fault cannot silently reapply in same epoch");

        verifier.onProgramDisconnected();
        auto restarted = packet(1.08, expected);
        require(verifier.observe(restarted) == TcpVerifier::Event::PROGRAM_STARTED,
                "Program restart was not detected");
        require(verifier.claimPendingEpoch() == restarted.program_epoch,
                "Restart must create a fresh configuration opportunity");
        const auto before_reset = restarted.program_epoch;
        auto clock_reset = packet(0.10, expected);
        require(verifier.observe(clock_reset) == TcpVerifier::Event::TIMESTAMP_RESET,
                "Controller timestamp rollback was not detected");
        require(clock_reset.program_epoch == before_reset + 1,
                "Timestamp reset must advance epoch exactly once");
        require(!verifier.motionReady(), "Timestamp reset must invalidate verification");

        auto bad_time = packet(std::numeric_limits<double>::quiet_NaN(), expected);
        require(verifier.observe(bad_time) == TcpVerifier::Event::INVALID_SAMPLE,
                "Invalid controller timestamp was accepted");
        require(verifier.isFault() && !verifier.motionReady(),
                "Invalid controller timestamp must fail closed");

        TcpVerifier stale(expected, std::chrono::milliseconds(2));
        auto fresh = packet(2.0, expected);
        stale.observe(fresh);
        const auto stale_epoch = stale.claimPendingEpoch();
        stale.beginVerification(stale_epoch, fresh.sequence);
        auto match1 = packet(2.02, expected);
        stale.observe(match1);
        auto match2 = packet(2.04, expected);
        stale.observe(match2);
        require(stale.motionReady(), "Fresh stream should be ready after two matches");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        require(!stale.motionReady(), "Age gate must block motion before watchdog runs");
        require(stale.onNoPackage(), "Watchdog must invalidate a stale RTDE stream");
        require(!stale.programRunning(), "Stale RTDE stream must invalidate running state");

        std::cout << "tcp_verifier_test passed\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "tcp_verifier_test failed: " << e.what() << '\n';
        return 1;
    }
}
