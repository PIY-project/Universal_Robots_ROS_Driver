#include <ur_robot_driver/tcp_verifier.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace
{
using Vector6 = TcpVerifier::Vector6;

struct Rotation
{
    double m[3][3]{};
};

Rotation fromRotationVector(const Vector6& v)
{
    const double x = v[3], y = v[4], z = v[5];
    const double theta = std::sqrt(x*x + y*y + z*z);
    Rotation r;
    for (unsigned i = 0; i < 3; ++i)
        r.m[i][i] = 1.0;
    if (theta < 1e-12)
        return r;

    const double a[3] = {x/theta, y/theta, z/theta};
    const double s = std::sin(theta), c = std::cos(theta);
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            r.m[i][j] = (i == j ? c : 0.0) + (1.0-c)*a[i]*a[j];
    r.m[0][1] -= s*a[2]; r.m[1][0] += s*a[2];
    r.m[0][2] += s*a[1]; r.m[2][0] -= s*a[1];
    r.m[1][2] -= s*a[0]; r.m[2][1] += s*a[0];
    return r;
}

double angularDistance(const Rotation& a, const Rotation& b)
{
    double trace = 0.0;
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            trace += a.m[i][j] * b.m[i][j];
    const double c = std::max(-1.0, std::min(1.0, (trace-1.0)/2.0));
    return std::acos(c);
}
}

TcpVerifier::TcpVerifier(const Vector6& expected, std::chrono::milliseconds max_sample_age)
    : expected_(quantizeForUrScript(expected)), max_sample_age_(max_sample_age)
{
    if (max_sample_age_.count() <= 0)
        throw std::invalid_argument("RTDE maximum sample age must be positive");
}

TcpVerifier::Vector6 TcpVerifier::quantizeForUrScript(const Vector6& tcp)
{
    Vector6 result{};
    for (unsigned i = 0; i < result.size(); ++i)
    {
        const double scaled = tcp[i] * 1000000.0;
        if (!std::isfinite(scaled) ||
            std::round(scaled) < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
            std::round(scaled) > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
            throw std::invalid_argument("TCP offset cannot be serialized as URCL SET_TCP_OFFSET");
        result[i] = std::round(scaled) / 1000000.0;
    }
    return result;
}

bool TcpVerifier::equivalent(const Vector6& expected, const Vector6& actual,
                             double translation_tolerance_m, double rotation_tolerance_rad)
{
    for (double value : expected)
        if (!std::isfinite(value)) return false;
    for (double value : actual)
        if (!std::isfinite(value)) return false;
    double d2 = 0.0;
    for (unsigned i = 0; i < 3; ++i)
        d2 += (expected[i]-actual[i]) * (expected[i]-actual[i]);
    return std::sqrt(d2) <= translation_tolerance_m &&
           angularDistance(fromRotationVector(expected), fromRotationVector(actual)) <= rotation_tolerance_rad;
}

void TcpVerifier::invalidateLocked()
{
    verified_ = false;
    verifying_ = false;
    consecutive_matches_ = 0;
}

bool TcpVerifier::freshLocked(std::chrono::steady_clock::time_point now) const
{
    return have_sample_ && now-latest_.received_at <= max_sample_age_;
}

TcpVerifier::Event TcpVerifier::observe(BridgeRtdeSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(sample.controller_timestamp))
    {
        invalidateLocked();
        fault_ = true;
        fault_reason_ = "Invalid RTDE controller timestamp";
        cv_.notify_all();
        return Event::INVALID_SAMPLE;
    }
    if (have_sample_ && sample.controller_timestamp == latest_.controller_timestamp)
    {
        sample.sequence = latest_.sequence;
        sample.program_epoch = latest_.program_epoch;
        sample.received_at = latest_.received_at;
        return Event::DUPLICATE_SAMPLE;
    }
    const bool playing = sample.runtime_state == 2;
    const bool timestamp_reset = have_sample_ && sample.controller_timestamp < latest_.controller_timestamp;
    Event event = Event::NONE;
    if (timestamp_reset)
    {
        ++epoch_;
        program_running_ = playing;
        invalidateLocked();
        fault_ = false;
        fault_reason_.clear();
        event = Event::TIMESTAMP_RESET;
    }
    else if (playing && !program_running_)
    {
        ++epoch_;
        program_running_ = true;
        invalidateLocked();
        fault_ = false;
        fault_reason_.clear();
        event = Event::PROGRAM_STARTED;
    }
    else if (!playing && program_running_)
    {
        ++epoch_;
        program_running_ = false;
        invalidateLocked();
        event = Event::PROGRAM_STOPPED;
    }

    sample.sequence = ++sequence_;
    sample.program_epoch = epoch_;
    sample.received_at = std::chrono::steady_clock::now();
    latest_ = sample;
    have_sample_ = true;

    if (program_running_ && verified_ && !equivalent(expected_, sample.tcp_offset))
    {
        invalidateLocked();
        fault_ = true;
        fault_reason_ = "TCP changed after verification";
        event = Event::TCP_CHANGED;
    }
    if (program_running_ && verifying_ && sample.sequence > verify_after_sequence_)
    {
        if (equivalent(expected_, sample.tcp_offset))
        {
            if (++consecutive_matches_ >= 2)
            {
                verifying_ = false;
                verified_ = true;
            }
        }
        else
        {
            consecutive_matches_ = 0;
        }
    }
    cv_.notify_all();
    return event;
}

bool TcpVerifier::onNoPackage()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!program_running_ || freshLocked(std::chrono::steady_clock::now()))
        return false;
    ++epoch_;
    program_running_ = false;
    invalidateLocked();
    cv_.notify_all();
    return true;
}

void TcpVerifier::onProgramDisconnected()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (program_running_)
        ++epoch_;
    program_running_ = false;
    invalidateLocked();
    cv_.notify_all();
}

std::uint64_t TcpVerifier::claimPendingEpoch()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (program_running_ && !verified_ && !fault_ && epoch_ != claimed_epoch_ &&
        freshLocked(std::chrono::steady_clock::now()))
    {
        claimed_epoch_ = epoch_;
        return epoch_;
    }
    return 0;
}

bool TcpVerifier::beginVerification(std::uint64_t epoch, std::uint64_t after_sequence)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch != epoch_ || !program_running_ || fault_)
        return false;
    verify_after_sequence_ = after_sequence;
    consecutive_matches_ = 0;
    verifying_ = true;
    return true;
}

TcpVerifier::WaitResult TcpVerifier::waitForVerification(std::uint64_t epoch, std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, timeout, [&] { return verified_ || fault_ || epoch != epoch_ || !program_running_; });
    if (epoch != epoch_ || !program_running_)
        return WaitResult::INTERRUPTED;
    if (verified_)
        return WaitResult::VERIFIED;
    return WaitResult::TIMEOUT;
}

void TcpVerifier::fail(std::uint64_t epoch, const std::string& reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch != epoch_)
        return;
    invalidateLocked();
    fault_ = true;
    fault_reason_ = reason;
    cv_.notify_all();
}

void TcpVerifier::waitForChange(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, timeout);
}

bool TcpVerifier::motionReady() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return program_running_ && verified_ && !fault_ && freshLocked(std::chrono::steady_clock::now());
}

bool TcpVerifier::isFault() const { std::lock_guard<std::mutex> lock(mutex_); return fault_; }
bool TcpVerifier::isVerified() const { std::lock_guard<std::mutex> lock(mutex_); return verified_; }
bool TcpVerifier::programRunning() const { std::lock_guard<std::mutex> lock(mutex_); return program_running_; }
bool TcpVerifier::snapshot(BridgeRtdeSample& sample) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_sample_) return false;
    sample = latest_;
    return true;
}
std::uint64_t TcpVerifier::sequence() const { std::lock_guard<std::mutex> lock(mutex_); return sequence_; }
std::uint64_t TcpVerifier::epoch() const { std::lock_guard<std::mutex> lock(mutex_); return epoch_; }
std::string TcpVerifier::faultReason() const { std::lock_guard<std::mutex> lock(mutex_); return fault_reason_; }
