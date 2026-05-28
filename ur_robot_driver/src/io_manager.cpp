#include <ur_robot_driver/io_manager.hpp>

#include <fstream>
#include <nlohmann/json.hpp>

// ---------------------------------------------------------------------------
//  File-local helpers
// ---------------------------------------------------------------------------

namespace
{

    // Reads a JSON array of strings of exactly `expected` size into name_arr starting at bit_offset.
    // Non-empty entries override the current value; empty entries keep the existing default.
    template <size_t N>
    bool applyNameGroup(const nlohmann::json &parent, const std::string &key,
                        size_t expected, size_t bit_offset,
                        std::array<std::string, N> &name_arr)
    {
        if (!parent.contains(key))
            return true; // group absent — keep defaults

        const auto &arr = parent[key];
        if (!arr.is_array() || arr.size() != expected)
        {
            ROS_WARN_STREAM("[IOManager]: '" << key << "' must have exactly "
                                             << expected << " entries — skipping group");
            return false;
        }

        for (size_t i = 0; i < expected; ++i)
        {
            const std::string name = arr[i].get<std::string>();
            if (!name.empty())
                name_arr[bit_offset + i] = name;
        }
        return true;
    }

} // namespace

// ---------------------------------------------------------------------------
//  Constructor / destructor
// ---------------------------------------------------------------------------

IOManager::IOManager(ros::NodeHandle &nh,
                     std::shared_ptr<urcl::UrDriver> driver,
                     const std::string &json_path)
    : nh_(nh), driver_(std::move(driver))
{
    if (json_path.empty() || !loadFromJson(json_path))
        loadDefaults();
}

IOManager::~IOManager()
{
    stop();
}

// ---------------------------------------------------------------------------
//  start / stop
// ---------------------------------------------------------------------------

void IOManager::start(double rate_hz)
{
    if (running_.load())
        return;

    rate_hz_ = rate_hz;
    running_.store(true);

    pub_ = nh_.advertise<rpwc_msgs::robotIOSignals>("io_signals_state", 1, true);
    srv_set_digital_ = nh_.advertiseService("set_digital_io_signal", &IOManager::callbackSetDigital, this);
    srv_get_digital_ = nh_.advertiseService("get_digital_io_signal", &IOManager::callbackGetDigital, this);

    pub_thread_ = std::thread(&IOManager::publisherThread, this);
}

void IOManager::stop()
{
    if (!running_.load())
        return;

    running_.store(false);

    if (pub_thread_.joinable())
        pub_thread_.join();

    srv_set_digital_.shutdown();
    srv_get_digital_.shutdown();
    pub_.shutdown();
}

// ---------------------------------------------------------------------------
//  Service callbacks
// ---------------------------------------------------------------------------

bool IOManager::callbackSetDigital(rpwc_msgs::setDigitalIOSignal::Request &req,
                                   rpwc_msgs::setDigitalIOSignal::Response &res)
{
    const auto it = digital_lookup_.find(req.signal_name);
    if (it == digital_lookup_.end())
    {
        res.success = false;
        res.message = "Unknown signal name: " + req.signal_name;
        return true;
    }

    const DigitalSignal &sig = it->second;
    switch (sig.kind)
    {
        case DigitalKind::StdIn:
            res.success = false;
            res.message = "Signal '" + req.signal_name + "' is read-only (standard input)";
            return true;
        case DigitalKind::CfgIn:
            res.success = false;
            res.message = "Signal '" + req.signal_name + "' is read-only (configurable input)";
            return true;
        case DigitalKind::SafetyIn:
        case DigitalKind::SafetyOut:
            res.success = false;
            res.message = "Signal '" + req.signal_name + "' is read-only (safety IO)";
            return true;
        case DigitalKind::ToolIn:
            res.success = false;
            res.message = "Signal '" + req.signal_name + "' is read-only (tool input)";
            return true;
        case DigitalKind::StdOut:
            res.success = driver_->getRTDEWriter().sendStandardDigitalOutput(sig.pin, req.value);
            break;
        case DigitalKind::CfgOut:
            res.success = driver_->getRTDEWriter().sendConfigurableDigitalOutput(sig.pin, req.value);
            break;
        case DigitalKind::ToolOut:
            res.success = driver_->getRTDEWriter().sendToolDigitalOutput(sig.pin, req.value);
            break;
    }

    if (!res.success)
        res.message = "Failed to set signal '" + req.signal_name + "'";

    return true;
}

bool IOManager::callbackGetDigital(rpwc_msgs::getDigitalIOSignal::Request &req, rpwc_msgs::getDigitalIOSignal::Response &res)
{
    const auto it = digital_lookup_.find(req.signal_name);
    if (it == digital_lookup_.end())
    {
        res.success = false;
        res.message = "Unknown signal name: " + req.signal_name;
        res.value = false;
        return true;
    }

    const DigitalSignal &sig = it->second;
    std::uint64_t bits = 0;

    switch (sig.kind)
    {
        case DigitalKind::StdIn:
        case DigitalKind::CfgIn:
        case DigitalKind::SafetyIn:
        case DigitalKind::ToolIn:
            bits = digital_input_bits.load(std::memory_order_relaxed);
            break;
        case DigitalKind::StdOut:
        case DigitalKind::CfgOut:
        case DigitalKind::SafetyOut:
        case DigitalKind::ToolOut:
            bits = digital_output_bits.load(std::memory_order_relaxed);
            break;
    }

    res.value = static_cast<bool>((bits >> sig.bit) & 0x1);
    res.success = true;
    res.message.clear();
    return true;
}

// ---------------------------------------------------------------------------
//  Publisher thread
// ---------------------------------------------------------------------------

void IOManager::publisherThread()
{
    ROS_INFO("[IOManager]: Start");

    ros::Rate rate(rate_hz_);
    std::uint64_t old_in = ~0ULL; // force first publish
    std::uint64_t old_out = ~0ULL;

    rpwc_msgs::robotIOSignals msg;
    rpwc_msgs::digitalIOSignal entry;

    while (running_.load())
    {
        const std::uint64_t in_bits = digital_input_bits.load(std::memory_order_relaxed);
        const std::uint64_t out_bits = digital_output_bits.load(std::memory_order_relaxed);

        if (in_bits == old_in && out_bits == old_out)
        {
            rate.sleep();
            continue;
        }

        old_in = in_bits;
        old_out = out_bits;

        msg.inputSignals.clear();
        msg.outputSignals.clear();
        // analogSignals intentionally left empty — future extension point

        for (size_t i = 0; i < kTotal; ++i)
        {
            entry.signalName.data = digital_input_names_[i];
            entry.value.data = static_cast<bool>((in_bits >> i) & 0x1);
            msg.inputSignals.push_back(entry);

            entry.signalName.data = digital_output_names_[i];
            entry.value.data = static_cast<bool>((out_bits >> i) & 0x1);
            msg.outputSignals.push_back(entry);
        }

        pub_.publish(msg);
        rate.sleep();
    }

    ROS_INFO("[IOManager]: Shutdown");
}

// ---------------------------------------------------------------------------
//  Signal name loading
// ---------------------------------------------------------------------------

void IOManager::loadDefaults()
{
    for (size_t i = 0; i < kTotal; ++i)
    {
        digital_input_names_[i] = defaultInputName(i);
        digital_output_names_[i] = defaultOutputName(i);
    }
    buildLookup({}, {}); // all pins unassigned → default CfgIn and CfgOut
}

bool IOManager::loadFromJson(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        ROS_WARN_STREAM("[IOManager]: Cannot open '" << path << "', using default signal names");
        return false;
    }

    nlohmann::json root;
    try
    {
        file >> root;
    }
    catch (const nlohmann::json::parse_error &e)
    {
        ROS_WARN_STREAM("[IOManager]: JSON parse error in '" << path << "': " << e.what()
                                                             << " — using default signal names");
        return false;
    }

    // Populate name arrays with defaults; JSON overrides non-empty entries
    for (size_t i = 0; i < kTotal; ++i)
    {
        digital_input_names_[i] = defaultInputName(i);
        digital_output_names_[i] = defaultOutputName(i);
    }

    std::array<CfgPinRole, kCfgCount> cfg_in, cfg_out;
    cfg_in.fill(CfgPinRole::Normal);
    cfg_out.fill(CfgPinRole::Normal);

    if (root.contains("inputs"))
    {
        const auto &inp = root["inputs"];

        applyNameGroup(inp, "standard", kStdCount, 0, digital_input_names_);
        applyNameGroup(inp, "configurable", kCfgCount, kStdCount, digital_input_names_);
        applyNameGroup(inp, "tool", kToolCount, kStdCount + kCfgCount, digital_input_names_);

        if (inp.contains("safety"))
        {
            const auto &arr = inp["safety"];
            if (arr.is_array() && arr.size() == kCfgCount)
            {
                for (size_t i = 0; i < kCfgCount; ++i)
                    if (arr[i].get<bool>())
                        cfg_in[i] = CfgPinRole::Safety;
            }
            else
                ROS_WARN("[IOManager]: 'inputs.safety' must have exactly 8 bool entries — skipping");
        }
    }

    if (root.contains("outputs"))
    {
        const auto &out = root["outputs"];

        applyNameGroup(out, "standard", kStdCount, 0, digital_output_names_);
        applyNameGroup(out, "configurable", kCfgCount, kStdCount, digital_output_names_);
        applyNameGroup(out, "tool", kToolCount, kStdCount + kCfgCount, digital_output_names_);

        if (out.contains("safety"))
        {
            const auto &arr = out["safety"];
            if (arr.is_array() && arr.size() == kCfgCount)
            {
                for (size_t i = 0; i < kCfgCount; ++i)
                    if (arr[i].get<bool>())
                        cfg_out[i] = CfgPinRole::Safety;
            }
            else
                ROS_WARN("[IOManager]: 'outputs.safety' must have exactly 8 bool entries — skipping");
        }
    }

    buildLookup(cfg_in, cfg_out);
    ROS_INFO_STREAM("[IOManager]: Loaded signal names from '" << path << "'");
    return true;
}

void IOManager::buildLookup(const std::array<CfgPinRole, kCfgCount> &cfg_in, const std::array<CfgPinRole, kCfgCount> &cfg_out)
{
    digital_lookup_.clear();

    // Standard signals (always both input and output)
    for (size_t i = 0; i < kStdCount; ++i)
    {
        digital_lookup_[digital_input_names_[i]] = {DigitalKind::StdIn, static_cast<uint8_t>(i), i};
        digital_lookup_[digital_output_names_[i]] = {DigitalKind::StdOut, static_cast<uint8_t>(i), i};
    }

    // Configurable signals — input and output sides are fully independent
    for (size_t i = 0; i < kCfgCount; ++i)
    {
        const size_t bit = kStdCount + i;
        const uint8_t pin = static_cast<uint8_t>(i);

        digital_lookup_[digital_input_names_[bit]] =
            {cfg_in[i] == CfgPinRole::Safety ? DigitalKind::SafetyIn : DigitalKind::CfgIn, pin, bit};
        digital_lookup_[digital_output_names_[bit]] =
            {cfg_out[i] == CfgPinRole::Safety ? DigitalKind::SafetyOut : DigitalKind::CfgOut, pin, bit};
    }

    // Tool signals (always both input and output)
    for (size_t i = 0; i < kToolCount; ++i)
    {
        const size_t bit = kStdCount + kCfgCount + i;
        const uint8_t pin = static_cast<uint8_t>(i);
        digital_lookup_[digital_input_names_[bit]] = {DigitalKind::ToolIn, pin, bit};
        digital_lookup_[digital_output_names_[bit]] = {DigitalKind::ToolOut, pin, bit};
    }
}

// ---------------------------------------------------------------------------
//  Private static helpers
// ---------------------------------------------------------------------------

std::string IOManager::defaultInputName(size_t bit)
{
    if (bit < kStdCount)
        return "standard_in_" + std::to_string(bit);
    if (bit < kStdCount + kCfgCount)
        return "configurable_in_" + std::to_string(bit - kStdCount);
    return "tool_in_" + std::to_string(bit - kStdCount - kCfgCount);
}

std::string IOManager::defaultOutputName(size_t bit)
{
    if (bit < kStdCount)
        return "standard_out_" + std::to_string(bit);
    if (bit < kStdCount + kCfgCount)
        return "configurable_out_" + std::to_string(bit - kStdCount);
    return "tool_out_" + std::to_string(bit - kStdCount - kCfgCount);
}
