#include "pico_panel.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <gui/gui.h>
#include <gui/tuner.h>
#include <imgui.h>
#include <iostream>
#include <module.h>
#include <module_com.h>
#include <radio_interface.h>
#include <signal_path/signal_path.h>
#include <iomanip>
#if !defined(_WIN32)
#include <dlfcn.h>
#endif
#include <sstream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>
#include <iterator>
#include <core.h>

namespace {
constexpr const char* kDefaultSerialDevice = "/dev/ttyACM0";
constexpr float kAdcMax = 4095.0f;
constexpr std::chrono::milliseconds kReconnectDelay{1000};
constexpr std::chrono::milliseconds kIdleDelay{25};
constexpr std::chrono::milliseconds kStateRefreshInterval{500};
constexpr float kSmoothingAlpha = 0.2f;
constexpr float kChangeDeadband = 0.01f;
constexpr float kSquelchMinDb = -100.0f;
constexpr float kSquelchMaxDb = 0.0f;
constexpr float kSquelchDisableThreshold = 0.02f;
constexpr std::array<int, 3> kRadioGroup = {RADIO_IFACE_MODE_NFM, RADIO_IFACE_MODE_WFM, RADIO_IFACE_MODE_AM};
constexpr std::array<int, 4> kHamGroup = {RADIO_IFACE_MODE_USB, RADIO_IFACE_MODE_LSB, RADIO_IFACE_MODE_DSB, RADIO_IFACE_MODE_CW};
constexpr int kDsdDemodIdConst = 0x1301;
constexpr int kOldDsdDemodIdConst = 0x1302;
constexpr uint64_t kHomeFrequencyHz = 440000000ULL;
constexpr double kDefaultHomeBandwidthHz = 12000.0;

float smoothValue(std::atomic<float>& slot, float target) {
    float previous = slot.load();
    float smoothed = previous + kSmoothingAlpha * (target - previous);
    slot.store(smoothed);
    return smoothed;
}

using ZoomControlFn = void (*)(float);

ZoomControlFn resolveZoomControlFn() {
    static std::once_flag once;
    static ZoomControlFn fn = nullptr;
    std::call_once(once, []() {
#if !defined(_WIN32)
        void* sym = dlsym(RTLD_DEFAULT, "sdrpp_set_zoom_control");
        if (sym) {
            fn = reinterpret_cast<ZoomControlFn>(sym);
        }
#endif
    });
    return fn;
}

void ApplyZoomCompat(float value) {
    if (auto* fn = resolveZoomControlFn()) {
        try {
            (*fn)(value);
            return;
        } catch (...) {
            std::cerr << "[pico_panel] setZoomControl threw, falling back\n";
        }
    }
    core::configManager.acquire();
    core::configManager.conf["zoomBw"] = value;
    core::configManager.release(true);

    double factor = static_cast<double>(value) * static_cast<double>(value);
    double wfBw = gui::waterfall.getBandwidth();
    double delta = wfBw - 1000.0;
    double finalBw = std::min(1000.0 + (factor * delta), wfBw);
    gui::waterfall.setViewBandwidth(finalBw * gui::waterfall.getUsableSpectrumRatio());

    if (!gui::waterfall.selectedVFO.empty()) {
        auto* vfo = gui::waterfall.vfos[gui::waterfall.selectedVFO];
        if (vfo) {
            gui::waterfall.setViewOffset(vfo->centerOffset);
        }
    }
}

}  // namespace

// --- Module metadata --------------------------------------------------------

SDRPP_MOD_INFO{
    /* Name            */ "Pico Panel",
    /* Description     */ "Control SDR++ via Raspberry Pi Pico hardware panel",
    /* Author          */ "dimeco & contributors",
    /* Version         */ 0, 2, 0,
    /* Max instances   */ 1
};

#ifdef _WIN32
#error "This module currently targets Linux builds of SDR++ Brown."
#endif

// --- PicoPanelModule implementation -----------------------------------------

PicoPanelModule::PicoPanelModule() {
    const char* env = std::getenv("SDRPP_PICO_PANEL_PORT");
    serialDevice = env ? env : kDefaultSerialDevice;
    lastMessageTime = std::chrono::steady_clock::now();
    lastStateRefresh = lastMessageTime;
    dsdDemodId = kDsdDemodIdConst;
    oldDsdDemodId = kOldDsdDemodIdConst;
    loadHomePreset();
    std::cout << "[pico_panel] Using serial device " << serialDevice << std::endl;
}

PicoPanelModule::~PicoPanelModule() {
    stop();
    std::cout << "[pico_panel] Module destructed" << std::endl;
}

void PicoPanelModule::drawUI() {
    bool isConnected = connected.load();

    ImGui::TextUnformatted("Pi Pico Panel");
    ImGui::Separator();
    ImGui::Text("Serial port: %s", serialDevice.c_str());
    ImGui::Text("Status: %s", isConnected ? "connected" : "looking for Pico...");
    ImGui::ProgressBar(lastVolume.load(), ImVec2(-1, 0), "Volume");

    ImGui::Text("Zoom knob: %3.0f%%", lastZoom.load() * 100.0f);
    float squelchVal = lastSquelch.load();
    if (squelchVal <= kSquelchDisableThreshold) {
        ImGui::TextUnformatted("Squelch knob: off");
    }
    else {
        float db = kSquelchMinDb + squelchVal * (kSquelchMaxDb - kSquelchMinDb);
        ImGui::Text("Squelch level: %.1f dB", db);
    }

    ImGui::Separator();
    ImGui::Text("Frequency step: %.0f Hz", freqSteps[freqStepIndex.load()]);
    ImGui::Text("Bandwidth step: %.0f Hz", bandwidthSteps[bandwidthStepIndex.load()]);
    std::string switchBits;
    switchBits.reserve(7);
    for (int i = 0; i < 7; ++i) {
        switchBits.push_back(switchLatch[i] ? '1' : '0');
    }
    auto ledMask = lastLedMask.load();
    std::string ledBits;
    ledBits.reserve(7);
    for (int i = 0; i < 7; ++i) {
        bool on = (ledMask >> i) & 0x1;
        ledBits.push_back(on ? '1' : '0');
    }
    ImGui::Text("Switch state (GP6 to GP12): %s", switchBits.c_str());
    ImGui::Text("LED state (GP14 to GP20): %s", ledBits.c_str());

    ImGui::Separator();
    ImGui::TextUnformatted("Home Preset");
    HomePreset presetCopy;
    {
        std::lock_guard<std::mutex> lock(homePresetMutex);
        presetCopy = homePreset;
    }
    bool homeChanged = false;
    homeChanged |= ImGui::InputDouble("Home frequency (Hz)", &presetCopy.frequency, 0.0, 0.0, "%.3f");
    struct ModeOption { const char* label; int id; };
    const ModeOption modeOptions[] = {
        {"FM", RADIO_IFACE_MODE_NFM},
        {"WFM", RADIO_IFACE_MODE_WFM},
        {"AM", RADIO_IFACE_MODE_AM},
        {"USB", RADIO_IFACE_MODE_USB},
        {"LSB", RADIO_IFACE_MODE_LSB},
        {"DSB", RADIO_IFACE_MODE_DSB},
        {"CW", RADIO_IFACE_MODE_CW},
        {"RAW", RADIO_IFACE_MODE_RAW},
        {"DSD", dsdDemodId},
        {"ODSD", oldDsdDemodId},
    };
    const char* modeLabels[IM_ARRAYSIZE(modeOptions)];
    for (int i = 0; i < IM_ARRAYSIZE(modeOptions); ++i) {
        modeLabels[i] = modeOptions[i].label;
    }
    int modeIndex = 0;
    for (int i = 0; i < IM_ARRAYSIZE(modeOptions); ++i) {
        if (modeOptions[i].id == presetCopy.mode) {
            modeIndex = i;
            break;
        }
    }
    if (ImGui::Combo("Home mode", &modeIndex, modeLabels, IM_ARRAYSIZE(modeOptions))) {
        presetCopy.mode = modeOptions[modeIndex].id;
        homeChanged = true;
    }
    homeChanged |= ImGui::InputDouble("Home bandwidth (Hz)", &presetCopy.bandwidth, 0.0, 0.0, "%.1f");
    if (presetCopy.frequency < 0.0) { presetCopy.frequency = 0.0; }
    if (presetCopy.bandwidth < 0.0) { presetCopy.bandwidth = 0.0; }
    if (homeChanged) {
        {
            std::lock_guard<std::mutex> lock(homePresetMutex);
            homePreset = presetCopy;
        }
        saveHomePreset();
        oledDirty.store(true);
    }

    auto now = std::chrono::steady_clock::now();
    auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMessageTime).count();
    ImGui::Text("Last Pico update: %lld ms ago", static_cast<long long>(ageMs));
}

void PicoPanelModule::start() {
    bool expected = false;
    if (!running.compare_exchange_strong(expected, true)) {
        return;
    }
    worker = std::thread(&PicoPanelModule::ioLoop, this);
}

void PicoPanelModule::stop() {
    if (!running.exchange(false)) {
        return;
    }
    closeSerial();
    if (worker.joinable()) {
        worker.join();
    }
}

void PicoPanelModule::ioLoop() {
    while (running.load()) {
        bool needOpen = false;
        {
            std::lock_guard<std::mutex> lock(serialMutex);
            needOpen = (serialFd < 0);
        }

        auto now = std::chrono::steady_clock::now();
        if (connected.load() && (now - lastStateRefresh) > kStateRefreshInterval) {
            lastStateRefresh = now;
            requestStateSync();
        }

        if (needOpen) {
            if (!openSerial()) {
                connected.store(false);
                std::this_thread::sleep_for(kReconnectDelay);
                continue;
            }
            ledDirty.store(true);
            oledDirty.store(true);
        }

        int fdCopy = -1;
        {
            std::lock_guard<std::mutex> lock(serialMutex);
            fdCopy = serialFd;
        }

        if (fdCopy < 0) {
            std::this_thread::sleep_for(kReconnectDelay);
            continue;
        }

        char buf[256];
        ssize_t bytes = ::read(fdCopy, buf, sizeof(buf));
        if (bytes > 0) {
            rxBuffer.append(buf, static_cast<size_t>(bytes));
            size_t pos = 0;
            while ((pos = rxBuffer.find('\n')) != std::string::npos) {
                std::string line = rxBuffer.substr(0, pos);
                rxBuffer.erase(0, pos + 1);
                handleLine(line);
            }
            continue;
        }

        if (bytes == 0 || (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
            std::this_thread::sleep_for(kIdleDelay);
            continue;
        }

        std::cerr << "[pico_panel] Serial read error (" << errno << "): " << std::strerror(errno) << std::endl;
        closeSerial();
        connected.store(false);
        std::this_thread::sleep_for(kReconnectDelay);
    }
}

bool PicoPanelModule::openSerial() {
    int fd = ::open(serialDevice.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "[pico_panel] Failed to open " << serialDevice << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "[pico_panel] tcgetattr failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return false;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[pico_panel] tcsetattr failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return false;
    }

    tcflush(fd, TCIOFLUSH);

    std::lock_guard<std::mutex> lock(serialMutex);
    serialFd = fd;
    connected.store(true);
    std::cout << "[pico_panel] Connected to " << serialDevice << std::endl;
    lastStateRefresh = std::chrono::steady_clock::now();
    requestStateSync(true);
    return true;
}

void PicoPanelModule::closeSerial() {
    std::lock_guard<std::mutex> lock(serialMutex);
    if (serialFd >= 0) {
        ::close(serialFd);
        serialFd = -1;
    }
    connected.store(false);
}

void PicoPanelModule::handleLine(const std::string& line) {
    auto colon = line.find(':');
    if (colon == std::string::npos) {
        return;
    }

    const std::string prefix = line.substr(0, colon);
    const std::string valueStr = line.substr(colon + 1);

    try {
        int raw = std::stoi(valueStr);
        float normalized = static_cast<float>(raw) / kAdcMax;

        if (prefix == "VOL") {
            applyVolume(normalized);
        }
        else if (prefix == "ZOOM") {
            applyZoom(normalized);
        }
        else if (prefix == "SQL") {
            applySquelch(normalized);
        }
        else if (prefix == "ENC_FREQ") {
            handleEncoderDelta(prefix, raw);
        }
        else if (prefix == "ENC_BW") {
            handleEncoderDelta(prefix, raw);
        }
        else if (prefix == "BTN_FREQ") {
            if (raw != 0) { cycleFrequencyStep(); }
        }
        else if (prefix == "BTN_BW") {
            if (raw != 0) { cycleBandwidthStep(); }
        }
        else if (prefix.size() >= 2 && prefix[0] == 'S' && prefix[1] == 'W') {
            int idx = std::stoi(prefix.substr(2));
            handleSwitchEvent(idx, raw != 0);
        }
        else {
            return;
        }

        lastMessageTime = std::chrono::steady_clock::now();
    } catch (const std::exception& ex) {
        std::cerr << "[pico_panel] Failed to parse line '" << line << "': " << ex.what() << std::endl;
    }
}

void PicoPanelModule::applyVolume(float normalized) {
    float level = std::clamp(normalized, 0.0f, 1.0f);
    float smoothed = smoothValue(lastVolume, level);

    static std::atomic<float> lastApplied{0.0f};
    float prev = lastApplied.load();
    if (std::fabs(smoothed - prev) < kChangeDeadband) {
        return;
    }
    lastApplied.store(smoothed);

    lastVolume.store(smoothed);

    if (auto stream = sigpath::sinkManager.recentStreeam) {
        stream->setVolume(smoothed);
    }
}

void PicoPanelModule::applyZoom(float normalized) {
    float raw = std::clamp(normalized, 0.0f, 1.0f);
    float smoothed = smoothValue(lastZoom, raw);
    float zoomValue = 1.0f - smoothed;

    static std::atomic<float> lastApplied{0.0f};
    float prev = lastApplied.load();
    if (std::fabs(zoomValue - prev) < kChangeDeadband) {
        return;
    }
    lastApplied.store(zoomValue);

    gui::mainWindow.addMainThreadTask([zoomValue]() {
        ApplyZoomCompat(zoomValue);
    });
}

void PicoPanelModule::applySquelch(float normalized) {
    float level = std::clamp(normalized, 0.0f, 1.0f);
    float smoothed = smoothValue(lastSquelch, level);

    static std::atomic<float> lastApplied{0.0f};
    float prev = lastApplied.load();
    if (std::fabs(smoothed - prev) < kChangeDeadband) {
        return;
    }
    lastApplied.store(smoothed);

    gui::mainWindow.addMainThreadTask([smoothed]() {
        std::string vfoName = gui::waterfall.selectedVFO;
        if (vfoName.empty()) {
            vfoName = "Radio";
        }

        if (core::modComManager.getModuleName(vfoName) != "radio") {
            return;
        }

        if (smoothed <= kSquelchDisableThreshold) {
            bool disabled = false;
            core::modComManager.callInterface(vfoName, RADIO_IFACE_CMD_SET_SQUELCH_ENABLED, &disabled, nullptr);
            return;
        }

        bool enabled = true;
        float squelchDb = kSquelchMinDb + smoothed * (kSquelchMaxDb - kSquelchMinDb);
        core::modComManager.callInterface(vfoName, RADIO_IFACE_CMD_SET_SQUELCH_ENABLED, &enabled, nullptr);
        core::modComManager.callInterface(vfoName, RADIO_IFACE_CMD_SET_SQUELCH_LEVEL, &squelchDb, nullptr);
    });
}

void PicoPanelModule::handleEncoderDelta(const std::string& prefix, int delta) {
    if (delta == 0) { return; }
    if (prefix == "ENC_FREQ") {
        queueFrequencyDelta(delta);
    }
    else if (prefix == "ENC_BW") {
        queueBandwidthDelta(delta);
    }
}

void PicoPanelModule::handleSwitchEvent(int switchId, bool pressed) {
    if (switchId < 1 || switchId > 8) { return; }
    switchLatch[switchId - 1] = pressed;
    if (!pressed) { return; }

    switch (static_cast<PanelSwitch>(switchId)) {
        case PanelSwitch::Play:
            gui::mainWindow.addMainThreadTask([]() {
                bool running = gui::mainWindow.sdrIsRunning();
                gui::mainWindow.setPlayState(!running);
            });
            break;
        case PanelSwitch::RadioModes:
            handleRadioModeCycle();
            break;
        case PanelSwitch::HamModes:
            handleHamModeCycle();
            break;
        case PanelSwitch::DecoderModes:
            handleDecoderCycle();
            break;
        case PanelSwitch::MuteAll:
            toggleMuteAll();
            break;
        case PanelSwitch::Ft8:
            toggleFt8();
            break;
        case PanelSwitch::Home:
            goHomeChannel();
            break;
    }
    ledDirty.store(true);
}

void PicoPanelModule::cycleFrequencyStep() {
    int idx = freqStepIndex.load();
    idx = (idx + 1) % static_cast<int>(freqSteps.size());
    freqStepIndex.store(idx);
    oledDirty.store(true);
}

void PicoPanelModule::cycleBandwidthStep() {
    int idx = bandwidthStepIndex.load();
    idx = (idx + 1) % static_cast<int>(bandwidthSteps.size());
    bandwidthStepIndex.store(idx);
    oledDirty.store(true);
}

void PicoPanelModule::queueFrequencyDelta(int delta) {
    double step = freqSteps[freqStepIndex.load()];
    gui::mainWindow.addMainThreadTask([this, deltaHz = step * delta]() {
        std::string vfo = activeVfo();
        if (vfo.empty()) { return; }
        double current = static_cast<double>(gui::freqSelect.frequency);
        double target = std::max(0.0, current + deltaHz);
        tuner::tune(currentTuningMode(), vfo, target);
        oledDirty.store(true);
    });
}

void PicoPanelModule::queueBandwidthDelta(int delta) {
    float step = static_cast<float>(bandwidthSteps[bandwidthStepIndex.load()] * delta);
    gui::mainWindow.addMainThreadTask([this, step]() {
        std::string vfo = activeVfo();
        if (vfo.empty()) { return; }
        float bandwidth = 0.0f;
        if (!core::modComManager.callInterface(vfo, RADIO_IFACE_CMD_GET_BANDWIDTH, nullptr, &bandwidth)) {
            return;
        }
        float newBw = std::clamp(bandwidth + step, 100.0f, 2000000.0f);
        core::modComManager.callInterface(vfo, RADIO_IFACE_CMD_SET_BANDWIDTH, &newBw, nullptr);
        oledDirty.store(true);
    });
}

void PicoPanelModule::goHomeChannel() {
    HomePreset presetCopy;
    {
        std::lock_guard<std::mutex> lock(homePresetMutex);
        presetCopy = homePreset;
    }
    gui::mainWindow.addMainThreadTask([this, presetCopy]() {
        std::string vfo = activeVfo();
        if (vfo.empty()) { return; }
        double freq = presetCopy.frequency > 0 ? presetCopy.frequency
                                               : static_cast<double>(kHomeFrequencyHz);
        tuner::tune(currentTuningMode(), vfo, freq);
        int mode = presetCopy.mode;
        if (mode < 0) {
            mode = dsdDemodId;
        }
        core::modComManager.callInterface(vfo, RADIO_IFACE_CMD_SET_MODE, &mode, nullptr);
        float bandwidth = static_cast<float>(presetCopy.bandwidth);
        if (bandwidth > 0.0f) {
            core::modComManager.callInterface(vfo, RADIO_IFACE_CMD_SET_BANDWIDTH, &bandwidth, nullptr);
        }
        oledDirty.store(true);
    });
}

void PicoPanelModule::requestStateSync(bool force) {
    if (!force) {
        bool expected = false;
        if (!stateRefreshQueued.compare_exchange_strong(expected, true)) {
            return;
        }
    } else {
        stateRefreshQueued.store(true);
    }
    gui::mainWindow.addMainThreadTask([this, force]() {
        PanelIndicators indicators;
        refreshIndicators(indicators);
        updateLedState(force);
        sendOledStatus(force);
        stateRefreshQueued.store(false);
    });
}

bool PicoPanelModule::currentMuteState() {
    for (const auto& entry : sigpath::sinkManager.streams) {
        auto* stream = entry.second;
        if (stream && stream->volumeAjust.getMuted()) {
            return true;
        }
    }
    return false;
}

void PicoPanelModule::updateLedState(bool force) {
    if (!connected.load()) { return; }
    uint8_t mask = computeLedMask();
    if (!force && mask == lastLedMask.load() && !ledDirty.load()) {
        return;
    }
    sendLedMask(mask);
    ledDirty.store(false);
}

uint8_t PicoPanelModule::computeLedMask() {
    uint8_t mask = 0;
    if (gui::mainWindow.sdrIsRunning()) { mask |= (1u << 0); }

    auto demod = currentDemodId();
    if (std::find(kRadioGroup.begin(), kRadioGroup.end(), demod) != kRadioGroup.end()) {
        mask |= (1u << 1);
    }
    if (std::find(kHamGroup.begin(), kHamGroup.end(), demod) != kHamGroup.end()) {
        mask |= (1u << 2);
    }
    bool dsdActive = (demod == dsdDemodId || demod == oldDsdDemodId);
    if (dsdActive) {
        mask |= (1u << 3);
    }
    if (muteAllState.load()) { mask |= (1u << 4); }
    if (ft8State.load()) { mask |= (1u << 5); }

    HomePreset presetCopy;
    {
        std::lock_guard<std::mutex> lock(homePresetMutex);
        presetCopy = homePreset;
    }
    if (presetCopy.frequency > 0.0 && presetCopy.mode >= 0 && demod == presetCopy.mode) {
        uint64_t freq = static_cast<uint64_t>(gui::freqSelect.frequency + 0.5);
        uint64_t target = static_cast<uint64_t>(presetCopy.frequency + 0.5);
        if (freq == target) {
            mask |= (1u << 6);
        }
    }
    return mask;
}

void PicoPanelModule::sendLedMask(uint8_t mask) {
    lastLedMask.store(mask);
    std::ostringstream oss;
    oss << "LED:" << static_cast<int>(mask) << "\n";
    writeSerial(oss.str());
}

void PicoPanelModule::sendOledStatus(bool force) {
    if (!connected.load()) { return; }
    uint64_t freq = gui::freqSelect.frequency;
    int mode = currentDemodId();
    bool running = gui::mainWindow.sdrIsRunning();
    double freqStep = freqSteps[freqStepIndex.load()];
    double bwStep = bandwidthSteps[bandwidthStepIndex.load()];

    std::ostringstream oss;
    oss << (running ? "ON" : "OFF") << "\n";
    oss << "\n";
    oss << demodName(mode) << "\n";
    oss << "\n";
    oss << "F " << formattedStep(freqStep) << "\n";
    oss << "\n";
    oss << "B " << formattedStep(bwStep) << "\n";
    oss << "\n";

    std::string payload = oss.str();
    if (!force && !oledDirty.load() && payload == lastOledText) {
        return;
    }
    lastOledText = payload;
    oledDirty.store(false);
    std::string encoded = payload;
    std::replace(encoded.begin(), encoded.end(), '\n', '\t');
    writeSerial("OLED:" + encoded + "\n");
}

void PicoPanelModule::refreshIndicators(PanelIndicators& indicators) {
    indicators.playing = gui::mainWindow.sdrIsRunning();

    int demod = currentDemodId();
    indicators.radioGroup = std::find(kRadioGroup.begin(), kRadioGroup.end(), demod) != kRadioGroup.end();
    indicators.hamGroup = std::find(kHamGroup.begin(), kHamGroup.end(), demod) != kHamGroup.end();
    indicators.dsdGroup = (demod == dsdDemodId || demod == oldDsdDemodId);

    bool muted = currentMuteState();
    muteAllState.store(muted);
    indicators.muteAll = muted;

    bool ft8 = core::moduleManager.instanceEnabled("FT8/FT4 Decoder");
    ft8State.store(ft8);
    indicators.ft8Active = ft8;
}

int PicoPanelModule::currentDemodId() {
    std::string vfo = activeVfo();
    int mode = RADIO_IFACE_MODE_NFM;
    core::modComManager.callInterface(vfo, RADIO_IFACE_CMD_GET_MODE, nullptr, &mode);
    return mode;
}

void PicoPanelModule::cycleDemodulation(const std::vector<int>& demods) {
    if (demods.empty()) { return; }
    std::string vfo = activeVfo();
    int current = currentDemodId();
    auto it = std::find(demods.begin(), demods.end(), current);
    int next = demods[(it == demods.end() ? 0 : (std::distance(demods.begin(), it) + 1) % demods.size())];
    core::modComManager.callInterface(vfo, RADIO_IFACE_CMD_SET_MODE, &next, nullptr);
}

void PicoPanelModule::handleRadioModeCycle() {
    cycleDemodulation(std::vector<int>(kRadioGroup.begin(), kRadioGroup.end()));
    oledDirty.store(true);
}

void PicoPanelModule::handleHamModeCycle() {
    cycleDemodulation(std::vector<int>(kHamGroup.begin(), kHamGroup.end()));
    oledDirty.store(true);
}

void PicoPanelModule::handleDecoderCycle() {
    std::vector<int> options = {dsdDemodId, oldDsdDemodId};
    cycleDemodulation(options);
    oledDirty.store(true);
}

void PicoPanelModule::toggleMuteAll() {
    bool newState = !muteAllState.load();
    muteAllState.store(newState);
    gui::mainWindow.addMainThreadTask([newState]() {
        sigpath::sinkManager.setAllMuted(newState);
    });
    ledDirty.store(true);
}

void PicoPanelModule::toggleFt8() {
    bool newState = !ft8State.load();
    ft8State.store(newState);
    gui::mainWindow.addMainThreadTask([newState]() {
        if (newState) {
            core::moduleManager.enableInstance("FT8/FT4 Decoder");
        } else {
            core::moduleManager.disableInstance("FT8/FT4 Decoder");
        }
    });
    ledDirty.store(true);
    oledDirty.store(true);
}

void PicoPanelModule::loadHomePreset() {
    HomePreset updated;
    updated.frequency = static_cast<double>(kHomeFrequencyHz);
    updated.bandwidth = kDefaultHomeBandwidthHz;
    updated.mode = RADIO_IFACE_MODE_NFM;
    core::configManager.acquire();
    try {
        if (core::configManager.conf.contains("picoPanel")) {
            auto& cfg = core::configManager.conf["picoPanel"];
            if (cfg.contains("home")) {
                auto& home = cfg["home"];
                if (home.contains("frequency")) {
                    updated.frequency = home["frequency"].get<double>();
                }
                if (home.contains("bandwidth")) {
                    updated.bandwidth = home["bandwidth"].get<double>();
                }
                if (home.contains("mode")) {
                    updated.mode = home["mode"].get<int>();
                }
            }
        }
    } catch (...) {
        // Ignore malformed config; defaults already set.
    }
    core::configManager.release();
    std::lock_guard<std::mutex> lock(homePresetMutex);
    homePreset = updated;
}

void PicoPanelModule::saveHomePreset() {
    HomePreset copy;
    {
        std::lock_guard<std::mutex> lock(homePresetMutex);
        copy = homePreset;
    }
    core::configManager.acquire();
    auto& home = core::configManager.conf["picoPanel"]["home"];
    home["frequency"] = copy.frequency;
    home["bandwidth"] = copy.bandwidth;
    home["mode"] = copy.mode;
    core::configManager.release(true);
}

std::string PicoPanelModule::formattedFrequency(uint64_t hz) const {
    std::ostringstream oss;
    if (hz < 1000ULL) {
        oss << hz << " Hz";
    }
    else if (hz < 1000000ULL) {
        oss << std::fixed << std::setprecision(3) << (hz / 1000.0) << " kHz";
    }
    else {
        oss << std::fixed << std::setprecision(4) << (hz / 1000000.0) << " MHz";
    }
    return oss.str();
}

std::string PicoPanelModule::formattedStep(double hz) const {
    if (hz <= 0.0) {
        return "0";
    }
    int exponent = 0;
    double mantissa = hz;
    while (mantissa >= 10.0) {
        mantissa /= 10.0;
        ++exponent;
    }
    while (mantissa < 1.0) {
        mantissa *= 10.0;
        --exponent;
    }
    int mant = static_cast<int>(std::round(mantissa));
    if (mant == 10) {
        mant = 1;
        ++exponent;
    }
    std::ostringstream oss;
    oss << mant << "E" << exponent;
    return oss.str();
}

std::string PicoPanelModule::demodName(int demod) const {
    if (demod == dsdDemodId) { return "DSD"; }
    if (demod == oldDsdDemodId) { return "ODSD"; }
    switch (demod) {
        case RADIO_IFACE_MODE_NFM: return "FM";
        case RADIO_IFACE_MODE_WFM: return "WFM";
        case RADIO_IFACE_MODE_AM:  return "AM";
        case RADIO_IFACE_MODE_USB: return "USB";
        case RADIO_IFACE_MODE_LSB: return "LSB";
        case RADIO_IFACE_MODE_DSB: return "DSB";
        case RADIO_IFACE_MODE_CW:  return "CW";
        case RADIO_IFACE_MODE_RAW: return "RAW";
        default: return "MODE";
    }
}

std::string PicoPanelModule::activeVfo() const {
    if (!gui::waterfall.selectedVFO.empty()) {
        return gui::waterfall.selectedVFO;
    }
    return "Radio";
}

int PicoPanelModule::currentTuningMode() const {
    bool center = false;
    core::configManager.acquire();
    if (core::configManager.conf.contains("centerTuning")) {
        center = core::configManager.conf["centerTuning"];
    }
    core::configManager.release();
    return center ? tuner::TUNER_MODE_CENTER : tuner::TUNER_MODE_NORMAL;
}

void PicoPanelModule::writeSerial(const std::string& line) {
    std::lock_guard<std::mutex> writer(sendMutex);
    std::lock_guard<std::mutex> lock(serialMutex);
    if (serialFd < 0) { return; }
    ::write(serialFd, line.c_str(), line.size());
}

// --- ModuleManager bridge ---------------------------------------------------

class PicoPanelInstance : public ModuleManager::Instance {
public:
    explicit PicoPanelInstance(std::string instanceName)
        : name(std::move(instanceName)) {
        gui::menu.registerEntry(name, menuHandler, this, this);
        std::cout << "[pico_panel] Instance '" << name << "' created" << std::endl;
    }

    ~PicoPanelInstance() override {
        gui::menu.removeEntry(name);
        panel.stop();
        std::cout << "[pico_panel] Instance '" << name << "' destroyed" << std::endl;
    }

    void postInit() override {
        std::cout << "[pico_panel] postInit for '" << name << "'" << std::endl;
        enable();
    }

    void enable() override {
        if (enabled) { return; }
        enabled = true;
        panel.start();
        std::cout << "[pico_panel] '" << name << "' enabled" << std::endl;
    }

    void disable() override {
        if (!enabled) { return; }
        enabled = false;
        panel.stop();
        std::cout << "[pico_panel] '" << name << "' disabled" << std::endl;
    }

    bool isEnabled() override {
        return enabled;
    }

private:
    static void menuHandler(void* ctx) {
        auto* self = static_cast<PicoPanelInstance*>(ctx);
        self->panel.drawUI();
    }

    std::string name;
    bool enabled = false;
    PicoPanelModule panel;
};

// --- SDR++ module entry points ----------------------------------------------

MOD_EXPORT void _INIT_() {
    std::cout << "[pico_panel] Module init" << std::endl;
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new PicoPanelInstance(std::move(name));
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete instance;
}

MOD_EXPORT void _END_() {
    std::cout << "[pico_panel] Module shutdown" << std::endl;
}
