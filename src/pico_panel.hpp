#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <radio_interface.h>

class PicoPanelModule {
public:
    PicoPanelModule();
    ~PicoPanelModule();

    void drawUI();

    void start();
    void stop();

private:
    enum class PanelSwitch {
        Play = 1,
        RadioModes = 2,
        HamModes = 3,
        DecoderModes = 4,
        MuteAll = 5,
        Ft8 = 6,
        Home = 7,
    };

    struct HomePreset {
        double frequency = 0.0;
        double bandwidth = 0.0;
        int mode = RADIO_IFACE_MODE_NFM;
    };

    struct PanelIndicators {
        bool playing = false;
        bool radioGroup = false;
        bool hamGroup = false;
        bool dsdGroup = false;
        bool muteAll = false;
        bool ft8Active = false;
    };

    void ioLoop();
    bool openSerial();
    void closeSerial();
    void handleLine(const std::string& line);
    void applyVolume(float normalized);
    void applyZoom(float normalized);
    void applySquelch(float normalized);
    void handleEncoderDelta(const std::string& prefix, int delta);
    void handleSwitchEvent(int switchId, bool pressed);
    void cycleFrequencyStep();
    void cycleBandwidthStep();
    void queueFrequencyDelta(int delta);
    void queueBandwidthDelta(int delta);
    void goHomeChannel();
    void loadHomePreset();
    void saveHomePreset();
    void requestStateSync(bool force = false);
    bool currentMuteState();
    void updateLedState(bool force = false);
    void sendLedMask(uint8_t mask);
    void sendOledStatus(bool force = false);
    uint8_t computeLedMask();
    void refreshIndicators(PanelIndicators& indicators);
    void cycleDemodulation(const std::vector<int>& demods);
    void toggleMuteAll();
    void toggleFt8();
    void handleRadioModeCycle();
    void handleHamModeCycle();
    void handleDecoderCycle();
    std::string formattedFrequency(uint64_t hz) const;
    std::string formattedStep(double hz) const;
    std::string demodName(int demod) const;
    std::string activeVfo() const;
    int currentTuningMode() const;
    int currentDemodId();
    void writeSerial(const std::string& line);

    std::string serialDevice;
    std::thread worker;
    std::atomic<bool> running{false};
    std::atomic<bool> connected{false};
    std::atomic<float> lastVolume{0.0f};
    std::atomic<float> lastZoom{0.0f};
    std::atomic<float> lastSquelch{0.0f};
    std::atomic<int> freqStepIndex{0};
    std::atomic<int> bandwidthStepIndex{0};
    std::array<double, 7> freqSteps{{50.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0, 10000000.0}};
    std::array<double, 3> bandwidthSteps{{50.0, 100.0, 1000.0}};
    HomePreset homePreset{};
    std::string lastOledText;
    std::atomic<uint8_t> lastLedMask{0};
    std::atomic<bool> ledDirty{true};
    std::atomic<bool> oledDirty{true};
    std::atomic<bool> muteAllState{false};
    std::atomic<bool> ft8State{false};
    std::atomic<bool> stateRefreshQueued{false};
    std::array<bool, 8> switchLatch{};
    std::chrono::steady_clock::time_point lastMessageTime;
    std::chrono::steady_clock::time_point lastStateRefresh;
    int serialFd = -1;
    std::mutex serialMutex;
    std::mutex sendMutex;
    mutable std::mutex homePresetMutex;
    std::string rxBuffer;
    int dsdDemodId = 0x1301;
    int oldDsdDemodId = 0x1302;
};
