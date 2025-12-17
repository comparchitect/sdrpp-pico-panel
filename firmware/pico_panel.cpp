#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <string>
#include <array>
#include <algorithm>
#include <cstdlib>
#include <vector>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace {
constexpr uint kI2CPortSda = 0;
constexpr uint kI2CPortScl = 1;
constexpr uint8_t kOledAddress = 0x3C;
constexpr int kOledWidth = 128;
constexpr int kOledHeight = 64;
constexpr uint kPotSamples = 16;
constexpr uint kPotReportIntervalMs = 20;
constexpr uint kSwitchPollIntervalMs = 2;
constexpr uint kEncoderPollIntervalMs = 2;
constexpr uint kHostPollIntervalMs = 1;
constexpr uint kOledColOffset = 2;  // SH1106 column offset
constexpr bool kOledPortraitMode = true;
constexpr uint kOledGlyphWidth = 5;
constexpr uint kOledGlyphHeight = 8;
constexpr uint kOledFontScale = 2;
constexpr uint kOledCharSpacing = 2;
constexpr uint kOledCharWidth =
    (kOledGlyphWidth * kOledFontScale) + kOledCharSpacing;
constexpr uint kOledLogicalWidth =
    kOledPortraitMode ? kOledHeight : kOledWidth;
constexpr uint kOledLogicalHeight =
    kOledPortraitMode ? kOledWidth : kOledHeight;
constexpr uint kOledTextAreaWidth = kOledLogicalWidth;
constexpr uint kOledTextColOffset = (kOledLogicalWidth - kOledTextAreaWidth) / 2;
constexpr uint kOledTextBaseRow = 0;
constexpr uint kOledInitDelayMs = 100;
constexpr uint kOledRetryIntervalMs = 500;
constexpr const char kOledBootBanner[] = "SDR  Pico Panel";

struct PotChannel {
    uint gpioPin;
    uint adcInput;
    const char* label;
    uint16_t lastValue = 0;
};

struct Encoder {
    uint pinA;
    uint pinB;
    uint8_t lastState = 0;
    const char* label;
};

struct SwitchInput {
    uint pin;
    const char* label;
    bool activeState = false;
    uint8_t debounce = 0;
    bool lastReported = false;
};

constexpr PotChannel kPots[] = {
    {26, 0, "VOL"},
    {27, 1, "ZOOM"},
    {28, 2, "SQL"},
};

constexpr Encoder kEncoders[] = {
    {2, 3, 0, "ENC_FREQ"},
    {4, 5, 0, "ENC_BW"},
};

constexpr SwitchInput kSwitches[] = {
    {6,  "SW1"},
    {7,  "SW2"},
    {8,  "SW3"},
    {9,  "SW4"},
    {10, "SW5"},
    {11, "SW6"},
    {12, "SW7"},
    {13, "BTN_BW"},
    {22, "BTN_FREQ"},
};

constexpr uint kLedPins[] = {14, 15, 16, 17, 18, 19, 20, 21};

absolute_time_t g_nextPotSample;
absolute_time_t g_nextSwitchPoll;
absolute_time_t g_nextEncoderPoll;
absolute_time_t g_nextHostPoll;

uint8_t g_ledMask = 0;
std::array<uint8_t, kOledWidth * (kOledHeight / 8)> g_oledBuffer{};
bool g_oledInitialized = false;
absolute_time_t g_nextOledRetry;

std::string g_hostRxBuffer;

constexpr int8_t kEncoderLookup[4][4] = {
    {0, -1, +1, 0},
    {+1, 0, 0, -1},
    {-1, 0, 0, +1},
    {0, +1, -1, 0},
};

void sendLine(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
}

void initPotChannels() {
    adc_init();
    for (const auto& pot : kPots) {
        adc_gpio_init(pot.gpioPin);
    }
    g_nextPotSample = make_timeout_time_ms(kPotReportIntervalMs);
}

void initEncoders(std::array<Encoder, std::size(kEncoders)>& encState) {
    for (size_t i = 0; i < encState.size(); ++i) {
        encState[i] = kEncoders[i];
        gpio_init(encState[i].pinA);
        gpio_init(encState[i].pinB);
        gpio_set_dir(encState[i].pinA, GPIO_IN);
        gpio_set_dir(encState[i].pinB, GPIO_IN);
        gpio_pull_up(encState[i].pinA);
        gpio_pull_up(encState[i].pinB);
        encState[i].lastState =
            (gpio_get(encState[i].pinA) << 1) | gpio_get(encState[i].pinB);
    }
    g_nextEncoderPoll = make_timeout_time_ms(kEncoderPollIntervalMs);
}

void initSwitches(std::array<SwitchInput, std::size(kSwitches)>& switchState) {
    for (size_t i = 0; i < switchState.size(); ++i) {
        switchState[i] = kSwitches[i];
        gpio_init(switchState[i].pin);
        gpio_set_dir(switchState[i].pin, GPIO_IN);
        gpio_pull_up(switchState[i].pin);
        switchState[i].activeState = !gpio_get(switchState[i].pin);
        switchState[i].lastReported = switchState[i].activeState;
    }
    g_nextSwitchPoll = make_timeout_time_ms(kSwitchPollIntervalMs);
}

void initLeds() {
    for (uint pin : kLedPins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }
}

void setLedMask(uint8_t mask) {
    g_ledMask = mask;
    for (size_t i = 0; i < std::size(kLedPins); ++i) {
        bool on = mask & (1u << i);
        gpio_put(kLedPins[i], on ? 1 : 0);
    }
}

// --- OLED driver -----------------------------------------------------------

const uint8_t kFont5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x62,0x51,0x49,0x49,0x46}, {0x22,0x49,0x49,0x49,0x36},
    {0x18,0x14,0x12,0x7F,0x10}, {0x2F,0x49,0x49,0x49,0x31},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x03,0x01,0x71,0x09,0x07},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x59,0x09,0x06},
    {0x3E,0x41,0x5D,0x59,0x4E}, {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18},
    {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00},
    {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C},
    {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00},
    {0x10,0x08,0x08,0x10,0x08}, {0x00,0x00,0x00,0x00,0x00},
};

const uint8_t* glyphForChar(char c) {
    if (c < 0x20 || c > 0x7F) {
        return kFont5x7[0];
    }
    return kFont5x7[c - 0x20];
}

void setPhysicalPixel(uint x, uint y, bool on) {
    if (x >= kOledWidth || y >= kOledHeight) {
        return;
    }
    size_t index = (y / 8) * kOledWidth + x;
    uint8_t mask = 1u << (y % 8);
    if (on) {
        g_oledBuffer[index] |= mask;
    } else {
        g_oledBuffer[index] &= static_cast<uint8_t>(~mask);
    }
}

void setLogicalPixel(uint x, uint y, bool on) {
    if (x >= kOledLogicalWidth || y >= kOledLogicalHeight) {
        return;
    }
    if (!kOledPortraitMode) {
        setPhysicalPixel(x, y, on);
        return;
    }
    uint physX = y;
    uint physY = (kOledLogicalWidth - 1) - x;
    setPhysicalPixel(physX, physY, on);
}

bool oledSendCommand(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    int written = i2c_write_blocking(i2c0, kOledAddress, buf, 2, false);
    return written == 2;
}

bool oledFlush() {
    for (int page = 0; page < (kOledHeight / 8); ++page) {
        if (!oledSendCommand(0xB0 + page)) { return false; }
        if (!oledSendCommand(0x00 | (kOledColOffset & 0x0F))) { return false; }
        if (!oledSendCommand(0x10 | (kOledColOffset >> 4))) { return false; }
        uint8_t buf[1 + kOledWidth];
        buf[0] = 0x40;
        memcpy(buf + 1, &g_oledBuffer[page * kOledWidth], kOledWidth);
        int written = i2c_write_blocking(i2c0, kOledAddress, buf, sizeof(buf), false);
        if (written != static_cast<int>(sizeof(buf))) { return false; }
    }
    return true;
}

void scheduleOledRetry() {
    g_nextOledRetry = make_timeout_time_ms(kOledRetryIntervalMs);
}

void oledClearBuffer() {
    std::fill(g_oledBuffer.begin(), g_oledBuffer.end(), 0x00);
}

void oledDrawGlyph(uint x, uint y, const uint8_t* glyph) {
    for (uint col = 0; col < kOledGlyphWidth; ++col) {
        uint8_t columnBits = glyph[col];
        for (uint row = 0; row < kOledGlyphHeight; ++row) {
            bool on = columnBits & (1u << row);
            for (uint dx = 0; dx < kOledFontScale; ++dx) {
                for (uint dy = 0; dy < kOledFontScale; ++dy) {
                    setLogicalPixel(x + col * kOledFontScale + dx,
                                    y + row * kOledFontScale + dy, on);
                }
            }
        }
    }
}

void oledDrawTextLine(uint row, const std::string& text, uint colOffset = 0,
                      uint maxWidth = kOledLogicalWidth) {
    uint charHeight = kOledGlyphHeight * kOledFontScale;
    if (row >= (kOledLogicalHeight / charHeight) || colOffset >= kOledLogicalWidth) {
        return;
    }
    size_t maxCols = std::min<size_t>(maxWidth, kOledLogicalWidth - colOffset);
    uint cursor = 0;
    uint y = row * charHeight;
    uint glyphWidthScaled = kOledGlyphWidth * kOledFontScale;
    for (char c : text) {
        const uint8_t* glyph = glyphForChar(c);
        if (cursor + glyphWidthScaled > maxCols) {
            break;
        }
        oledDrawGlyph(colOffset + cursor, y, glyph);
        cursor += glyphWidthScaled;
        if (cursor >= maxCols) {
            break;
        }
        // spacing column
        for (uint col = 0; col < kOledCharSpacing && cursor + col < maxCols; ++col) {
            for (uint rowBit = 0; rowBit < charHeight; ++rowBit) {
                setLogicalPixel(colOffset + cursor + col, y + rowBit, false);
            }
        }
        cursor += kOledCharSpacing;
    }
    // Clear any remaining columns on the line segment
    while (cursor < maxCols) {
        for (uint rowBit = 0; rowBit < charHeight; ++rowBit) {
            setLogicalPixel(colOffset + cursor, y + rowBit, false);
        }
        ++cursor;
    }
}

std::vector<std::string> wrapTextForOled(const std::string& text, size_t maxChars) {
    std::vector<std::string> lines;
    if (maxChars == 0) {
        lines.emplace_back();
        return lines;
    }
    std::string current;
    for (char c : text) {
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (!current.empty()) {
                lines.push_back(current);
                current.clear();
            } else {
                lines.emplace_back();
            }
            continue;
        }
        if (current.size() >= maxChars) {
            lines.push_back(current);
            current.clear();
        }
        current.push_back(c);
    }
    if (!current.empty()) {
        lines.push_back(current);
    }
    if (lines.empty()) {
        lines.emplace_back();
    }
    return lines;
}

bool oledInit() {
    i2c_init(i2c0, 400000);
    gpio_set_function(kI2CPortSda, GPIO_FUNC_I2C);
    gpio_set_function(kI2CPortScl, GPIO_FUNC_I2C);
    gpio_pull_up(kI2CPortSda);
    gpio_pull_up(kI2CPortScl);
    sleep_ms(kOledInitDelayMs);

    const uint8_t initSeq[] = {
        0xAE, 0x02, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1,
        0xA6, 0xA8, 0x3F, 0xC8, 0xD3, 0x00, 0xD5, 0x80,
        0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14, 0xAF,
    };
    for (uint8_t cmd : initSeq) {
        if (!oledSendCommand(cmd)) {
            g_oledInitialized = false;
            return false;
        }
    }
    oledClearBuffer();
    if (!oledFlush()) {
        g_oledInitialized = false;
        return false;
    }
    g_oledInitialized = true;
    return true;
}

void updateOledText(const std::string& text) {
    if (!g_oledInitialized) { return; }
    oledClearBuffer();
    const size_t charsPerLine =
        std::max<size_t>(1, kOledTextAreaWidth / kOledCharWidth);
    std::vector<std::string> lines = wrapTextForOled(text, charsPerLine);
    const uint totalRows = kOledLogicalHeight / (kOledGlyphHeight * kOledFontScale);
    uint startRow = kOledTextBaseRow;
    if (startRow >= totalRows) {
        startRow = totalRows - 1;
    }
    const uint availableRows = totalRows - startRow;
    const size_t linesToDraw =
        std::min(lines.size(), static_cast<size_t>(availableRows));
    for (size_t i = 0; i < linesToDraw; ++i) {
        oledDrawTextLine(startRow + i, lines[i], kOledTextColOffset,
                         kOledTextAreaWidth);
    }
    if (!oledFlush()) {
        g_oledInitialized = false;
        scheduleOledRetry();
    }
}

// --- Host command parsing --------------------------------------------------

void handleHostCommand(const std::string& line) {
    if (line.rfind("LED:", 0) == 0) {
        int val = std::stoi(line.substr(4));
        setLedMask(static_cast<uint8_t>(val));
        return;
    }
    if (line.rfind("OLED:", 0) == 0) {
        std::string text = line.substr(5);
        std::replace(text.begin(), text.end(), '\t', '\n');
        updateOledText(text);
        return;
    }
}

void pollHost() {
    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch < 0) { break; }
        if (ch == '\n' || ch == '\r') {
            if (!g_hostRxBuffer.empty()) {
                handleHostCommand(g_hostRxBuffer);
                g_hostRxBuffer.clear();
            }
        } else {
            if (g_hostRxBuffer.size() < 128) {
                g_hostRxBuffer.push_back(static_cast<char>(ch));
            }
        }
    }
}

// --- Sampling --------------------------------------------------------------

void samplePots(std::array<PotChannel, std::size(kPots)>& pots) {
    if (!time_reached(g_nextPotSample)) {
        return;
    }
    g_nextPotSample = make_timeout_time_ms(kPotReportIntervalMs);
    for (auto& pot : pots) {
        adc_select_input(pot.adcInput);
        uint32_t accum = 0;
        for (uint i = 0; i < kPotSamples; ++i) {
            accum += adc_read();
        }
        uint16_t value = accum / kPotSamples;
        if (std::abs(static_cast<int>(value) - static_cast<int>(pot.lastValue)) > 8) {
            pot.lastValue = value;
            sendLine("%s:%u", pot.label, value);
        }
    }
}

void pollEncoders(std::array<Encoder, std::size(kEncoders)>& encState) {
    if (!time_reached(g_nextEncoderPoll)) {
        return;
    }
    g_nextEncoderPoll = make_timeout_time_ms(kEncoderPollIntervalMs);
    for (auto& enc : encState) {
        uint8_t state = (gpio_get(enc.pinA) << 1) | gpio_get(enc.pinB);
        int8_t delta = kEncoderLookup[enc.lastState][state];
        enc.lastState = state;
        if (delta != 0) {
            sendLine("%s:%d", enc.label, delta);
        }
    }
}

void pollSwitches(std::array<SwitchInput, std::size(kSwitches)>& switches) {
    if (!time_reached(g_nextSwitchPoll)) {
        return;
    }
    g_nextSwitchPoll = make_timeout_time_ms(kSwitchPollIntervalMs);
    for (auto& sw : switches) {
        bool state = !gpio_get(sw.pin);
        if (state == sw.activeState) {
            sw.debounce = 0;
            continue;
        }
        if (++sw.debounce >= 4) {
            sw.activeState = state;
            sw.debounce = 0;
            if (sw.lastReported != state) {
                sw.lastReported = state;
                sendLine("%s:%d", sw.label, state ? 1 : 0);
            }
        }
    }
}

}  // namespace

int main() {
    stdio_init_all();
    initLeds();
    g_nextOledRetry = make_timeout_time_ms(0);
    if (oledInit()) {
        updateOledText(kOledBootBanner);
    } else {
        scheduleOledRetry();
    }

    std::array<PotChannel, std::size(kPots)> pots{};
    for (size_t i = 0; i < pots.size(); ++i) { pots[i] = kPots[i]; }
    initPotChannels();

    std::array<Encoder, std::size(kEncoders)> encoders{};
    initEncoders(encoders);

    std::array<SwitchInput, std::size(kSwitches)> switches{};
    initSwitches(switches);

    g_nextHostPoll = make_timeout_time_ms(kHostPollIntervalMs);

    while (true) {
        samplePots(pots);
        pollEncoders(encoders);
        pollSwitches(switches);
        if (!g_oledInitialized && time_reached(g_nextOledRetry)) {
            if (oledInit()) {
                updateOledText(kOledBootBanner);
            } else {
                scheduleOledRetry();
            }
        }
        if (time_reached(g_nextHostPoll)) {
            pollHost();
            g_nextHostPoll = make_timeout_time_ms(kHostPollIntervalMs);
        }
        tight_loop_contents();
    }
    return 0;
}
