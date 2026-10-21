/*****************************************************************
 * **/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <LittleFS.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#define LORA_UART_BAUD   9600
#define LORA_FRAME_SYNC  0xAA
#define LORA_MAX_FRAME   96
#define LORA_MAX_PAYLOAD (LORA_MAX_FRAME - 3)
#define LORA_BYTE_TIMEOUT_MS 200UL

class LoRaE220Class {
public:
    bool begin(uint32_t freqHz = 433125000UL) {
        _ready = false;
        if (_serial == nullptr) return false;

        pinMode(_m0, OUTPUT);
        pinMode(_m1, OUTPUT);
        pinMode(_aux, INPUT);
        digitalWrite(_m0, LOW);
        digitalWrite(_m1, LOW);
        _serial->begin(LORA_UART_BAUD, SERIAL_8N1, _rxPin, _txPin);
        delay(50);
        if (!waitAuxHigh(2000)) {
            Serial.println("[LORA] AUX khong len HIGH luc khoi dong.");
            return false;
        }

        long channelCalc = lround((freqHz / 1000000.0) - 410.125);
        if (channelCalc < 0 || channelCalc > 83) {
            Serial.printf("[LORA] Tan so ngoai dai E220: %lu Hz\n", (unsigned long)freqHz);
            return false;
        }
        uint8_t channel = (uint8_t)channelCalc;

        if (!enterConfigMode()) {
            Serial.println("[LORA] Khong vao duoc che do cau hinh (AUX timeout).");
            enterNormalMode();
            return false;
        }
        bool cfgOk = pushConfig(channel);
        bool normalOk = enterNormalMode();
        _ready = cfgOk && normalOk;
        return _ready;
    }

    void setPins(uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin,
                 HardwareSerial &serialPort, uint8_t rxPin, uint8_t txPin) {
        _m0 = m0Pin; _m1 = m1Pin; _aux = auxPin;
        _serial = &serialPort; _rxPin = rxPin; _txPin = txPin;
    }

    void setSignalBandwidth(long) {}
    void setSpreadingFactor(int) {}
    void setCodingRate4(int) {}
    void enableCrc() {}

    void setTxPower(int dBm) {
        if (dBm >= 30)      _txPowerBits = 0b00;
        else if (dBm >= 27) _txPowerBits = 0b01;
        else if (dBm >= 24) _txPowerBits = 0b10;
        else                _txPowerBits = 0b11;
        if (_ready) {
            if (!enterConfigMode()) { _ready = false; return; }
            bool cfgOk = pushConfig(_channel);
            bool normalOk = enterNormalMode();
            _ready = cfgOk && normalOk;
        }
    }

    bool beginPacket() { _txLen = 0; return _ready && _serial != nullptr; }

    size_t write(const uint8_t *buf, size_t len) {
        if (!_ready || _serial == nullptr || (buf == nullptr && len != 0)) return 0;
        if (len > sizeof(_txBuf) - _txLen) return 0;
        if (len != 0) memcpy(_txBuf + _txLen, buf, len);
        _txLen += (uint8_t)len;
        return len;
    }

    bool endPacket(bool /*async*/ = false) {
        if (!_ready || _serial == nullptr || _txLen == 0 || _txLen > sizeof(_txBuf)) return false;
        uint8_t frame[LORA_MAX_FRAME];
        size_t fi = 0;
        frame[fi++] = LORA_FRAME_SYNC;
        frame[fi++] = _txLen;
        memcpy(frame + fi, _txBuf, _txLen); fi += _txLen;
        frame[fi++] = crc8(_txBuf, _txLen);

        if (!waitAuxHigh(100)) return false;
        return _serial->write(frame, fi) == fi;
    }

    int parsePacket() {
        if (_serial == nullptr) return 0;
        expireStale();
        if (_rxReady) { _rxReady = false; return _rxLen; }
        while (_serial->available() && !_rxReady) feed((uint8_t)_serial->read());
        if (_rxReady) { _rxReady = false; return _rxLen; }
        return 0;
    }

    int available() { return (int)_rxLen - (int)_rxReadPos; }

    int read() {
        if (_rxReadPos >= _rxLen) return -1;
        return _rxBuf[_rxReadPos++];
    }

    void readBytes(uint8_t *buf, size_t len) {
        if (buf == nullptr || len == 0) return;
        int avail = available();
        size_t n = (avail <= 0) ? 0 : (((size_t)avail < len) ? (size_t)avail : len);
        if (n) memcpy(buf, _rxBuf + _rxReadPos, n);
        _rxReadPos += (uint8_t)n;
    }

    int packetRssi() { return _lastRssi; }

private:
    uint8_t _m0 = 0, _m1 = 0, _aux = 0, _rxPin = 0, _txPin = 0;
    HardwareSerial *_serial = nullptr;
    bool    _ready = false;
    uint8_t _channel = 23;
    uint8_t _txPowerBits = 0;

    uint8_t _txBuf[LORA_MAX_PAYLOAD]; uint8_t _txLen = 0;
    uint8_t _rxBuf[LORA_MAX_FRAME]; uint8_t _rxLen = 0; uint8_t _rxReadPos = 0;
    bool    _rxReady = false;
    int     _lastRssi = 0;

    enum RxState { WAIT_SYNC, WAIT_LEN, WAIT_DATA, WAIT_CRC, WAIT_RSSI };
    RxState _state = WAIT_SYNC;
    uint8_t _frameLen = 0, _frameIdx = 0;
    uint8_t _frameBuf[LORA_MAX_FRAME];

    unsigned long _lastByteMs = 0;

    void expireStale() {
        if (_state == WAIT_SYNC) return;
        if (millis() - _lastByteMs < LORA_BYTE_TIMEOUT_MS) return;
        if (_state == WAIT_RSSI) {
            memcpy(_rxBuf, _frameBuf, _frameLen);
            _rxLen = _frameLen; _rxReadPos = 0; _rxReady = true;
        }
        _state = WAIT_SYNC;
    }

    void feed(uint8_t b) {
        _lastByteMs = millis();
        switch (_state) {
            case WAIT_SYNC:
                if (b == LORA_FRAME_SYNC) _state = WAIT_LEN;
                break;
            case WAIT_LEN:
                if (b == 0 || b > sizeof(_frameBuf)) { _state = WAIT_SYNC; break; }
                _frameLen = b; _frameIdx = 0; _state = WAIT_DATA;
                break;
            case WAIT_DATA:
                _frameBuf[_frameIdx++] = b;
                if (_frameIdx >= _frameLen) _state = WAIT_CRC;
                break;
            case WAIT_CRC:
                if (crc8(_frameBuf, _frameLen) == b) {
                    _state = WAIT_RSSI;
                } else {
                    _state = WAIT_SYNC;
                }
                break;
            case WAIT_RSSI:
                _lastRssi = (int)b - 256;
                memcpy(_rxBuf, _frameBuf, _frameLen);
                _rxLen = _frameLen; _rxReadPos = 0; _rxReady = true;
                _state = WAIT_SYNC;
                break;
        }
    }

    static uint8_t crc8(const uint8_t *data, uint8_t len) {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; b++)
                crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
        return crc;
    }

    bool waitAuxHigh(unsigned long timeoutMs) {
        if (_serial == nullptr) return false;
        const uint32_t t0 = millis();
        while (digitalRead(_aux) == LOW) {
            if ((uint32_t)(millis() - t0) >= timeoutMs) return false;
            delay(2);
        }
        delay(2);
        return digitalRead(_aux) == HIGH;
    }

    bool enterConfigMode() {
        _ready = false;
        digitalWrite(_m0, HIGH); digitalWrite(_m1, HIGH);
        delay(20);
        return waitAuxHigh(1000);
    }

    bool enterNormalMode() {
        digitalWrite(_m0, LOW); digitalWrite(_m1, LOW);
        delay(20);
        bool ok = waitAuxHigh(1000);
        _ready = ok;
        return ok;
    }

    bool pushConfig(uint8_t channel) {
        if (_serial == nullptr || channel > 83) return false;
        _channel = channel;
        uint8_t cmd[11] = {
            0xC0, 0x00, 0x08,
            0xFF, 0xFF,
            0x62,
            (uint8_t)(_txPowerBits & 0x03),
            channel,
            0x80,
            0x00, 0x00
        };
        while (_serial->available()) (void)_serial->read();
        if (_serial->write(cmd, sizeof(cmd)) != sizeof(cmd)) return false;
        unsigned long t0 = millis();
        uint8_t resp[sizeof(cmd)] = {0};
        uint8_t ri = 0;
        while ((uint32_t)(millis() - t0) < 500UL && ri < sizeof(resp)) {
            if (_serial->available()) resp[ri++] = (uint8_t)_serial->read();
            else delay(1);
        }
        bool ok = (ri == sizeof(resp) && resp[0] == 0xC1 && resp[1] == cmd[1] && resp[2] == cmd[2]);
        if (ok) {
            for (size_t i = 3; i < sizeof(resp); ++i) {
                if (resp[i] != cmd[i]) { ok = false; break; }
            }
        }
        if (!ok) {
            Serial.printf("[LORA-CFG] THAT BAI - resp count=%d, hex: ", ri);
            for (uint8_t i = 0; i < ri; i++) Serial.printf("%02X ", resp[i]);
            Serial.println();
        }
        return ok;
    }
};

LoRaE220Class LoRa;

#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <Preferences.h>
#include <queue>
#include <time.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define WIFI_SSID_DEFAULT "VuThiMai"
#define WIFI_PASS_DEFAULT "09121958"

#define BOT_TOKEN_DEFAULT "8777904636:AAGMBfWJDtN8WvsUtXhYT9SEXBfAnKL9ySE"
#define CHAT_ID_DEFAULT   "8752050398"

static void selfHealNoteCause(const char *why);

#define FW_VERSION        "20.0"
#define OTA_GITHUB_OWNER  "Trong99x"
#define OTA_GITHUB_REPO   "trambom"
#define OTA_GITHUB_BRANCH "main"
#define OTA_MANIFEST_URL  "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/versions.json"
#define OTA_RAW_BASE_URL  "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/"
#define OTA_MAX_VERSIONS  15

#define MASTER_ID    0
#define SLAVE_M1_ID  1
#define SLAVE_M2_ID  2
#define SLAVE_H_ID   3

#define LORA_M0        5
#define LORA_M1       18
#define LORA_AUX      19
#define LORA_UART_RX  23
#define LORA_UART_TX  14

#define LORA_FREQ  433E6
#define LORA_BW    125E3
#define LORA_SF    12
#define LORA_CR    5
#define TX_POWER   10
#define LORA_INIT_MAX_RETRY  5
#define RUNTIME_CHECKPOINT_MS 300000UL

// Tự động reset định kỳ mỗi 6 giờ (21600000 ms) khi tất cả bơm tắt
#define AUTO_REBOOT_INTERVAL_MS 21600000UL
#define AUTO_REBOOT_CHECK_MS      660000UL

#define FLOAT_MASTER_PIN 32
#define HB_OUT_PIN        26
#define HB_UART_TX        13     // UART 9600 chỉ phát -> chân G32 của mạch reset (nối chung GND)
#define HB_ST_MS        5000UL   // gửi dòng trạng thái mỗi 5 giây
// Chân HB_OUT_PIN chỉ nháy khi TẤT CẢ điều kiện dưới đây còn tốt; sai một cái -> đứng yên -> mạch ngoài reset chip.
#define HB_LOOP_ALIVE_MS   10000UL     // loop() không chạy quá 10 giây
#define HB_LORA_ALIVE_MS   30000UL     // vLoRaRealtimeTask không chạy quá 30 giây
#define HB_TG_STUCK_MS    300000UL     // vTelegramTask kẹt quá 5 phút (bỏ qua khi đang OTA)
#define HB_TRAM_STUCK_MS  120000UL     // vTramPushTask kẹt quá 2 phút
#define HB_WIFI_LOST_MS  1800000UL     // mất WiFi liên tục quá 30 phút
#define HB_TG_BAD_MS     1500000UL     // WiFi còn nhưng Telegram không phản hồi quá 25 phút
#define TG_PROBE_MS       300000UL     // 5 phút thăm dò Telegram 1 lần (getMe)
volatile uint32_t loopAliveMs = 0, loraAliveMs = 0, tgAliveMs = 0, tramAliveMs = 0, tgLastOkMs = 0;
const char * volatile hbReason = NULL;     // NULL = chân 26 đang nháy bình thường; khác NULL = lý do đã ngừng nháy
volatile bool bootAlertPending = false;

#define RELAY_M1_PIN     27
#define RELAY_M2_PIN     25

#define WATER_TRIG_PIN   33
#define WATER_ECHO_PIN   34

#define WATER_TANK_HEIGHT_CM_DEFAULT   200
#define WATER_FULL_OFFSET_CM_DEFAULT   0
#define WATER_READ_INTERVAL_MS 60000UL
#define WATER_PULSE_TIMEOUT_US 30000UL
#define WATER_SAMPLE_COUNT     3
#define WATER_SENSOR_ENABLED_DEFAULT false
#define WATER_LOW_PERCENT      15
#define WATER_HIGH_PERCENT     95
#define WATER_ALERT_DEBOUNCE_MS 60000UL

#define WATER_PUMP_BLOCK_LOW_PERCENT_DEFAULT     20
#define WATER_PUMP_BLOCK_HIGH_PERCENT_DEFAULT    96
#define WATER_PUMP_HIGH_RECOVER_PERCENT_DEFAULT  90
#define WATER_PUMP_H_ALONE_STOP_PERCENT          100

#define HEARTBEAT_MS              60000UL
#define SLAVE_TIMEOUT_MS         600000UL
#define FLOAT_DEBOUNCE_MS          5000UL
#define ACK_DELAY_MS               2500UL
#define CMD_DELAY_MS              3000UL
#define PUMP_H_RESYNC_MS         150000UL
#define TELEGRAM_POLL_MS           5000UL   
#define LOOP_DELAY_ACTIVE_MS         20UL
#define LOOP_DELAY_IDLE_MS          300UL
#define WDT_TIMEOUT_SEC               60
#define LORA_MUTEX_TIMEOUT_MS      300UL
#define TELEGRAM_HTTP_TIMEOUT_SEC    10
#define STATUS_CACHE_TTL_MS       20000UL
#define CMD_MAX_RETRIES               5
#define BOOT_GRACE_MS            120000UL



#define FLOAT_FULL    1
#define FLOAT_LOW     0
#define FLOAT_UNKNOWN 2
#define RELAY_ON    HIGH
#define RELAY_OFF   LOW
#define MODE_AUTO   0
#define MODE_MANUAL 1

#define CMD_REQUEST_STATUS 0xFE
#define FLOAT_DATA_VALID_BIT 0x80

#define UI_DIVIDER   "━━━━━━━━━━━━━━━━━━━━"
#define ICON_OK      "🟢"
#define ICON_BAD     "🔴"
#define ICON_WARN    "⚠️"
#define ICON_CRIT    "🚨"
#define ICON_SUCCESS "✅"
#define ICON_INFO    "ℹ️"

#define ALERT_LOG_SIZE 8
SemaphoreHandle_t alertLogMutex = NULL;
String       alertLog[ALERT_LOG_SIZE];
int          alertLogIdx = 0;

enum MessageType : uint8_t {
    MSG_HEARTBEAT = 1,
    MSG_FLOAT     = 2,
    MSG_COMMAND   = 3,
    MSG_ACK       = 4,
    MSG_OTA_START = 10,
    MSG_OTA_DATA  = 11,
    MSG_OTA_ACK   = 12,
    MSG_OTA_END   = 13,
    MSG_OTA_ABORT = 14
};

typedef struct __attribute__((packed)) {
    uint8_t  sender;
    uint8_t  receiver;
    uint8_t  msgType;
    uint16_t seq;
    uint8_t  payload;
    uint32_t uptime;
} Packet;

#define OTA_CHUNK_SIZE 64
typedef struct __attribute__((packed)) {
    uint8_t  sender;
    uint8_t  receiver;
    uint8_t  msgType;
    uint16_t chunkIndex;
    uint16_t chunkLen;
    uint32_t totalSize;
    uint32_t crc32;
    uint8_t  data[OTA_CHUNK_SIZE];
} OtaPacket;

uint32_t crc32Compute(uint32_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (~(crc & 1) + 1));
    }
    return crc;
}

#define QUEUE_SIZE 64
struct PendingEntry {
    Packet        pkt;
    bool          used;
    uint8_t       retries;
    unsigned long lastSentAt;
};
PendingEntry pendingTable[QUEUE_SIZE];
portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;

String                currentBotToken = BOT_TOKEN_DEFAULT;
String                currentChatId   = CHAT_ID_DEFAULT;
Preferences           tgPrefs;
Preferences           configPrefs;
Preferences           floatPrefs;

WiFiClientSecure     secured_client;
UniversalTelegramBot bot(BOT_TOKEN_DEFAULT, secured_client);

#define MAX_WIFI_NETWORKS 5
struct WifiCred {
    String ssid;
    String pass;
};
Preferences wifiPrefs;
WifiCred    savedWifiList[MAX_WIFI_NETWORKS];
int         savedWifiCount = 0;

String      currentSsid = WIFI_SSID_DEFAULT;
String      currentPass = WIFI_PASS_DEFAULT;

// Biến bổ sung phục vụ cơ chế tự động kết nối lại chủ động
unsigned long wifiLostTimestamp = 0;


portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool          slaveM1Online         = false;
volatile bool          slaveM2Online         = false;
volatile bool          slaveHOnline          = false;
volatile uint8_t       slaveM1Float          = FLOAT_UNKNOWN;
volatile uint8_t       slaveM2Float          = FLOAT_UNKNOWN;
volatile unsigned long slaveM1LastSeen        = 0;
volatile unsigned long slaveM2LastSeen        = 0;
volatile unsigned long slaveHLastSeen         = 0;

volatile uint8_t       rawSlaveM1Float        = FLOAT_UNKNOWN;
volatile unsigned long slaveM1FloatChangeTime = 0;
volatile uint8_t       rawSlaveM2Float        = FLOAT_UNKNOWN;
volatile unsigned long slaveM2FloatChangeTime = 0;

volatile bool          needResyncM1 = false;
volatile bool          needResyncM2 = false;

bool                   floatUnknownAlerted = false;

volatile bool          m1AutoRunning         = false;
volatile bool          m2AutoRunning         = false;

volatile int           lastRssiM1 = -120;
volatile int           lastRssiM2 = -120;
volatile int           lastRssiH  = -120;
volatile unsigned long bootTime   = 0;

volatile bool loraHardwareOk       = false;

volatile unsigned long pumpM1RuntimeTodaySec = 0;
volatile unsigned long pumpM2RuntimeTodaySec = 0;
volatile unsigned long pumpHRuntimeTodaySec  = 0;
volatile unsigned long pumpM1OnSinceMs       = 0;
volatile unsigned long pumpM2OnSinceMs       = 0;
volatile unsigned long pumpHOnSinceMs        = 0;
int           trackedDayMarker      = -1;

Preferences runtimePrefs;
static void runtimeSave();   // lưu giờ chạy hôm nay vào NVS (định nghĩa bên dưới), gọi trước khi khởi động lại

volatile uint16_t      txSeq       = 0;
volatile uint8_t       controlMode = MODE_AUTO;

volatile bool          pumpM1  = false;
volatile bool          pumpM2  = false;
volatile bool          pumpH   = false;
volatile uint8_t       speedH  = 0;

bool          lastPumpM1         = false;
bool          lastPumpM2         = false;
bool          lastPumpH          = false;
uint8_t       lastSpeedH         = 255;
uint8_t       lastNotifiedSpeedH = 0;

volatile bool          manualPumpM1 = false;
volatile bool          manualPumpM2 = false;
volatile bool          manualPumpH  = false;

#define RELAY_ON_GAP_MS            1500UL
#define RELAY_OFF_GAP_MS            600UL
#define RELAY_MIN_OFF_MS           3000UL
#define RELAY_LORA_TX_CLEAR_MS      700UL
#define RELAY_QUIET_MS              400UL
volatile unsigned long relayLastSwitchMs = 0;
volatile unsigned long lastLoraTxMs      = 0;
static inline bool relayQuiet() { return (millis() - relayLastSwitchMs) < RELAY_QUIET_MS; }


volatile uint8_t       telegramSpeed = 100;

uint8_t       masterFloat     = FLOAT_FULL;
uint8_t       rawMasterFloat  = FLOAT_FULL;
unsigned long floatChangeTime = 0;

uint8_t       lastMasterFloatSent = FLOAT_FULL;

unsigned long lastPumpHCmdTime     = 0;

int           waterLevelPercent   = -1;
float         waterLevelDistanceCm = -1;
bool          waterSensorOk       = false;
unsigned long lastWaterReadMs     = 0;
unsigned long lastWaterAlertMs    = 0;
bool          waterLowAlertSent   = false;
bool          waterHighAlertSent  = false;

Preferences   waterCfgPrefs;
int           waterTankHeightCm  = WATER_TANK_HEIGHT_CM_DEFAULT;
int           waterFullOffsetCm  = WATER_FULL_OFFSET_CM_DEFAULT;
bool          waterSensorEnabled = WATER_SENSOR_ENABLED_DEFAULT;

int           waterPumpBlockLowPercent    = WATER_PUMP_BLOCK_LOW_PERCENT_DEFAULT;
int           waterPumpBlockHighPercent   = WATER_PUMP_BLOCK_HIGH_PERCENT_DEFAULT;
int           waterPumpHighRecoverPercent = WATER_PUMP_HIGH_RECOVER_PERCENT_DEFAULT;
bool          waterPumpHBlockedByFull     = false;

unsigned long lastTelegramPoll     = 0;

String        cachedStatusShort = "";
String        cachedStatusFull  = "";
unsigned long lastStatusBuild   = 0;


volatile uint32_t loraRxOk      = 0;
volatile uint32_t loraRxBadSize = 0;


unsigned long lastAutoRebootCheck = 0;
String bootResetReason = "";

struct AckItem { Packet pkt; } ackQueue[QUEUE_SIZE];
volatile uint8_t ackHead = 0, ackTail = 0;
unsigned long    lastAckSend = 0;

struct CmdItem { Packet pkt; } cmdQueue[QUEUE_SIZE];
volatile uint8_t cmdHead = 0, cmdTail = 0;
unsigned long    lastCmdSend = 0;

std::queue<String> telegramQueue;
const size_t       MAX_TELEGRAM_QUEUE_SIZE = 8;

SemaphoreHandle_t loraMutex          = NULL;
SemaphoreHandle_t telegramQueueMutex = NULL;
SemaphoreHandle_t coreSyncMutex      = NULL;
SemaphoreHandle_t telegramClientMutex = NULL;
TaskHandle_t      loraTaskHandle     = NULL;
TaskHandle_t      telegramTaskHandle = NULL;

void releaseTelegramClient() {
    if (telegramClientMutex != NULL && xSemaphoreTake(telegramClientMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        secured_client.stop();
        xSemaphoreGive(telegramClientMutex);
    }
}

volatile bool otaRunning = false;

TaskHandle_t      otaLoraTaskHandle       = NULL;
volatile bool     otaLoraActive           = false;
volatile bool     otaLoraCancelRequested  = false;
volatile uint16_t otaLoraChunkIndex       = 0;
volatile uint16_t otaLoraTotalChunks      = 0;
uint8_t            otaLoraTargetId        = 0;
String             otaLoraTargetKey       = "";
String             otaLoraVersion         = "";

volatile bool     otaAckReceived = false;
volatile uint16_t otaAckSeq      = 0;
volatile uint8_t  otaAckStatus   = 0;

String divider() { return String(UI_DIVIDER) + "\n"; }

String floatText(uint8_t state) {
    if (state == FLOAT_UNKNOWN) return String(ICON_WARN) + " Chưa xác định";
    return (state == FLOAT_LOW) ? String(ICON_BAD) + " Cạn" : String(ICON_OK) + " Đầy";
}

String statusBadge(bool online) {
    return online ? String(ICON_OK) + " ONLINE" : String(ICON_BAD) + " OFFLINE";
}

String pumpBadge(bool on) {
    return on ? String(ICON_OK) + " BẬT" : String(ICON_BAD) + " TẮT";
}

String signalBadge(int rssi) {
    String bars;
    if      (rssi >= -70)  bars = "▂▄▆█";
    else if (rssi >= -90)  bars = "▂▄▆▁";
    else if (rssi >= -110) bars = "▂▄▁▁";
    else                    bars = "▂▁▁▁";
    return bars + " " + String(rssi) + " dBm";
}

String alertBlock(const char *icon, const String &title, const String &detail = "") {
    String m = String(icon) + " *" + title + "*";
    if (detail.length() > 0) m += "\n" + detail;
    return m;
}

String escapeMarkdown(const String &raw) {
    String out;
    out.reserve(raw.length() + 8);
    for (unsigned int i = 0; i < raw.length(); i++) {
        char c = raw.charAt(i);
        if (c == '_' || c == '*' || c == '`' || c == '[') out += '\\';
        out += c;
    }
    return out;
}

#define MAX_USERS   20
#define ROLE_USER   0
#define ROLE_ADMIN  1

Preferences usersPrefs;
String      userIds[MAX_USERS];
uint8_t     userRoles[MAX_USERS];
int         userCount = 0;

int findUserIndex(const String &chatId) {
    for (int i = 0; i < userCount; i++) {
        if (userIds[i] == chatId) return i;
    }
    return -1;
}

bool isRegisteredUser(const String &chatId) { return findUserIndex(chatId) >= 0; }

bool isAdminUser(const String &chatId) {
    int idx = findUserIndex(chatId);
    return (idx >= 0) && (userRoles[idx] == ROLE_ADMIN);
}

void saveUsersToNVS() {
    usersPrefs.begin("users", false);
    usersPrefs.putInt("count", userCount);
    for (int i = 0; i < userCount; i++) {
        usersPrefs.putString(("id" + String(i)).c_str(), userIds[i]);
        usersPrefs.putUChar(("role" + String(i)).c_str(), userRoles[i]);
    }
    usersPrefs.end();
}

void loadUsersFromNVS() {
    usersPrefs.begin("users", true);
    userCount = usersPrefs.getInt("count", 0);
    if (userCount < 0) userCount = 0;
    if (userCount > MAX_USERS) userCount = MAX_USERS;
    for (int i = 0; i < userCount; i++) {
        userIds[i]   = usersPrefs.getString(("id" + String(i)).c_str(), "");
        userRoles[i] = usersPrefs.getUChar(("role" + String(i)).c_str(), ROLE_USER);
    }
    usersPrefs.end();

    if (userCount == 0) {
        userIds[0]   = CHAT_ID_DEFAULT;
        userRoles[0] = ROLE_ADMIN;
        userCount    = 1;
        saveUsersToNVS();
    }
}

bool addUserToList(const String &chatId, uint8_t role) {
    if (chatId.length() == 0) return false;
    if (findUserIndex(chatId) >= 0) return false;
    if (userCount >= MAX_USERS) return false;
    userIds[userCount]   = chatId;
    userRoles[userCount] = role;
    userCount++;
    saveUsersToNVS();
    return true;
}

bool removeUserFromList(const String &chatId) {
    int idx = findUserIndex(chatId);
    if (idx < 0) return false;
    for (int i = idx; i < userCount - 1; i++) {
        userIds[i]   = userIds[i + 1];
        userRoles[i] = userRoles[i + 1];
    }
    userCount--;
    saveUsersToNVS();
    return true;
}

String buildUsersListMessage() {
    String m = "👥 *DANH SÁCH NGƯỜI DÙNG*\n" + divider();
    m += "🔑 *ADMIN*\n";
    bool any = false;
    for (int i = 0; i < userCount; i++) {
        if (userRoles[i] == ROLE_ADMIN) { m += "├─ `" + userIds[i] + "`\n"; any = true; }
    }
    if (!any) m += "├─ (không có)\n";
    m += "\n👤 *USER*\n";
    any = false;
    for (int i = 0; i < userCount; i++) {
        if (userRoles[i] == ROLE_USER) { m += "├─ `" + userIds[i] + "`\n"; any = true; }
    }
    if (!any) m += "├─ (không có)\n";
    m += "\n📊 Tổng: " + String(userCount) + "/" + String(MAX_USERS);
    return m;
}

String formatTimeSpan(unsigned long ms) {
    unsigned long s = ms / 1000;
    unsigned long m = s / 60; s %= 60;
    unsigned long h = m / 60; m %= 60;
    char buf[20];
    snprintf(buf, sizeof(buf), "%luh %02lum %02lus", h, m, s);
    return String(buf);
}

void logAlert(const String &msg) {
    if (alertLogMutex == NULL) return;
    if (xSemaphoreTake(alertLogMutex, pdMS_TO_TICKS(25)) != pdTRUE) return;
    alertLog[alertLogIdx] = msg;
    alertLogIdx = (alertLogIdx + 1) % ALERT_LOG_SIZE;
    xSemaphoreGive(alertLogMutex);
}

void sendTelegramAlert(const String &msg) {
    Serial.println("[TELEGRAM QUEUED] " + msg);
    logAlert(msg);
    if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
        if (telegramQueue.size() < MAX_TELEGRAM_QUEUE_SIZE) {
            telegramQueue.push(msg);
        }
        xSemaphoreGive(telegramQueueMutex);
    }
}

#define TELEGRAM_CLIENT_MUTEX_WAIT_MS 15000UL
#define TELEGRAM_MAX_CHUNK_BYTES 2800

static bool telegramSendChunked(const String &chatId, const String &msg, const String &mode) {
    int len = msg.length();
    if (len <= TELEGRAM_MAX_CHUNK_BYTES) {
        bool ok = bot.sendMessage(chatId, msg, mode);
        if (!ok && mode.length() > 0) ok = bot.sendMessage(chatId, msg, "");
        return ok;
    }
    bool allOk = true;
    int pos = 0;
    while (pos < len) {
        int end = pos + TELEGRAM_MAX_CHUNK_BYTES;
        if (end >= len) end = len;
        else {
            int nl = msg.lastIndexOf('\n', end);
            if (nl > pos) end = nl + 1;
            else { while (end > pos && (((uint8_t)msg[end]) & 0xC0) == 0x80) end--; }
            if (end <= pos) end = (pos + TELEGRAM_MAX_CHUNK_BYTES < len) ? pos + TELEGRAM_MAX_CHUNK_BYTES : len;
        }
        String part = msg.substring(pos, end);
        if (part.length() > 0) {
            bool ok = bot.sendMessage(chatId, part, mode);
            if (!ok && mode.length() > 0) ok = bot.sendMessage(chatId, part, "");
            if (!ok) allOk = false;
        }
        pos = end;
    }
    return allOk;
}

bool telegramSendMessageSafe(const String &chatId, const String &msg, const String &mode) {
    if (telegramClientMutex == NULL) return telegramSendChunked(chatId, msg, mode);
    if (xSemaphoreTake(telegramClientMutex, pdMS_TO_TICKS(TELEGRAM_CLIENT_MUTEX_WAIT_MS)) != pdTRUE) {
        return false;
    }
    bool ok = telegramSendChunked(chatId, msg, mode);
    xSemaphoreGive(telegramClientMutex);
    if (ok) tgLastOkMs = millis();
    return ok;
}

int telegramGetUpdatesSafe(int32_t offset) {
    if (telegramClientMutex == NULL) return bot.getUpdates(offset);
    if (xSemaphoreTake(telegramClientMutex, pdMS_TO_TICKS(TELEGRAM_CLIENT_MUTEX_WAIT_MS)) != pdTRUE) {
        return 0;
    }
    int n = bot.getUpdates(offset);
    xSemaphoreGive(telegramClientMutex);
    if (n > 0) tgLastOkMs = millis();
    return n;
}

// Thăm dò Telegram (getMe): getUpdates trả 0 cả khi lỗi lẫn khi không có tin nên không dùng để biết còn sống hay không
static bool telegramProbeSafe() {
    if (telegramClientMutex == NULL || xSemaphoreTake(telegramClientMutex, pdMS_TO_TICKS(TELEGRAM_CLIENT_MUTEX_WAIT_MS)) != pdTRUE) return false;
    bool ok = bot.getMe();
    xSemaphoreGive(telegramClientMutex);
    if (ok) tgLastOkMs = millis();
    return ok;
}

void sendTelegramDirect(const String &msg) {
    logAlert(msg);
    bool md = (msg.indexOf('*') != -1);
    if (userCount == 0) {
        telegramSendMessageSafe(currentChatId, msg, md ? "Markdown" : "");
        return;
    }
    for (int i = 0; i < userCount; i++) {
        telegramSendMessageSafe(userIds[i], msg, md ? "Markdown" : "");
    }
}

void sendTelegramReplyTo(const String &chatId, const String &msg) {
    logAlert(msg);
    bool md = (msg.indexOf('*') != -1);
    telegramSendMessageSafe(chatId, msg, md ? "Markdown" : "");
}

#define OTA_CONFIRM_TIMEOUT_MS 300000UL
String        otaPendingVersion   = "";
String        otaPendingFile      = "";
String        otaPendingTarget    = "master";
unsigned long otaPendingExpireAt  = 0;

// OTA CHỈ chạy khi có lệnh Telegram (/update, /update_to...). Không có tự động theo giờ.


struct OtaEntry {
    String version;
    String file;
    String note;
    String target;
};
OtaEntry      otaList[OTA_MAX_VERSIONS];
int           otaListCount = 0;

struct OtaBusyGuard {
    bool prev;
    OtaBusyGuard()  { prev = otaRunning; otaRunning = true; }
    ~OtaBusyGuard() { otaRunning = prev; }
};

bool fetchOtaManifest() {
    OtaBusyGuard busyGuard;
    releaseTelegramClient();
    if (WiFi.status() != WL_CONNECTED) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Chưa kết nối WiFi."));
        return false;
    }

    WiFiClientSecure verClient;
    verClient.setInsecure();
    verClient.setTimeout(15000);

    HTTPClient http;
    http.setTimeout(18000);

    if (!http.begin(verClient, OTA_MANIFEST_URL)) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Không kết nối được GitHub."));
        return false;
    }

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Lỗi HTTP " + String(code) + " khi tải versions.json"));
        http.end();
        return false;
    }

    String body = http.getString();
    http.end();
    body.trim();

    if (body.length() == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "versions.json rỗng hoặc không đọc được."));
        return false;
    }

    DynamicJsonDocument doc(4096);
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "versions.json sai định dạng JSON: " + escapeMarkdown(String(err.c_str()))));
        return false;
    }

    JsonArray arr = doc["versions"].as<JsonArray>();
    if (arr.isNull() || arr.size() == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "versions.json không có mục \"versions\" hợp lệ."));
        return false;
    }

    otaListCount = 0;
    for (JsonObject item : arr) {
        if (otaListCount >= OTA_MAX_VERSIONS) break;
        const char* v = item["version"] | "";
        const char* f = item["file"]    | "";
        const char* n = item["note"]    | "";
        const char* t = item["target"] | "master";
        if (strlen(v) == 0 || strlen(f) == 0) continue;
        otaList[otaListCount].version = String(v);
        otaList[otaListCount].file    = String(f);
        otaList[otaListCount].note    = String(n);
        otaList[otaListCount].target  = String(t);
        otaListCount++;
    }

    if (otaListCount == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "versions.json không có mục nào đủ \"version\" + \"file\"."));
        return false;
    }

    return true;
}

int findLatestOtaIndexForTarget(const String& target) {
    for (int i = 0; i < otaListCount; i++) {
        if (otaList[i].target == target) return i;
    }
    return -1;
}

int filterOtaByTarget(const String& target, int* outIdx, int maxOut) {
    int n = 0;
    for (int i = 0; i < otaListCount && n < maxOut; i++) {
        if (otaList[i].target == target) outIdx[n++] = i;
    }
    return n;
}

String targetDisplayName(const String& target) {
    if (target == "m1") return "Slave Moong 1";
    if (target == "m2") return "Slave Moong 2";
    if (target == "h")  return "Slave Tram Ho";
    return "Master";
}

void setOtaPending(const String& version, const String& file, const String& target) {
    otaPendingVersion  = version;
    otaPendingFile     = file;
    otaPendingTarget   = target;
    otaPendingExpireAt = millis() + OTA_CONFIRM_TIMEOUT_MS;

    if (target == "master") {
        sendTelegramDirect(alertBlock(ICON_WARN, "⬇️ Tự động cập nhật (không cần xác nhận)",
            "Hiện tại: " + escapeMarkdown(String(FW_VERSION)) + " → Mới: " + escapeMarkdown(version) +
            " (" + escapeMarkdown(file) + ")\nĐang tải & flash ngay..."));
        performOTAUpdate();
        return;
    }

    sendTelegramDirect(alertBlock(ICON_WARN, "Đã chọn bản để cập nhật [" + targetDisplayName(target) + "] — cần xác nhận",
        "Chọn: " + escapeMarkdown(version) + " (" + escapeMarkdown(file) + ")"
        "\n\n⚠️ Cập nhật QUA LORA cho slave có thể mất TỪ VÀI CHỤC PHÚT ĐẾN VÀI GIỜ.\nGõ /update_confirm để bắt đầu."
        "\nGõ /update_cancel để huỷ."
        "\n⏳ Yêu cầu này hết hạn sau 5 phút nếu không xác nhận."));
}

void checkOtaUpdateForTarget(const String& target) {
    if (!fetchOtaManifest()) return;

    int idxArr[OTA_MAX_VERSIONS];
    int n = filterOtaByTarget(target, idxArr, OTA_MAX_VERSIONS);
    if (n == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Không tìm thấy bản nào cho " + targetDisplayName(target),
            "versions.json chưa có mục nào với target=\"" + target + "\"."));
        return;
    }

    int latest = idxArr[0];
    if (target == "master" && otaList[latest].version == FW_VERSION) {
        otaPendingVersion = ""; otaPendingFile = ""; otaPendingTarget = "master";
        sendTelegramDirect(alertBlock(ICON_OK, "Đã là bản mới nhất", "Bản hiện tại: " + escapeMarkdown(String(FW_VERSION))));
        return;
    }

    setOtaPending(otaList[latest].version, otaList[latest].file, target);
}

void listOtaVersionsForTarget(const String& target) {
    if (!fetchOtaManifest()) return;

    int idxArr[OTA_MAX_VERSIONS];
    int n = filterOtaByTarget(target, idxArr, OTA_MAX_VERSIONS);
    if (n == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Không tìm thấy bản nào cho " + targetDisplayName(target),
            "versions.json chưa có mục nào với target=\"" + target + "\"."));
        return;
    }

    String msg = "";
    if (target == "master") msg += "Bản hiện tại: " + escapeMarkdown(String(FW_VERSION)) + "\n";
    for (int k = 0; k < n; k++) {
        int i = idxArr[k];
        msg += "\n" + String(k + 1) + ". " + escapeMarkdown(otaList[i].version);
        if (target == "master" && otaList[i].version == FW_VERSION) msg += " (đang chạy)";
        msg += "\n    file: " + escapeMarkdown(otaList[i].file);
        if (otaList[i].note.length() > 0) msg += "\n    " + escapeMarkdown(otaList[i].note);
    }
    String cmdHint = (target == "master") ? "/update_to" : ("/lora_update_to " + target);
    msg += "\n\nGõ " + cmdHint + " <số thứ tự> hoặc <version> để chọn bản cần cài.";

    sendTelegramDirect(alertBlock(ICON_INFO, "Danh sách firmware [" + targetDisplayName(target) + "] trên GitHub", msg));
}

void selectOtaVersionForTarget(const String& target, String arg) {
    arg.trim();
    if (arg.length() == 0) {
        sendTelegramDirect(alertBlock(ICON_WARN, "Thiếu tham số",
            "Dùng: <lệnh> <số thứ tự>  hoặc  <lệnh> <version>"));
        return;
    }

    if (!fetchOtaManifest()) return;

    int idxArr[OTA_MAX_VERSIONS];
    int n = filterOtaByTarget(target, idxArr, OTA_MAX_VERSIONS);
    if (n == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Không tìm thấy bản nào cho " + targetDisplayName(target)));
        return;
    }

    bool isNumeric = true;
    for (unsigned int i = 0; i < arg.length(); i++) {
        if (!isDigit(arg.charAt(i))) { isNumeric = false; break; }
    }

    int foundIdx = -1;
    if (isNumeric) {
        int pos = arg.toInt();
        if (pos >= 1 && pos <= n) foundIdx = idxArr[pos - 1];
    }
    if (foundIdx == -1) {
        for (int k = 0; k < n; k++) {
            if (otaList[idxArr[k]].version == arg) { foundIdx = idxArr[k]; break; }
        }
    }

    if (foundIdx == -1) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Không tìm thấy bản này",
            "\"" + escapeMarkdown(arg) + "\" không khớp số thứ tự hay version nào cho " + targetDisplayName(target) + "."));
        return;
    }

    setOtaPending(otaList[foundIdx].version, otaList[foundIdx].file, target);
}

void checkOTAUpdate()             { checkOtaUpdateForTarget("master"); }
void listOtaVersions()            { listOtaVersionsForTarget("master"); }
void selectOtaVersion(String arg) { selectOtaVersionForTarget("master", arg); }

void performOTAUpdate() {
    portENTER_CRITICAL(&stateMux);
    const bool allPumpsOff = (!pumpM1 && !pumpM2 && !pumpH);
    portEXIT_CRITICAL(&stateMux);
    if (!allPumpsOff) {
        otaPendingVersion = "";
        otaPendingFile = "";
        otaPendingTarget = "master";
        otaPendingExpireAt = 0;
        sendTelegramDirect(alertBlock(ICON_WARN, "Đã chặn OTA Master để bảo vệ bơm",
            "Hiện còn ít nhất một bơm đang chạy. Hãy dừng tất cả bơm, sau đó gửi lại /update."));
        return;
    }

    String remoteVersion = otaPendingVersion;
    String remoteFile    = otaPendingFile;
    otaPendingVersion = ""; otaPendingFile = "";

    otaRunning = true;

    String firmwareUrl = String(OTA_RAW_BASE_URL) + remoteFile;

    sendTelegramDirect(alertBlock(ICON_WARN, "⬇️ Đang cập nhật firmware mới...",
        "Hiện tại: " + escapeMarkdown(String(FW_VERSION)) + " → Mới: " + escapeMarkdown(remoteVersion) + "\nFile: " + escapeMarkdown(remoteFile)));

    delay(1000);

    releaseTelegramClient();
    WiFiClientSecure fwClient;
    fwClient.setInsecure();
    fwClient.setTimeout(15000);
    fwClient.setHandshakeTimeout(10);

    httpUpdate.rebootOnUpdate(false);

    t_httpUpdate_return ret = httpUpdate.update(fwClient, firmwareUrl);

    if (ret == HTTP_UPDATE_OK) {
        sendTelegramDirect(alertBlock(ICON_SUCCESS, "✅ OTA thành công",
            "Đã cập nhật lên bản: " + escapeMarkdown(remoteVersion) + "\nThiết bị sẽ khởi động lại ngay bây giờ..."));
        delay(1000);
        telegramGetUpdatesSafe(bot.last_message_received + 1);
        selfHealNoteCause("Cập nhật firmware OTA xong");
        runtimeSave(); ESP.restart();
    } else {
        otaRunning = false;

        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại",
            "Lỗi #" + String(httpUpdate.getLastError()) + ": " + escapeMarkdown(httpUpdate.getLastErrorString())));
    }
}

bool isValidLoraTarget(const String& t) { return (t == "m1" || t == "m2" || t == "h"); }

uint8_t loraTargetToId(const String& t) {
    if (t == "m1") return SLAVE_M1_ID;
    if (t == "m2") return SLAVE_M2_ID;
    if (t == "h")  return SLAVE_H_ID;
    return 0xFF;
}

bool downloadToLittleFS(const String& url, const char* path, uint32_t& outSize) {
    if (WiFi.status() != WL_CONNECTED) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Chưa kết nối WiFi."));
        return false;
    }

    OtaBusyGuard busyGuard;
    releaseTelegramClient();
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(15000);

    HTTPClient http;
    http.setTimeout(20000);
    if (!http.begin(client, url)) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Không kết nối được GitHub."));
        return false;
    }

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Lỗi HTTP " + String(code) + " khi tải file firmware."));
        http.end();
        return false;
    }

    int contentLen = http.getSize();
    if (contentLen <= 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Không xác định được dung lượng file."));
        http.end();
        return false;
    }

    File f = LittleFS.open(path, "w");
    if (!f) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Không tạo được file tạm trên LittleFS."));
        http.end();
        return false;
    }

    WiFiClient* stream = http.getStreamPtr();
    uint8_t buf[512];
    int remaining = contentLen;
    unsigned long lastData = millis();

    while (remaining > 0 && (millis() - lastData) < 20000UL) {
        size_t avail = stream->available();
        if (avail > 0) {
            size_t want = avail;
            if (want > sizeof(buf)) want = sizeof(buf);
            if (want > (size_t)remaining) want = (size_t)remaining;
            int n = stream->readBytes(buf, want);
            if (n > 0) {
                f.write(buf, n);
                remaining -= n;
                lastData = millis();
            }
        } else {
            delay(5);
        }
    }
    f.close();
    http.end();

    if (remaining > 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại",
            "Tải file bị ngắt giữa chừng (thiếu " + String(remaining) + " / " + String(contentLen) + " byte)."));
        LittleFS.remove(path);
        return false;
    }

    outSize = (uint32_t)contentLen;
    return true;
}

bool sendOtaBigAndWaitAck(uint8_t targetId, uint8_t msgType, uint16_t chunkIndex, uint16_t chunkLen,
                           uint32_t totalSize, uint32_t crc32Val, const uint8_t *data,
                           uint16_t expectedAckSeq, uint8_t maxRetries, unsigned long ackTimeoutMs) {
    OtaPacket opkt;
    memset(&opkt, 0, sizeof(opkt));
    opkt.sender     = MASTER_ID;
    opkt.receiver   = targetId;
    opkt.msgType    = msgType;
    opkt.chunkIndex = chunkIndex;
    opkt.chunkLen   = chunkLen;
    opkt.totalSize  = totalSize;
    opkt.crc32      = crc32Val;
    if (data != NULL && chunkLen > 0) memcpy(opkt.data, data, chunkLen);

    for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
        portENTER_CRITICAL(&stateMux);
        otaAckReceived = false;
        otaAckSeq = 0;
        otaAckStatus = 0;
        portEXIT_CRITICAL(&stateMux);

        bool sent = false;
        if (loraMutex != NULL && xSemaphoreTake(loraMutex, pdMS_TO_TICKS(LORA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            bool started = LoRa.beginPacket();
            size_t written = started ? LoRa.write((uint8_t*)&opkt, sizeof(OtaPacket)) : 0;
            sent = (written == sizeof(OtaPacket)) && LoRa.endPacket(true);
            xSemaphoreGive(loraMutex);
        }
        if (!sent) { delay(20); continue; }

        unsigned long waitStart = millis();
        while (millis() - waitStart < ackTimeoutMs) {
            bool got; uint16_t seq; uint8_t status;
            portENTER_CRITICAL(&stateMux);
            got = otaAckReceived; seq = otaAckSeq; status = otaAckStatus;
            portEXIT_CRITICAL(&stateMux);
            if (got && seq == expectedAckSeq) return (status == 1);
            delay(20);
        }
    }
    return false;
}

bool sendOtaSmallAndWaitAck(uint8_t targetId, uint8_t msgType, uint16_t seqVal,
                             uint16_t expectedAckSeq, uint8_t maxRetries, unsigned long ackTimeoutMs) {
    Packet pkt = { MASTER_ID, targetId, msgType, seqVal, 0, (uint32_t)millis() };

    for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
        portENTER_CRITICAL(&stateMux);
        otaAckReceived = false;
        otaAckSeq = 0;
        otaAckStatus = 0;
        portEXIT_CRITICAL(&stateMux);

        bool sent = false;
        if (loraMutex != NULL && xSemaphoreTake(loraMutex, pdMS_TO_TICKS(LORA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            bool started = LoRa.beginPacket();
            size_t written = started ? LoRa.write((uint8_t*)&pkt, sizeof(Packet)) : 0;
            sent = (written == sizeof(Packet)) && LoRa.endPacket(true);
            xSemaphoreGive(loraMutex);
        }
        if (!sent) { delay(20); continue; }

        unsigned long waitStart = millis();
        while (millis() - waitStart < ackTimeoutMs) {
            bool got; uint16_t seq; uint8_t status;
            portENTER_CRITICAL(&stateMux);
            got = otaAckReceived; seq = otaAckSeq; status = otaAckStatus;
            portEXIT_CRITICAL(&stateMux);
            if (got && seq == expectedAckSeq) return (status == 1);
            delay(20);
        }
    }
    return false;
}

void vLoraOtaRelayTask(void *pvParameters) {
    File f = LittleFS.open("/lora_ota.bin", "r");
    if (!f) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Không đọc lại được file firmware tạm."));
    } else {
        uint32_t totalSize   = f.size();
        uint16_t totalChunks = (uint16_t)((totalSize + OTA_CHUNK_SIZE - 1) / OTA_CHUNK_SIZE);

        uint32_t crc = 0xFFFFFFFF;
        {
            uint8_t cbuf[256];
            while (f.available()) {
                int n = f.read(cbuf, sizeof(cbuf));
                if (n <= 0) break;
                crc = crc32Compute(crc, cbuf, n);
            }
            crc ^= 0xFFFFFFFF;
            f.seek(0);
        }

        otaLoraTotalChunks = totalChunks;
        otaLoraChunkIndex  = 0;

        sendTelegramDirect(alertBlock(ICON_WARN, "Bắt đầu cập nhật LoRa: " + targetDisplayName(otaLoraTargetKey),
            "Bản: " + escapeMarkdown(otaLoraVersion) + "\nDung lượng: " + String(totalSize) + " byte\nSố chunk: " + String(totalChunks) +
            "\n\n⚠️ Quá trình này có thể mất TỪ VÀI CHỤC PHÚT ĐẾN VÀI GIỜ."
            "\nGõ /lora_update_status để xem tiến độ, /lora_update_abort để huỷ."));

        bool started = sendOtaBigAndWaitAck(otaLoraTargetId, MSG_OTA_START, totalChunks, OTA_CHUNK_SIZE,
                                             totalSize, crc, NULL, 0, 5, 3000);
        if (!started) {
            sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại",
                targetDisplayName(otaLoraTargetKey) + " không phản hồi START."));
        } else {
            bool allOk = true;
            for (uint16_t i = 0; i < totalChunks; i++) {
                if (otaLoraCancelRequested) {
                    sendOtaSmallAndWaitAck(otaLoraTargetId, MSG_OTA_ABORT, 0, 0xFFFE, 2, 1000);
                    sendTelegramDirect(alertBlock(ICON_WARN, "Đã huỷ cập nhật LoRa", targetDisplayName(otaLoraTargetKey)));
                    allOk = false;
                    break;
                }

                uint8_t buf[OTA_CHUNK_SIZE];
                int n = f.read(buf, OTA_CHUNK_SIZE);
                if (n <= 0) { allOk = false; break; }

                uint32_t chunkCrc = crc32Compute(0xFFFFFFFF, buf, n) ^ 0xFFFFFFFF;
                bool chunkOk = sendOtaBigAndWaitAck(otaLoraTargetId, MSG_OTA_DATA, i, (uint16_t)n,
                                                     0, chunkCrc, buf, i, 5, 4000);
                if (!chunkOk) {
                    sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại",
                        targetDisplayName(otaLoraTargetKey) + " không xác nhận chunk " + String(i)));
                    allOk = false;
                    break;
                }

                otaLoraChunkIndex = i + 1;
                if (totalChunks > 0 && ((i % 100 == 0) || (i == (uint16_t)(totalChunks - 1)))) {
                    int pct = (int)((long)(i + 1) * 100L / totalChunks);
                    sendTelegramDirect(alertBlock(ICON_INFO, "Tiến độ " + targetDisplayName(otaLoraTargetKey),
                        String(i + 1) + "/" + String(totalChunks) + " chunk (" + String(pct) + "%)"));
                }
            }

            if (allOk) {
                bool ended = sendOtaSmallAndWaitAck(otaLoraTargetId, MSG_OTA_END, totalChunks, 0xFFFF, 5, 5000);
                if (ended) {
                    sendTelegramDirect(alertBlock(ICON_SUCCESS, "Cập nhật LoRa THÀNH CÔNG",
                        targetDisplayName(otaLoraTargetKey) + " đã nhận đủ firmware và đang tự khởi động lại."));
                } else {
                    sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại ở bước cuối",
                        targetDisplayName(otaLoraTargetKey) + " không xác nhận kết thúc."));
                }
            }
        }
        f.close();
    }

    LittleFS.remove("/lora_ota.bin");
    otaLoraActive          = false;
    otaLoraCancelRequested = false;
    vTaskDelete(NULL);
}

void performLoraOtaUpdate() {
    String remoteVersion = otaPendingVersion;
    String remoteFile    = otaPendingFile;
    String target         = otaPendingTarget;
    otaPendingVersion = ""; otaPendingFile = ""; otaPendingTarget = "master";

    if (otaLoraActive) {
        sendTelegramDirect(alertBlock(ICON_WARN, "Đang có 1 phiên cập nhật LoRa khác chạy",
            "Gõ /lora_update_status để xem."));
        return;
    }
    if (!isValidLoraTarget(target)) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Target không hợp lệ: " + target));
        return;
    }

    String firmwareUrl = String(OTA_RAW_BASE_URL) + remoteFile;

    sendTelegramDirect(alertBlock(ICON_WARN, "⬇️ Đang tải firmware cho " + targetDisplayName(target) + "...",
        "Bản: " + escapeMarkdown(remoteVersion) + "\nFile: " + escapeMarkdown(remoteFile)));

    if (!LittleFS.begin(true)) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Không mở được bộ nhớ tạm (LittleFS)."));
        return;
    }
    LittleFS.remove("/lora_ota.bin");

    uint32_t downloadedSize = 0;
    if (!downloadToLittleFS(firmwareUrl, "/lora_ota.bin", downloadedSize)) {
        return;
    }

    otaLoraTargetKey       = target;
    otaLoraTargetId        = loraTargetToId(target);
    otaLoraVersion         = remoteVersion;
    otaLoraChunkIndex      = 0;
    otaLoraTotalChunks     = 0;
    otaLoraCancelRequested = false;
    otaLoraActive          = true;

    otaLoraTaskHandle = NULL;
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        vLoraOtaRelayTask, "LoraOtaRelay", 20480, NULL, 1, &otaLoraTaskHandle, 1);
    if (taskCreated != pdPASS) {
        otaLoraActive = false;
        otaLoraCancelRequested = false;
        otaLoraTaskHandle = NULL;
        LittleFS.remove("/lora_ota.bin");
        sendTelegramDirect(alertBlock(ICON_BAD, "Không khởi chạy được OTA LoRa", "Thiếu bộ nhớ để tạo task."));
    }
}

void showLoraOtaStatus() {
    if (!otaLoraActive) {
        sendTelegramDirect(alertBlock(ICON_INFO, "Không có phiên cập nhật LoRa nào đang chạy."));
        return;
    }
    uint16_t idx = otaLoraChunkIndex, total = otaLoraTotalChunks;
    int pct = (total > 0) ? (int)((long)idx * 100L / total) : 0;
    sendTelegramDirect(alertBlock(ICON_INFO, "Tiến độ cập nhật LoRa: " + targetDisplayName(otaLoraTargetKey),
        "Bản: " + escapeMarkdown(otaLoraVersion) + "\n" + String(idx) + "/" + String(total) + " chunk (" + String(pct) + "%)"));
}

void abortLoraOtaTransfer() {
    if (!otaLoraActive) {
        sendTelegramDirect(alertBlock(ICON_INFO, "Không có phiên cập nhật LoRa nào đang chạy để huỷ."));
        return;
    }
    otaLoraCancelRequested = true;
    sendTelegramDirect(alertBlock(ICON_WARN, "Đã gửi yêu cầu huỷ — đang chờ tác vụ dừng lại..."));
}

// Chỉ ghi "lý do" của lần ESP.restart() kế tiếp (RTC_NOINIT sống qua reset mềm) để báo lên Telegram sau khi khởi động.
struct SelfHealRtc { uint32_t magic; char cause[60]; };
static RTC_NOINIT_ATTR SelfHealRtc shRtc;
#define SELFHEAL_MAGIC 0x5E1F4EA2UL

static void shEnsure() {
    if (shRtc.magic != SELFHEAL_MAGIC) { memset(&shRtc, 0, sizeof(shRtc)); shRtc.magic = SELFHEAL_MAGIC; }
}

static String selfHealOnBoot() {
    esp_reset_reason_t r = esp_reset_reason();
    if (r == ESP_RST_POWERON || r == ESP_RST_BROWNOUT) memset(&shRtc, 0, sizeof(shRtc));
    shEnsure();
    String cause = "";
    if (r == ESP_RST_SW && shRtc.cause[0]) cause = shRtc.cause;
    shRtc.cause[0] = 0;
    return cause;
}

static void selfHealNoteCause(const char *why) { shEnsure(); strlcpy(shRtc.cause, why, sizeof(shRtc.cause)); }

String getResetReasonText() {
    esp_reset_reason_t r = esp_reset_reason();
    switch (r) {
        case ESP_RST_POWERON:   return "Cấp nguồn (Power On)";
        case ESP_RST_EXT:       return "Reset từ chân EXT/nút nhấn";
        case ESP_RST_SW:        return "Reset phần mềm (ESP.restart)";
        case ESP_RST_PANIC:     return "🚨 PANIC / Lỗi phần mềm nghiêm trọng";
        case ESP_RST_INT_WDT:   return "🚨 WATCHDOG NGẮT (Interrupt WDT)";
        case ESP_RST_TASK_WDT:  return "🚨 WATCHDOG TASK - Có task bị treo";
        case ESP_RST_WDT:       return "🚨 WATCHDOG khác";
        case ESP_RST_DEEPSLEEP: return "Thức dậy từ Deep Sleep";
        case ESP_RST_BROWNOUT:  return "🚨 SỤT ÁP NGUỒN (Brownout)";
        default:                return "Không xác định";
    }
}

void logResetReason() {
    bootResetReason = getResetReasonText();
    String shCause = selfHealOnBoot();
    if (shCause.length()) bootResetReason += " — tự khởi động lại do: " + shCause;
    Serial.println("[BOOT] Lý do khởi động: " + bootResetReason);
}

// Nhịp sống ra mạch giám sát ngoài. So sánh có dấu (int32_t) để tránh báo sai khi task khác vừa cập nhật mốc thời gian.
// Trả NULL nếu mọi thứ tốt, ngược lại trả lý do đầu tiên (chân 26 sẽ đứng yên). `bits` = mặt nạ mọi lỗi đang có, gửi cho mạch reset:
//   1 loop() đứng | 2 task Telegram kẹt | 4 Telegram im >25 phút dù WiFi nối | 8 mất WiFi >30 phút | 16 task LoRa đứng | 32 task đẩy số liệu kẹt
static const char *hbCheck(uint32_t &wifiOkMs, uint32_t &bits) {
    uint32_t now = millis();
    if (WiFi.status() == WL_CONNECTED) wifiOkMs = now;
    else tgLastOkMs = now;                                             // mất WiFi: đếm 25 phút Telegram từ lúc WiFi có lại
    if (otaRunning) { tgAliveMs = now; tgLastOkMs = now; }             // OTA chạy lâu trong task Telegram: không tính là kẹt
    bits = 0;
    const char *why = NULL;
#define HB_FAIL(cond, bit, text) if (cond) { bits |= (bit); if (!why) why = (text); }
    HB_FAIL((int32_t)(now - loopAliveMs) > (int32_t)HB_LOOP_ALIVE_MS,  1,  "loop() dung");
    HB_FAIL((int32_t)(now - loraAliveMs) > (int32_t)HB_LORA_ALIVE_MS,  16, "task LoRa dung");
    HB_FAIL((int32_t)(now - tgAliveMs)   > (int32_t)HB_TG_STUCK_MS,    2,  "task Telegram ket");
    HB_FAIL((int32_t)(now - tramAliveMs) > (int32_t)HB_TRAM_STUCK_MS,  32, "task day so lieu len tram ket");
    HB_FAIL((int32_t)(now - wifiOkMs)    > (int32_t)HB_WIFI_LOST_MS,   8,  "mat WiFi hon 30 phut");
    HB_FAIL((int32_t)(now - tgLastOkMs)  > (int32_t)HB_TG_BAD_MS,      4,  "Telegram khong phan hoi hon 25 phut");
#undef HB_FAIL
    return why;
}

// Dòng UART: "$<thân>*<XOR 2 hex>\n"  (thân = "ST,..." hoặc "BT,fw,lý do")
static void uartLine(const char *body) {
    uint8_t x = 0;
    for (const char *c = body; *c; c++) x ^= (uint8_t)*c;
    char line[240];
    int n = snprintf(line, sizeof(line), "$%s*%02X\n", body, x);
    if (n > 0 && n < (int)sizeof(line)) Serial1.write((const uint8_t *)line, (size_t)n);
}

static void uartSendStatus(uint32_t bits) {
    uint32_t now = millis();
    auto age = [&](uint32_t t) { int32_t d = (int32_t)(now - t); return (unsigned long)(d < 0 ? 0 : d); };
    unsigned long pumps  = (pumpM1 ? 1 : 0) | (pumpM2 ? 2 : 0) | (pumpH ? 4 : 0);
    unsigned long slaves = (slaveM1Online ? 1 : 0) | (slaveM2Online ? 2 : 0) | (slaveHOnline ? 4 : 0);   // bit = đang online
    bool wifi = (WiFi.status() == WL_CONNECTED);
    char body[128];
    snprintf(body, sizeof(body), "ST,%lu,%lu,%lu,%d,%d,%lu,%lu,%lu,%lu,%d,%lu",
             (unsigned long)(now / 1000UL), (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap(),
             wifi ? 1 : 0, wifi ? (int)WiFi.RSSI() : 0,
             age(tgAliveMs), age(loopAliveMs), pumps, slaves, loraHardwareOk ? 1 : 0, (unsigned long)bits);
    uartLine(body);
}

static void uartSendBoot() {
    char reason[104];
    size_t n = strlcpy(reason, bootResetReason.c_str(), sizeof(reason));
    if (n >= sizeof(reason)) n = sizeof(reason) - 1;
    for (size_t k = 0; k < n; k++) if (reason[k] == '\n' || reason[k] == '\r' || reason[k] == '*') reason[k] = ' ';
    size_t i = n;                                                       // cắt gọn, không để lại nửa ký tự UTF-8
    while (i > 0 && (((uint8_t)reason[i - 1]) & 0xC0) == 0x80) i--;
    if (i > 0) {
        uint8_t st = (uint8_t)reason[i - 1];
        size_t need = st >= 0xF0 ? 4 : st >= 0xE0 ? 3 : st >= 0xC0 ? 2 : 1;
        if (need > 1 && (n - (i - 1)) < need) reason[i - 1] = 0;
    }
    char body[160];
    snprintf(body, sizeof(body), "BT,%s,%s", FW_VERSION, reason);
    uartLine(body);
}

static void vHeartbeatOutTask(void *pvParameters) {
    uint32_t wifiOkMs = millis(), lastLog = 0, lastSt = 0;
    const uint32_t t0 = millis();
    static const uint32_t btAt[3] = {8000UL, 20000UL, 35000UL};          // dòng "vừa khởi động" gửi 3 lần để chắc chắn mạch reset nhận được
    uint8_t btSent = 0;
    const char *prev = NULL;
    for (;;) {
        uint32_t bits = 0;
        const char *bad = hbCheck(wifiOkMs, bits);
        hbReason = bad;
        if (bad != prev) { Serial.printf("[HB] %s\n", bad ? bad : "binh thuong tro lai"); prev = bad; }
        if (millis() - lastLog > 60000UL) {
            lastLog = millis();
            Serial.printf("[HB] %s | wifi=%d | Telegram OK cach day %lus | heap %lu\n", bad ? bad : "OK", (int)(WiFi.status() == WL_CONNECTED),
                          (unsigned long)((millis() - tgLastOkMs) / 1000UL), (unsigned long)ESP.getFreeHeap());
        }
        uint32_t now = millis();
        if (now - lastSt >= HB_ST_MS) { lastSt = now; uartSendStatus(bits); }
        if (btSent < 3 && now - t0 >= btAt[btSent]) { uartSendBoot(); btSent++; }

        if (!bad) {
            digitalWrite(HB_OUT_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(HB_OUT_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(900));
        } else {
            digitalWrite(HB_OUT_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void saveWifiListToNVS() {
    wifiPrefs.begin("wifi_cfg", false);
    wifiPrefs.clear();
    wifiPrefs.putInt("count", savedWifiCount);
    for (int i = 0; i < savedWifiCount; i++) {
        wifiPrefs.putString(("ssid" + String(i)).c_str(), savedWifiList[i].ssid);
        wifiPrefs.putString(("pass" + String(i)).c_str(), savedWifiList[i].pass);
    }
    wifiPrefs.end();
}

void loadWifiCredentials() {
    wifiPrefs.begin("wifi_cfg", true);
    int count = wifiPrefs.getInt("count", 0);
    savedWifiCount = 0;
    for (int i = 0; i < count && i < MAX_WIFI_NETWORKS; i++) {
        String s = wifiPrefs.getString(("ssid" + String(i)).c_str(), "");
        String p = wifiPrefs.getString(("pass" + String(i)).c_str(), "");
        if (s.length() > 0) {
            savedWifiList[savedWifiCount].ssid = s;
            savedWifiList[savedWifiCount].pass = p;
            savedWifiCount++;
        }
    }
    wifiPrefs.end();

    if (savedWifiCount == 0) {
        savedWifiList[0].ssid = WIFI_SSID_DEFAULT;
        savedWifiList[0].pass = WIFI_PASS_DEFAULT;
        savedWifiCount = 1;
        saveWifiListToNVS();
    }
    currentSsid = savedWifiList[0].ssid;
    currentPass = savedWifiList[0].pass;
}

int addOrUpdateWifiNetwork(const String &ssid, const String &pass) {
    for (int i = 0; i < savedWifiCount; i++) {
        if (savedWifiList[i].ssid == ssid) {
            savedWifiList[i].pass = pass;
            saveWifiListToNVS();
            return 2;
        }
    }
    if (savedWifiCount >= MAX_WIFI_NETWORKS) return 0;
    savedWifiList[savedWifiCount].ssid = ssid;
    savedWifiList[savedWifiCount].pass = pass;
    savedWifiCount++;
    saveWifiListToNVS();
    return 1;
}

void deleteWifiByIndex(int index) {
    int targetIndex = index - 1;
    if (targetIndex < 0 || targetIndex >= savedWifiCount) return;
    for (int i = targetIndex; i < savedWifiCount - 1; i++) {
        savedWifiList[i].ssid = savedWifiList[i + 1].ssid;
        savedWifiList[i].pass = savedWifiList[i + 1].pass;
    }
    savedWifiCount--;
    if (savedWifiCount == 0) {
        savedWifiList[0].ssid = WIFI_SSID_DEFAULT;
        savedWifiList[0].pass = WIFI_PASS_DEFAULT;
        savedWifiCount = 1;
    }
    saveWifiListToNVS();
}

bool scanAndPickBestWifi() {
    if (savedWifiCount == 0) return false;
    int n = WiFi.scanNetworks();
    if (n <= 0) { WiFi.scanDelete(); return false; }

    int bestSavedIdx = -1;
    int bestRssi     = -1000;
    for (int i = 0; i < savedWifiCount; i++) {
        for (int j = 0; j < n; j++) {
            if (WiFi.SSID(j) == savedWifiList[i].ssid) {
                int r = WiFi.RSSI(j);
                if (r > bestRssi) { bestRssi = r; bestSavedIdx = i; }
            }
        }
    }
    WiFi.scanDelete();
    if (bestSavedIdx < 0) return false;
    currentSsid = savedWifiList[bestSavedIdx].ssid;
    currentPass = savedWifiList[bestSavedIdx].pass;
    return true;
}

void loadTelegramCredentials() {
    tgPrefs.begin("tg_cfg", true);
    String t = tgPrefs.getString("token", "");
    String c = tgPrefs.getString("chatid", "");
    tgPrefs.end();
    if (t.length() > 0 && c.length() > 0) {
        currentBotToken = t;
        currentChatId   = c;
    }
}

void saveTelegramCredentialsToNVS(const String &token, const String &chatId) {
    tgPrefs.begin("tg_cfg", false);
    tgPrefs.putString("token", token);
    tgPrefs.putString("chatid", chatId);
    tgPrefs.end();
}

bool testTelegramToken(const String &token, const String &chatId) {
    WiFiClientSecure *testClient = new WiFiClientSecure();
    if (!testClient) return false;
    testClient->setInsecure();
    testClient->setTimeout(TELEGRAM_HTTP_TIMEOUT_SEC);
    UniversalTelegramBot testBot(token, *testClient);
    bool result = testBot.sendMessage(chatId, alertBlock(ICON_SUCCESS, "Xác thực Token mới thành công"), "Markdown");
    testClient->stop();
    delete testClient;
    return result;
}

String buildAlertSummary() {
    String snapshot[ALERT_LOG_SIZE];
    int snapIdx = 0;
    if (alertLogMutex != NULL && xSemaphoreTake(alertLogMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        snapIdx = alertLogIdx;
        for (int i = 0; i < ALERT_LOG_SIZE; ++i) snapshot[i] = alertLog[i];
        xSemaphoreGive(alertLogMutex);
    } else {
        return String(ICON_WARN) + " Không đọc được log.\n";
    }

    String out;
    int count = 0;
    for (int i = 0; i < ALERT_LOG_SIZE && count < 5; ++i) {
        int idx = (snapIdx - 1 - i + ALERT_LOG_SIZE) % ALERT_LOG_SIZE;
        if (snapshot[idx].length() > 0) {
            out += "├─ " + snapshot[idx] + "\n";
            ++count;
        }
    }
    if (out.length() == 0) out = ICON_OK " Không có cảnh báo gần đây.\n";
    return out;
}

String buildPendingListMessage() {
    String m = "📋 *LỆNH CHỜ XÁC NHẬN*\n" + divider();
    const unsigned long now = millis();
    PendingEntry snapshot[QUEUE_SIZE];
    portENTER_CRITICAL(&pendingMux);
    memcpy(snapshot, pendingTable, sizeof(snapshot));
    portEXIT_CRITICAL(&pendingMux);

    int count = 0;
    for (int i = 0; i < QUEUE_SIZE; ++i) {
        if (!snapshot[i].used) continue;
        String dest;
        switch (snapshot[i].pkt.receiver) {
            case SLAVE_M1_ID: dest = "Moong 1"; break;
            case SLAVE_M2_ID: dest = "Moong 2"; break;
            case SLAVE_H_ID:  dest = "Trạm Hồ"; break;
            default:          dest = "ID " + String(snapshot[i].pkt.receiver); break;
        }
        m += "├─ #" + String(snapshot[i].pkt.seq) + " → " + dest +
             " | Thử: " + String(snapshot[i].retries) +
             " | Chờ: " + String((now - snapshot[i].lastSentAt) / 1000UL) + "s\n";
        ++count;
    }
    if (count == 0) m += ICON_OK " Không có lệnh nào đang chờ.\n";
    return m;
}

String buildHelpMessage(bool isAdmin) {
    String h = "🇻🇳 *TRẠM BƠM MỎ ĐÁ — HƯỚNG DẪN*\n" + divider();
    h += "📊 *GIÁM SÁT*\n├─ `/status` — Tổng quan hệ thống\n├─ `/status_full` — Chi tiết cấu hình + Log lỗi\n├─ `/mucnuoc` — Mực nước téc (siêu âm)\n├─ `/pending` — Lệnh LoRa chưa nhận ACK\n├─ `/version` — Xem phiên bản hiện tại\n└─ `/myid` — Xem Chat ID của bạn\n\n";
    h += "⚙️ *VẬN HÀNH*\n├─ `/auto` — Chế độ TỰ ĐỘNG\n└─ `/manual` — Chế độ THỦ CÔNG\n\n";
    h += "🛠️ *THỦ CÔNG* (Gõ /manual trước)\n├─ `/bat_m1` `/tat_m1` — Bơm Moong 1\n├─ `/bat_m2` `/tat_m2` — Bơm Moong 2\n└─ `/bat_h` `/tat_h` — Bơm Trạm Hồ\n\n";
    if (!isAdmin) {
        h += "ℹ️ Một số lệnh cấu hình/OTA chỉ dành cho ADMIN.";
        return h;
    }
    h += "🔑 *QUẢN LÝ NGƯỜI DÙNG (ADMIN)*\n├─ `/adduser <chat_id>` — Thêm người dùng\n├─ `/deluser <chat_id>` — Xóa người dùng\n├─ `/users` — Danh sách người dùng\n└─ `/admin` — Xem quyền của bạn\n\n";
    h += "🚨 *KHẨN CẤP (ADMIN)*\n└─ `/reset` — Khởi động lại toàn bộ\n\n";
    h += "📐 *CẤU HÌNH TỐC ĐỘ (ADMIN)*\n└─ `/seth [0-100]` — Đặt tốc độ nền khi chạy 1 bơm (%)\n";
    h += "🚰 *CẢM BIẾN MỰC NƯỚC TÉC (ADMIN)*\n├─ `/water_on` `/water_off` — Bật/tắt cảm biến siêu âm\n├─ `/set_tank_height <cm>` — Chiều cao téc\n└─ `/set_water_offset <cm>` — Khoảng cách đo lúc téc ĐẦY\n\n";
    h += "📶 *MẠNG & BOT (ADMIN)*\n├─ `/list_wifi` — Xem DS WiFi\n├─ `/del_wifi <số>` — Xóa WiFi theo STT\n├─ `/set_wifi <SSID>;<PASS>` — Thêm WiFi\n└─ `/set_token <TOKEN>;<ID>` — Đổi Token Bot\n";
    return h;
}

String buildStatus(const char *mode) {
    unsigned long now = millis();
    if (strcmp(mode, "full") == 0 && (now - lastStatusBuild) < STATUS_CACHE_TTL_MS && cachedStatusFull.length() > 0) return cachedStatusFull;
    if (strcmp(mode, "short") == 0 && (now - lastStatusBuild) < STATUS_CACHE_TTL_MS && cachedStatusShort.length() > 0) return cachedStatusShort;

    portENTER_CRITICAL(&stateMux);
    bool    sM1On = slaveM1Online; bool sM2On = slaveM2Online; bool sHOn = slaveHOnline;
    uint8_t sM1F  = slaveM1Float;  uint8_t sM2F = slaveM2Float; uint8_t mf = masterFloat;
    int     rM1   = lastRssiM1;    int     rM2  = lastRssiM2;   int     rH = lastRssiH;
    unsigned long m1RunSec = pumpM1RuntimeTodaySec + (pumpM1OnSinceMs != 0 ? (millis() - pumpM1OnSinceMs) / 1000UL : 0);
    unsigned long m2RunSec = pumpM2RuntimeTodaySec + (pumpM2OnSinceMs != 0 ? (millis() - pumpM2OnSinceMs) / 1000UL : 0);
    unsigned long hRunSec  = pumpHRuntimeTodaySec  + (pumpHOnSinceMs  != 0 ? (millis() - pumpHOnSinceMs)  / 1000UL  : 0);
    portEXIT_CRITICAL(&stateMux);

    bool   wifiOk  = (WiFi.status() == WL_CONNECTED);
    String ip      = wifiOk ? WiFi.localIP().toString() : "n/a";
    bool   allGood = sM1On && sM2On && sHOn && wifiOk && loraHardwareOk;

    String msg = "🇻🇳 *TRẠM BƠM MỎ ĐÁ*\n" + divider();
    msg += String(allGood ? ICON_OK : ICON_WARN) + " *" + String(allGood ? "Hệ thống Bình Thường" : "Cảnh báo kết nối") + "*\n";
    msg += "⏱️ Uptime: " + formatTimeSpan(millis()) + "\n⚙️ Chế độ: " + String(controlMode == MODE_AUTO ? "🔄 TỰ ĐỘNG" : "🛠️ THỦ CÔNG") + "\n\n";
    msg += "💧 *MỰC NƯỚC TANK*\n├─ Master TANK    :  " + floatText(mf) + "\n├─ Moong 1 TANK:   " + floatText(sM1F) + "\n└─ Moong 2 TANK:   " + floatText(sM2F) + "\n\n";
    msg += "🚰 *MỰC NƯỚC TÉC (siêu âm)*\n└─ " + (waterSensorEnabled && waterSensorOk ? String(waterLevelPercent) + "%" : "Chưa sẵn sàng") + "\n\n";
    msg += "🔌 *TRẠNG THÁI BƠM*\n├─ Bơm Moong 1: " + pumpBadge(pumpM1) + "\n";
    msg += "├─ Bơm Moong 2: " + pumpBadge(pumpM2) + "\n";
    msg += "└─ BƠM HỒ     :      " + pumpBadge(pumpH) + (pumpH ? " (" + String(speedH) + "%)" : "") + "\n\n";

    if (trackedDayMarker != -1) {
        msg += "⏱️ *THỜI GIAN CHẠY BƠM HÔM NAY*\n├─ Moong 1: " + formatTimeSpan(m1RunSec * 1000UL) +
               "\n├─ Moong 2: " + formatTimeSpan(m2RunSec * 1000UL) +
               "\n└─ Trạm Hồ: " + formatTimeSpan(hRunSec * 1000UL) + "\n\n";
    }

    msg += "📶 *LORA:* " + statusBadge(sM1On) + "\n";
    msg += "🌐 *WIFI:* " + String(wifiOk ? ICON_OK : ICON_BAD) + " " + ip;
    { const char *hr = hbReason; msg += "\n🫀 *NHỊP G26:* " + (hr ? String(ICON_BAD) + " DỪNG - " + String(hr) : String(ICON_OK) + " hoạt động"); }

    if (strcmp(mode, "full") == 0) {
        cachedStatusFull = msg;
    } else {
        cachedStatusShort = msg;
    }
    lastStatusBuild = now;
    return msg;
}

void pendingAdd(const Packet &pkt) {
    portENTER_CRITICAL(&pendingMux);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (!pendingTable[i].used) {
            pendingTable[i].pkt        = pkt;
            pendingTable[i].used       = true;
            pendingTable[i].retries    = 0;
            pendingTable[i].lastSentAt = millis();
            break;
        }
    }
    portEXIT_CRITICAL(&pendingMux);
}

void pendingRemove(uint8_t sender, uint16_t seq) {
    portENTER_CRITICAL(&pendingMux);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (pendingTable[i].used && pendingTable[i].pkt.seq == seq && pendingTable[i].pkt.receiver == sender) {
            pendingTable[i].used = false;
            break;
        }
    }
    portEXIT_CRITICAL(&pendingMux);
}

void processPendingRetries() {
    unsigned long now = millis();
    Packet retryPkts[4]; int retryCount = 0;

    portENTER_CRITICAL(&pendingMux);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (!pendingTable[i].used) continue;
        if (now - pendingTable[i].lastSentAt < (ACK_DELAY_MS * 3)) continue;
        if (pendingTable[i].retries < CMD_MAX_RETRIES) {
            if (retryCount < 4) {
                retryPkts[retryCount++] = pendingTable[i].pkt;
                pendingTable[i].retries++;
                pendingTable[i].lastSentAt = now;
            } else {
                break;
            }
        } else {
            pendingTable[i].used = false;
        }
    }
    portEXIT_CRITICAL(&pendingMux);

    if (retryCount > 0 && xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < retryCount; i++) {
            uint8_t nextTail = (cmdTail + 1) % QUEUE_SIZE;
            if (nextTail != cmdHead) {
                cmdQueue[cmdTail].pkt = retryPkts[i];
                cmdTail = nextTail;
            }
        }
        xSemaphoreGive(coreSyncMutex);
    }
}

void setupWiFiSingle() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    if (!scanAndPickBestWifi()) Serial.println("[WIFI] Dùng cấu hình mặc định.");
    WiFi.begin(currentSsid.c_str(), currentPass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) { delay(200); }

    secured_client.setInsecure();    
    secured_client.setTimeout(TELEGRAM_HTTP_TIMEOUT_SEC);
    secured_client.setHandshakeTimeout(10);
    bootAlertPending = true;         
}

// Bổ sung hàm tự động phục hồi kết nối WiFi chủ động
void handleWiFiAutoRecovery() {
    if (WiFi.status() == WL_CONNECTED) {
        wifiLostTimestamp = 0; // Đã kết nối lại, reset mốc đếm
    } else {
        unsigned long now = millis();
        if (wifiLostTimestamp == 0) {
            wifiLostTimestamp = now; // Ghi nhận thời điểm bắt đầu mất mạng
        } 
        // Nếu mất mạng liên tục quá 2 phút, thực hiện ngắt và kết nối lại hoàn toàn
        else if (now - wifiLostTimestamp > 120000UL) {
            Serial.println("[WIFI] Mất mạng quá lâu, tiến hành quét và kết nối lại từ đầu...");
            WiFi.disconnect();
            delay(100);
            if (scanAndPickBestWifi()) {
                WiFi.begin(currentSsid.c_str(), currentPass.c_str());
            } else {
                WiFi.begin(WIFI_SSID_DEFAULT, WIFI_PASS_DEFAULT);
            }
            wifiLostTimestamp = millis(); // Đặt lại mốc thời gian tránh gọi dồn dập
        }
    }
}

bool tryInitLoRaOnce() {
    LoRa.setTxPower(TX_POWER);
    if (LoRa.begin(LORA_FREQ)) {
        LoRa.setSignalBandwidth(LORA_BW);
        LoRa.setSpreadingFactor(LORA_SF);
        LoRa.setCodingRate4(LORA_CR);
        LoRa.enableCrc();
        return true;
    }
    return false;
}

void initLoRa() {
    LoRa.setPins(LORA_M0, LORA_M1, LORA_AUX, Serial2, LORA_UART_RX, LORA_UART_TX);

    int  retry = 0;
    bool ok    = false;
    while (retry < LORA_INIT_MAX_RETRY) {
        ok = tryInitLoRaOnce();
        if (ok) break;
        retry++;
        delay(1000);
    }

    if (ok) {
        loraHardwareOk = true;
    } else {
        loraHardwareOk = false;
        sendTelegramDirect(alertBlock(ICON_CRIT, "LỖI PHẦN CỨNG LORA", "Không khởi tạo được module LoRa."));
    }
}

void disableBluetooth() {
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
}

void initWatchdog() {
    esp_task_wdt_config_t cfg = { .timeout_ms = WDT_TIMEOUT_SEC * 1000, .idle_core_mask = 0, .trigger_panic = true };
    esp_err_t e = esp_task_wdt_init(&cfg);
    if (e == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&cfg);
}

void initGPIO() {
    pinMode(FLOAT_MASTER_PIN, INPUT_PULLUP);
    pinMode(RELAY_M1_PIN, OUTPUT); pinMode(RELAY_M2_PIN, OUTPUT);
    digitalWrite(RELAY_M1_PIN, RELAY_OFF); digitalWrite(RELAY_M2_PIN, RELAY_OFF);

    pinMode(WATER_TRIG_PIN, OUTPUT);
    digitalWrite(WATER_TRIG_PIN, LOW);
    pinMode(WATER_ECHO_PIN, INPUT);
}

void handleNewMessages(int n) {
    for (int i = 0; i < n; i++) {
        String senderChatId = bot.messages[i].chat_id;
        String text = bot.messages[i].text; String user = bot.messages[i].from_name;
        user = escapeMarkdown(user);
        text.trim();

        if (text.startsWith("/start") || text == "/myid") {
            String outMsg;
            if (isRegisteredUser(senderChatId)) {
                outMsg = buildHelpMessage(isAdminUser(senderChatId));
            } else {
                outMsg = "👋 *CHÀO MỪNG*\nID: `" + senderChatId + "`";
            }
            sendTelegramReplyTo(senderChatId, outMsg);
            bot.messages[i].text = ""; bot.messages[i].from_name = "";
            continue;
        }

        if (!isRegisteredUser(senderChatId)) {
            sendTelegramReplyTo(senderChatId, "❌ Chưa được cấp quyền.");
            bot.messages[i].text = ""; bot.messages[i].from_name = "";
            continue;
        }

        bool senderIsAdmin = isAdminUser(senderChatId);
        bool isAdminOnlyCmd =
            text == "/reset" || text.startsWith("/seth ") ||
            text.startsWith("/set_tank_height ") || text.startsWith("/set_water_offset ") ||
            text == "/water_on" || text == "/water_off" ||
            text.startsWith("/set_wifi") || text.startsWith("/del_wifi") ||
            text.startsWith("/set_token") ||
            text == "/update" || text == "/update_list" ||
            text.startsWith("/update_to") || text == "/update_confirm" || text == "/update_cancel" ||
            text.startsWith("/lora_update") ||
            text.startsWith("/adduser") || text.startsWith("/deluser") ||
            text == "/users" || text == "/admin";

        if (isAdminOnlyCmd && !senderIsAdmin) {
            sendTelegramReplyTo(senderChatId, alertBlock(ICON_WARN, "Không đủ quyền", "Lệnh này chỉ dành cho ADMIN."));
            bot.messages[i].text = ""; bot.messages[i].from_name = "";
            continue;
        }

        if (text == "/help") {
            sendTelegramReplyTo(senderChatId, buildHelpMessage(senderIsAdmin));
        }
        else if (text.startsWith("/adduser")) {
            String args = text.substring(8); args.trim();
            addUserToList(args, ROLE_USER);
            sendTelegramReplyTo(senderChatId, "✅ Đã thêm người dùng");
        }
        else if (text.startsWith("/deluser")) {
            String args = text.substring(8); args.trim();
            removeUserFromList(args);
            sendTelegramReplyTo(senderChatId, "✅ Đã xóa người dùng");
        }
        else if (text == "/users") {
            sendTelegramReplyTo(senderChatId, buildUsersListMessage());
        }
        else if (text == "/status") {
            String outMsg = buildStatus("short");
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(20)) == pdTRUE) { telegramQueue.push(outMsg); xSemaphoreGive(telegramQueueMutex); }
        }
        else if (text == "/reset") {
            sendTelegramDirect(alertBlock(ICON_CRIT, "Khởi động lại toàn bộ"));
            telegramGetUpdatesSafe(bot.last_message_received + 1);
            delay(500);
            selfHealNoteCause("Lệnh /reset từ Telegram");
            runtimeSave(); ESP.restart();
        }
        else if (controlMode == MODE_MANUAL) {
            bool doAlert = false, turnOn = false;
            String pumpName = "";
            portENTER_CRITICAL(&stateMux);
            if      (text == "/bat_m1") { manualPumpM1 = true; doAlert = true; turnOn = true; pumpName = "Moong 1"; }
            else if (text == "/tat_m1") { manualPumpM1 = false; doAlert = true; turnOn = false; pumpName = "Moong 1"; }
            else if (text == "/bat_m2") { manualPumpM2 = true; doAlert = true; turnOn = true; pumpName = "Moong 2"; }
            else if (text == "/tat_m2") { manualPumpM2 = false; doAlert = true; turnOn = false; pumpName = "Moong 2"; }
            else if (text == "/bat_h")  { manualPumpH = true; doAlert = true; turnOn = true; pumpName = "Trạm Hồ"; }
            else if (text == "/tat_h")  { manualPumpH = false; doAlert = true; turnOn = false; pumpName = "Trạm Hồ"; }
            portEXIT_CRITICAL(&stateMux);
            if (doAlert) sendTelegramAlert(alertBlock(turnOn ? ICON_OK : ICON_BAD, "Lệnh điều khiển " + pumpName));
        }

        bot.messages[i].text = "";
        bot.messages[i].from_name = "";
    }
}

bool sendPacket(Packet &pkt) {
    if (loraMutex == NULL) return false;
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(LORA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&pkt, sizeof(Packet));
        bool ok = LoRa.endPacket(true);
        xSemaphoreGive(loraMutex);
        if (ok) lastLoraTxMs = millis();
        return ok;
    }
    return false;
}

void sendAck(uint8_t receiver, uint16_t seq) {
    if (xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t next = (ackTail + 1) % QUEUE_SIZE;
        if (next == ackHead) {
            xSemaphoreGive(coreSyncMutex);
            return;
        }
        Packet p = {MASTER_ID, receiver, MSG_ACK, seq, 0, (uint32_t)millis()};
        ackQueue[ackTail].pkt = p; ackTail = next;
        xSemaphoreGive(coreSyncMutex);
    }
}

bool queueCommand(uint8_t receiver, uint8_t payload) {
    bool queued = false;
    if (xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t next = (cmdTail + 1) % QUEUE_SIZE;
        if (next == cmdHead) {
            xSemaphoreGive(coreSyncMutex);
            return false;
        }
        Packet p = {MASTER_ID, receiver, MSG_COMMAND, txSeq++, payload, (uint32_t)millis()};
        cmdQueue[cmdTail].pkt = p; cmdTail = next;
        portENTER_CRITICAL(&pendingMux);
        for (int i = 0; i < QUEUE_SIZE; i++) {
            if (pendingTable[i].used && pendingTable[i].pkt.receiver == receiver && pendingTable[i].pkt.msgType == MSG_COMMAND)
                pendingTable[i].used = false;
        }
        portEXIT_CRITICAL(&pendingMux);
        pendingAdd(p);
        queued = true;
        xSemaphoreGive(coreSyncMutex);
    }
    return queued;
}

void processAckQueue() {
    if (relayQuiet()) return;
    bool has = false; Packet pkt;
    if (xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (ackHead != ackTail && millis() - lastAckSend >= ACK_DELAY_MS) {
            pkt = ackQueue[ackHead].pkt; ackHead = (ackHead + 1) % QUEUE_SIZE;
            has = true; lastAckSend = millis();
        }
        xSemaphoreGive(coreSyncMutex);
    }
    if (has) sendPacket(pkt);
}

void processCommandQueue() {
    if (relayQuiet()) return;
    bool has = false; Packet pkt;
    if (xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (cmdHead != cmdTail && millis() - lastCmdSend >= CMD_DELAY_MS) {
            pkt = cmdQueue[cmdHead].pkt; cmdHead = (cmdHead + 1) % QUEUE_SIZE;
            has = true; lastCmdSend = millis();
        }
        xSemaphoreGive(coreSyncMutex);
    }
    if (has) sendPacket(pkt);
}

void processLoRa() {
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    int sz = LoRa.parsePacket();
    if (sz == 0) { xSemaphoreGive(loraMutex); return; }

    if (sz != sizeof(Packet)) {
        while (LoRa.available()) LoRa.read();
        loraRxBadSize++;
        xSemaphoreGive(loraMutex);
        return;
    }

    Packet pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(Packet));
    int rssi = LoRa.packetRssi();
    xSemaphoreGive(loraMutex);
    loraRxOk++;
    unsigned long now = millis();

    bool     needAck     = false;
    uint8_t  ackReceiver = 0;
    uint16_t ackSeq      = 0;
    bool     isAck       = (pkt.msgType == MSG_ACK);
    bool     isOtaAck    = (pkt.msgType == MSG_OTA_ACK);

    portENTER_CRITICAL(&stateMux);
    switch (pkt.sender) {
        case SLAVE_M1_ID: {
            bool wasOfflineM1 = !slaveM1Online;
            slaveM1Online = true; slaveM1LastSeen = now; lastRssiM1 = rssi;
            if (pkt.msgType == MSG_FLOAT) {
                uint8_t newRaw = (pkt.payload & 0x01);
                if (newRaw != rawSlaveM1Float) { rawSlaveM1Float = newRaw; slaveM1FloatChangeTime = now; }
                m1AutoRunning = ((pkt.payload >> 1) & 0x01);
            }
            if (wasOfflineM1) needResyncM1 = true;
            if (!isAck && !isOtaAck) { needAck = true; ackReceiver = pkt.sender; ackSeq = pkt.seq; }
            break;
        }
        case SLAVE_M2_ID: {
            bool wasOfflineM2 = !slaveM2Online;
            slaveM2Online = true; slaveM2LastSeen = now; lastRssiM2 = rssi;
            if (pkt.msgType == MSG_FLOAT) {
                uint8_t newRaw = (pkt.payload & 0x01);
                if (newRaw != rawSlaveM2Float) { rawSlaveM2Float = newRaw; slaveM2FloatChangeTime = now; }
                m2AutoRunning = ((pkt.payload >> 1) & 0x01);
            }
            if (wasOfflineM2) needResyncM2 = true;
            if (!isAck && !isOtaAck) { needAck = true; ackReceiver = pkt.sender; ackSeq = pkt.seq; }
            break;
        }
        case SLAVE_H_ID:
            slaveHOnline = true; slaveHLastSeen = now; lastRssiH = rssi;
            if (!isAck && !isOtaAck) { needAck = true; ackReceiver = pkt.sender; ackSeq = pkt.seq; }
            break;
    }
    portEXIT_CRITICAL(&stateMux);

    if (isOtaAck) {
        portENTER_CRITICAL(&stateMux);
        otaAckSeq      = pkt.seq;
        otaAckStatus   = pkt.payload;
        otaAckReceived = true;
        portEXIT_CRITICAL(&stateMux);
    }

    if (isAck) pendingRemove(pkt.sender, pkt.seq);
    if (needAck) sendAck(ackReceiver, ackSeq);
}

void vLoRaRealtimeTask(void *pvParameters) {
    esp_task_wdt_add(NULL);
    unsigned long lastHeartbeat = millis();
    unsigned long lastRetryCheck = millis();
    unsigned long heartbeatInterval = HEARTBEAT_MS;

    for (;;) {
        esp_task_wdt_reset();
        loraAliveMs = millis();
        if (!loraHardwareOk) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        processLoRa();
        processAckQueue();
        processCommandQueue();

        if (millis() - lastRetryCheck > 5000UL) { processPendingRetries(); lastRetryCheck = millis(); }

        if (millis() - lastHeartbeat > heartbeatInterval && !relayQuiet()) {
            uint16_t seq = 0;
            if (xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                seq = txSeq++;
                xSemaphoreGive(coreSyncMutex);
            }
            Packet hb = {MASTER_ID, 255, MSG_HEARTBEAT, seq, 0, (uint32_t)millis()};
            sendPacket(hb); lastHeartbeat = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTelegramTask(void *pvParameters) {
    uint32_t lastProbe = millis();
    uint8_t  probeFail = 0;
    for (;;) {
        tgAliveMs = millis();
        if (otaRunning || WiFi.status() != WL_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (bootAlertPending) {
            bootAlertPending = false;
            sendTelegramAlert(alertBlock(ICON_SUCCESS, "Hệ thống đã kết nối", "Lý do khởi động: " + bootResetReason));
        }

        String msg = "";
        if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (!telegramQueue.empty()) { msg = telegramQueue.front(); telegramQueue.pop(); }
            xSemaphoreGive(telegramQueueMutex);
        }
        if (msg.length() > 0) {
            const char *md = (msg.indexOf('*') != -1) ? "Markdown" : "";
            if (userCount > 0) {
                for (int u = 0; u < userCount; u++) telegramSendMessageSafe(userIds[u], msg, md);
            } else {
                telegramSendMessageSafe(currentChatId, msg, md);
            }
        }
        if (millis() - lastTelegramPoll > TELEGRAM_POLL_MS) {
            int n = telegramGetUpdatesSafe(bot.last_message_received + 1);
            if (n > 0) handleNewMessages(n);
            lastTelegramPoll = millis();
        }
        if (millis() - lastProbe > TG_PROBE_MS) {
            lastProbe = millis();
            if (telegramProbeSafe()) probeFail = 0;
            else if (++probeFail >= 3) {                       
                probeFail = 0;
                Serial.println("[TG] Thăm dò lỗi 3 lần -> xả kết nối TLS + nối lại WiFi");
                releaseTelegramClient();
                WiFi.reconnect();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void checkSlaveTimeout() {
    unsigned long now = millis();
    if (now - bootTime < BOOT_GRACE_MS) return;
    bool aM1 = false, aM2 = false, aH = false;
    portENTER_CRITICAL(&stateMux);
    if (slaveM1Online && now - slaveM1LastSeen > SLAVE_TIMEOUT_MS) { slaveM1Online = false; aM1 = true; }
    if (slaveM2Online && now - slaveM2LastSeen > SLAVE_TIMEOUT_MS) { slaveM2Online = false; aM2 = true; }
    if (slaveHOnline  && now - slaveHLastSeen  > SLAVE_TIMEOUT_MS) { slaveHOnline  = false; aH  = true; }
    portEXIT_CRITICAL(&stateMux);
    if (aM1) sendTelegramAlert(alertBlock(ICON_CRIT, "Mất kết nối Moong 1 (>10p)"));
    if (aM2) sendTelegramAlert(alertBlock(ICON_CRIT, "Mất kết nối Moong 2 (>10p)"));
    if (aH)  sendTelegramAlert(alertBlock(ICON_CRIT, "Mất kết nối Trạm Hồ (>10p)"));
}

float waterMeasureOnceCm() {
    digitalWrite(WATER_TRIG_PIN, LOW);
    delayMicroseconds(3);
    digitalWrite(WATER_TRIG_PIN, HIGH);
    delayMicroseconds(12);
    digitalWrite(WATER_TRIG_PIN, LOW);

    unsigned long durationUs = pulseIn(WATER_ECHO_PIN, HIGH, WATER_PULSE_TIMEOUT_US);
    if (durationUs == 0) return -1;
    return (durationUs * 0.0343f) / 2.0f;
}

void waterSortSamples(float *a, int n) {
    for (int i = 0; i < n - 1; i++)
        for (int j = i + 1; j < n; j++)
            if (a[j] < a[i]) { float t = a[i]; a[i] = a[j]; a[j] = t; }
}

void readWaterLevelSensor() {
    if (!waterSensorEnabled) {
        waterSensorOk = false;
        return;
    }

    float samples[WATER_SAMPLE_COUNT];
    int   validCount = 0;
    for (int i = 0; i < WATER_SAMPLE_COUNT; i++) {
        float d = waterMeasureOnceCm();
        if (d > 0) samples[validCount++] = d;
        delay(15);
    }

    if (validCount == 0) {
        waterSensorOk = false;
        return;
    }

    waterSortSamples(samples, validCount);
    float distCm = samples[validCount / 2];

    waterLevelDistanceCm = distCm;
    waterSensorOk = true;

    float usableHeight = waterTankHeightCm - waterFullOffsetCm;
    float levelCm = waterTankHeightCm - distCm;
    if (usableHeight <= 0) usableHeight = waterTankHeightCm;

    int pct = (int) round((levelCm / usableHeight) * 100.0f);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    waterLevelPercent = pct;
}

void readMasterFloat() {
    uint8_t cur = (digitalRead(FLOAT_MASTER_PIN) == LOW) ? FLOAT_LOW : FLOAT_FULL;
    if (cur != rawMasterFloat) { rawMasterFloat = cur; floatChangeTime = millis(); }
    if (millis() - floatChangeTime >= FLOAT_DEBOUNCE_MS && masterFloat != rawMasterFloat) {
        masterFloat = rawMasterFloat;
        sendTelegramAlert(alertBlock(ICON_INFO, "Phao Master đổi trạng thái", floatText(masterFloat)));
    }
}

void processControl() {
    unsigned long now = millis();

    portENTER_CRITICAL(&stateMux);
    bool    sM1On          = slaveM1Online;
    bool    sM2On          = slaveM2Online;
    uint8_t sM1F           = slaveM1Float;
    uint8_t sM2F           = slaveM2Float;
    uint8_t currentMode    = controlMode;
    uint8_t currentTgSpeed = telegramSpeed;
    bool    mPumpM1        = manualPumpM1;
    bool    mPumpM2        = manualPumpM2;
    bool    mPumpH         = manualPumpH;
    uint8_t curMasterFloat = masterFloat;
    portEXIT_CRITICAL(&stateMux);

    bool localPumpM1 = false;
    bool localPumpM2 = false;

    if (currentMode == MODE_AUTO) {
        if (curMasterFloat == FLOAT_LOW) {
            localPumpM1 = false;
            localPumpM2 = false;
        } else {
            localPumpM1 = (sM1On && sM1F == FLOAT_LOW);
            localPumpM2 = (sM2On && sM2F == FLOAT_LOW);
        }
    } else {
        localPumpM1 = mPumpM1;
        localPumpM2 = mPumpM2;
    }

    bool targetPumpH  = false;
    uint8_t targetSpeedH = 0;

    if (currentMode == MODE_AUTO) {
        if (curMasterFloat == FLOAT_LOW) {
            targetPumpH  = true;
            targetSpeedH = 100;
        } else {
            if (localPumpM1 && localPumpM2)      { targetPumpH = true; targetSpeedH = 100; }
            else if (localPumpM1 || localPumpM2) { targetPumpH = true; targetSpeedH = currentTgSpeed; }
            else                                  { targetPumpH = false; targetSpeedH = 0; }
        }
    } else {
        targetPumpH  = mPumpH;
        targetSpeedH = mPumpH ? 100 : 0;
    }

    bool masterFloatChanged = (curMasterFloat != lastMasterFloatSent);
    bool logicChanged       = (targetPumpH != pumpH) || (targetSpeedH != lastSpeedH) || masterFloatChanged;
    bool timeoutResync      = (now - lastPumpHCmdTime > PUMP_H_RESYNC_MS);

    if (logicChanged || timeoutResync) {
        if (queueCommand(SLAVE_H_ID, targetSpeedH)) {
            lastPumpHCmdTime    = now;
            lastMasterFloatSent = curMasterFloat;
            portENTER_CRITICAL(&stateMux);
            pumpH  = targetPumpH;
            speedH = targetSpeedH;
            portEXIT_CRITICAL(&stateMux);
            lastSpeedH = targetSpeedH;
        }
    }

    {
        static bool          physM1 = false, physM2 = false;
        static unsigned long m1OffAt = 0, m2OffAt = 0;
        bool wantM1 = localPumpM1, wantM2 = localPumpM2;

        unsigned long since   = now - relayLastSwitchMs;
        bool          loraClr = otaLoraActive || ((now - lastLoraTxMs) >= RELAY_LORA_TX_CLEAR_MS);
        bool          done    = false;
        if (!done && physM1 && !wantM1 && since >= RELAY_OFF_GAP_MS && loraClr) { physM1 = false; m1OffAt = now; done = true; }
        if (!done && physM2 && !wantM2 && since >= RELAY_OFF_GAP_MS && loraClr) { physM2 = false; m2OffAt = now; done = true; }
        if (!done && !physM1 && wantM1 && since >= RELAY_ON_GAP_MS && (now - m1OffAt) >= RELAY_MIN_OFF_MS && loraClr) { physM1 = true; done = true; }
        if (!done && !physM2 && wantM2 && since >= RELAY_ON_GAP_MS && (now - m2OffAt) >= RELAY_MIN_OFF_MS && loraClr) { physM2 = true; done = true; }
        if (done) relayLastSwitchMs = now;
        
        localPumpM1 = physM1;
        localPumpM2 = physM2;
    }

    portENTER_CRITICAL(&stateMux);
    pumpM1 = localPumpM1;
    pumpM2 = localPumpM2;
    portEXIT_CRITICAL(&stateMux);

    digitalWrite(RELAY_M1_PIN, localPumpM1 ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_M2_PIN, localPumpM2 ? RELAY_ON : RELAY_OFF);
}

void checkScheduledReboot() {
    if (AUTO_REBOOT_INTERVAL_MS == 0) return;
    unsigned long now = millis();
    if (now - lastAutoRebootCheck < AUTO_REBOOT_CHECK_MS) return;
    lastAutoRebootCheck = now;
    if (now < AUTO_REBOOT_INTERVAL_MS) return;

    portENTER_CRITICAL(&stateMux);
    bool allPumpsOff = (!pumpM1 && !pumpM2 && !pumpH);
    portEXIT_CRITICAL(&stateMux);

    if (allPumpsOff) {
        sendTelegramDirect(alertBlock(ICON_INFO, "Khởi động lại định kỳ (bảo trì mỗi 6h)",
                            "Đã chạy liên tục " + formatTimeSpan(now) + ". Tất cả bơm đang TẮT, tiến hành khởi động lại."));
        delay(500);
        selfHealNoteCause("Bảo trì định kỳ mỗi 6 giờ");
        runtimeSave(); ESP.restart();
    }
}

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

static bool          rtNtpStarted = false;
static unsigned long rtLastTick = 0, rtLastSave = 0;
static unsigned long rtAcc[3] = {0, 0, 0};      

static int rtDayMarker() {
    time_t t = time(nullptr);
    if (t < 1600000000L) return -1;               
    struct tm tm; localtime_r(&t, &tm);
    return (tm.tm_year + 1900) * 1000 + tm.tm_yday;
}
static void runtimeSave() {
    if (trackedDayMarker == -1) return;
    runtimePrefs.begin("runtime", false);
    runtimePrefs.putInt("day", trackedDayMarker);
    runtimePrefs.putULong("m1", pumpM1RuntimeTodaySec);
    runtimePrefs.putULong("m2", pumpM2RuntimeTodaySec);
    runtimePrefs.putULong("h",  pumpHRuntimeTodaySec);
    runtimePrefs.end();
    rtLastSave = millis();
}
static void runtimeTick() {
    unsigned long now = millis();
    if (!rtNtpStarted && WiFi.status() == WL_CONNECTED) {
        configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com", "vn.pool.ntp.org");
        rtNtpStarted = true;
    }
    int day = rtDayMarker();
    if (day == -1) { rtLastTick = now; return; }  

    if (trackedDayMarker == -1) {                 
        runtimePrefs.begin("runtime", true);
        int savedDay = runtimePrefs.getInt("day", -1);
        unsigned long a = runtimePrefs.getULong("m1", 0), b = runtimePrefs.getULong("m2", 0), c = runtimePrefs.getULong("h", 0);
        runtimePrefs.end();
        if (savedDay == day) { pumpM1RuntimeTodaySec = a; pumpM2RuntimeTodaySec = b; pumpHRuntimeTodaySec = c; }
        else                 { pumpM1RuntimeTodaySec = pumpM2RuntimeTodaySec = pumpHRuntimeTodaySec = 0; }
        trackedDayMarker = day; rtLastTick = now; rtLastSave = now;
        return;
    }
    if (day != trackedDayMarker) {                
        pumpM1RuntimeTodaySec = pumpM2RuntimeTodaySec = pumpHRuntimeTodaySec = 0;
        rtAcc[0] = rtAcc[1] = rtAcc[2] = 0;
        trackedDayMarker = day;
        runtimeSave();
    }
    unsigned long dt = now - rtLastTick; rtLastTick = now;
    if (dt > 5000UL) dt = 5000UL;                 
    bool on[3] = {pumpM1, pumpM2, pumpH};
    volatile unsigned long *sec[3] = {&pumpM1RuntimeTodaySec, &pumpM2RuntimeTodaySec, &pumpHRuntimeTodaySec};
    for (int i = 0; i < 3; i++) {
        if (!on[i]) continue;
        rtAcc[i] += dt;
        if (rtAcc[i] >= 1000UL) { *sec[i] += rtAcc[i] / 1000UL; rtAcc[i] %= 1000UL; }
    }
    if (now - rtLastSave >= RUNTIME_CHECKPOINT_MS) runtimeSave();
}

#define TRAM_PUSH_KEY      "tramdien"      
#define TRAM_MDNS_NAME     "tramdien"      
#define TRAM_IP_FALLBACK   ""              
#define TRAM_PUSH_MS       3000UL
static IPAddress tramIp;
static bool      tramIpOk = false;
static bool      pushMdnsStarted = false;

static bool tramResolve() {
    if (tramIpOk) return true;
    String fb = TRAM_IP_FALLBACK;
    if (fb.length() && tramIp.fromString(fb)) { tramIpOk = true; return true; }
    if (!pushMdnsStarted) { MDNS.begin("master"); pushMdnsStarted = true; }
    IPAddress ip = MDNS.queryHost(TRAM_MDNS_NAME, 2000);
    if (ip != IPAddress((uint32_t)0)) { tramIp = ip; tramIpOk = true; return true; }
    return false;
}

static void vTramPushTask(void *) {
    vTaskDelay(pdMS_TO_TICKS(15000));                       
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TRAM_PUSH_MS));
        tramAliveMs = millis();
        if (WiFi.status() != WL_CONNECTED || otaRunning || otaLoraActive) continue;
        if (!tramResolve()) { vTaskDelay(pdMS_TO_TICKS(7000)); continue; }

        portENTER_CRITICAL(&stateMux);
        long v[32];
        v[0]  = controlMode;      v[1]  = pumpM1;   v[2]  = pumpM2;   v[3]  = pumpH;
        v[4]  = speedH;           v[5]  = telegramSpeed;
        v[6]  = slaveM1Online;    v[7]  = slaveM2Online;  v[8]  = slaveHOnline;
        v[9]  = slaveM1Float;     v[10] = slaveM2Float;   v[11] = masterFloat;
        v[12] = lastRssiM1;       v[13] = lastRssiM2;     v[14] = lastRssiH;
        unsigned long now = millis();
        v[15] = pumpM1RuntimeTodaySec + (pumpM1OnSinceMs ? (now - pumpM1OnSinceMs) / 1000UL : 0);
        v[16] = pumpM2RuntimeTodaySec + (pumpM2OnSinceMs ? (now - pumpM2OnSinceMs) / 1000UL : 0);
        v[17] = pumpHRuntimeTodaySec  + (pumpHOnSinceMs  ? (now - pumpHOnSinceMs)  / 1000UL : 0);
        portEXIT_CRITICAL(&stateMux);
        v[18] = waterLevelPercent;  v[19] = waterSensorOk;  v[20] = waterSensorEnabled;
        v[21] = (long)waterLevelDistanceCm;
        v[22] = loraHardwareOk;     v[23] = WiFi.RSSI();    v[24] = waterPumpHBlockedByFull;
        v[25] = (long)(millis() / 1000UL);
        v[26] = waterPumpBlockLowPercent;  v[27] = waterPumpBlockHighPercent;  v[28] = waterPumpHighRecoverPercent;
        v[29] = 0;                          
        v[30] = (trackedDayMarker != -1);   
        v[31] = 0;

        String d; d.reserve(160);
        for (int i = 0; i < 32; i++) { if (i) d += ','; d += String(v[i]); }
        String url = String("http://") + tramIp.toString() + "/phase_status?k=" TRAM_PUSH_KEY "&fw=" + FW_VERSION + "&d=" + d;

        WiFiClient cl; cl.setTimeout(3);
        HTTPClient http; http.setTimeout(3000); http.setReuse(false);
        bool ok = false;
        if (http.begin(cl, url)) { int code = http.GET(); ok = (code == 200); http.end(); }
        if (!ok) { tramIpOk = false; vTaskDelay(pdMS_TO_TICKS(5000)); }   
    }
}

void setup() {
    pinMode(RELAY_M1_PIN, OUTPUT); digitalWrite(RELAY_M1_PIN, RELAY_OFF);
    pinMode(RELAY_M2_PIN, OUTPUT); digitalWrite(RELAY_M2_PIN, RELAY_OFF);
    Serial.begin(115200);
    delay(100);

    disableBluetooth();
    logResetReason();
    randomSeed(esp_random());

    telegramQueueMutex  = xSemaphoreCreateMutex();
    loraMutex           = xSemaphoreCreateMutex();
    coreSyncMutex       = xSemaphoreCreateMutex();
    telegramClientMutex = xSemaphoreCreateMutex();
    alertLogMutex       = xSemaphoreCreateMutex();

    if (!LittleFS.begin(true)) {
        Serial.println("[LORA-OTA] Không mount được LittleFS.");
    }

    for (int i = 0; i < QUEUE_SIZE; i++) pendingTable[i].used = false;
    for (int i = 0; i < ALERT_LOG_SIZE; i++) alertLog[i] = "";

    loadWifiCredentials();
    loadTelegramCredentials();
    loadUsersFromNVS();

    configPrefs.begin("pump_cfg", true);
    telegramSpeed = configPrefs.getInt("h_speed", 100);
    configPrefs.end();

    waterCfgPrefs.begin("water_cfg", true);
    waterTankHeightCm  = waterCfgPrefs.getInt("height", WATER_TANK_HEIGHT_CM_DEFAULT);
    waterFullOffsetCm  = waterCfgPrefs.getInt("offset", WATER_FULL_OFFSET_CM_DEFAULT);
    waterSensorEnabled = waterCfgPrefs.getBool("enabled", WATER_SENSOR_ENABLED_DEFAULT);
    waterCfgPrefs.end();

    floatPrefs.begin("float_cfg", true);
    uint8_t savedM1Float = floatPrefs.getUChar("m1", FLOAT_UNKNOWN);
    uint8_t savedM2Float = floatPrefs.getUChar("m2", FLOAT_UNKNOWN);
    floatPrefs.end();
    slaveM1Float = rawSlaveM1Float = savedM1Float;
    slaveM2Float = rawSlaveM2Float = savedM2Float;

    bot.updateToken(currentBotToken);

    initGPIO();
    setupWiFiSingle();
    initLoRa();
    initWatchdog();

    esp_task_wdt_add(NULL);

    bot.longPoll = 0;
    bot.maxMessageLength = 4096;
    bot.waitForResponse  = 6000;
    
    unsigned long ms = millis();
    slaveM1LastSeen = ms; slaveM2LastSeen = ms; slaveHLastSeen = ms; bootTime = ms;
    lastAutoRebootCheck = ms;

    xTaskCreatePinnedToCore(vLoRaRealtimeTask, "LoRaTask",     4096, NULL, 5, &loraTaskHandle,     0);
    xTaskCreatePinnedToCore(vTelegramTask,     "TelegramTask", 12288, NULL, 1, &telegramTaskHandle, 1);

    pinMode(HB_OUT_PIN, OUTPUT); digitalWrite(HB_OUT_PIN, LOW);
    Serial1.begin(9600, SERIAL_8N1, -1, HB_UART_TX);   
    loopAliveMs = loraAliveMs = tgAliveMs = tramAliveMs = tgLastOkMs = millis();
    xTaskCreate(vHeartbeatOutTask, "HbOut", 5120, NULL, 1, NULL);
    xTaskCreatePinnedToCore(vTramPushTask, "TramPush", 6144, NULL, 1, NULL, 1);   
    relayLastSwitchMs = millis();
}

void loop() {
    esp_task_wdt_reset();
    loopAliveMs = millis();
    
    // Tích hợp kiểm tra và tự động kết nối lại WiFi chủ động mỗi vòng lặp
    handleWiFiAutoRecovery();

    checkSlaveTimeout();
    runtimeTick();
    readMasterFloat();
    if (millis() - lastWaterReadMs >= WATER_READ_INTERVAL_MS && !relayQuiet()) {
        lastWaterReadMs = millis();
        readWaterLevelSensor();
    }
    processControl();
    checkScheduledReboot();
    bool busy = pumpM1 || pumpM2 || pumpH || otaLoraActive;
    vTaskDelay(pdMS_TO_TICKS(busy ? LOOP_DELAY_ACTIVE_MS : LOOP_DELAY_IDLE_MS));
}