bool g_loraReady = false;
/*****************************************************************
 * MONOLITHIC MASTER NODE - ESP32 + SX1278 + TELEGRAM
 * VERSION 15.11.2 - OTA UPDATE QUA GITHUB
 *
 * THAY ĐỔI SO VỚI v15.11.1:
 *  [FIX-19] Thêm tính năng OTA (cập nhật firmware qua mạng) lấy file
 *           firmware.bin + version.txt từ repo GitHub public
 *           (Trong99x/trambom, nhánh main). Chỉ kích hoạt THỦ CÔNG
 *           bằng lệnh Telegram /update (không tự động định kỳ, tránh
 *           tự ý flash lúc đang vận hành). Thêm lệnh /version để xem
 *           bản hiện tại.
 *
 * VERSION 15.10.0 - FIX PHAO SLAVE "ĐÓNG BĂNG" Ở TRẠNG THÁI CŨ SAU
 *                    KHI MASTER KHỞI ĐỘNG LẠI (WATCHDOG/RESET)
 *
 * THAY ĐỔI SO VỚI v15.9.0:
 *  [FIX-13] (Option A - Persist NVS) slaveM1Float/slaveM2Float được
 *           lưu vào NVS mỗi khi debounce commit giá trị mới, và được
 *           nạp lại trong setup(). Trước đây, sau mỗi lần reboot,
 *           2 biến này luôn reset về FLOAT_FULL (Đầy) bất kể trạng
 *           thái thật, và CHỈ được cập nhật lại khi slave gửi một
 *           gói MSG_FLOAT MỚI. Vì slave chỉ gửi MSG_FLOAT khi trạng
 *           thái phao của nó THAY ĐỔI (tiết kiệm băng thông LoRa),
 *           nếu phao vẫn cạn y nguyên từ trước khi Master treo/reset,
 *           slave sẽ không gửi lại gói nào nữa → Master hiển thị
 *           sai "Đầy" vô thời hạn → không bật lại bơm dù tank đang
 *           cạn thật ngoài đời.
 *  [FIX-14] (Option B - Trạng thái UNKNOWN) Thêm FLOAT_UNKNOWN cho
 *           trường hợp chưa từng có dữ liệu NVS (lần đầu flash máy)
 *           lẫn chưa có gói MSG_FLOAT nào từ slave. Hệ thống giữ bơm
 *           tắt (an toàn) NHƯNG bắn cảnh báo Telegram rõ ràng thay vì
 *           âm thầm hiển thị "Đầy" giả.
 *  [FIX-15] (Option C - Resync chủ động + mở rộng giao thức) Khi một
 *           slave M1/M2 chuyển từ OFFLINE → ONLINE (kể cả lần kết nối
 *           đầu tiên sau khi Master reboot), Master chủ động gửi lệnh
 *           CMD_REQUEST_STATUS yêu cầu slave báo cáo lại trạng thái
 *           phao ngay lập tức (CẦN slave nâng cấp firmware để phản
 *           hồi lệnh này bằng một gói MSG_FLOAT). Đồng thời, Master
 *           cũng chấp nhận đọc trạng thái phao lồng trong gói
 *           MSG_HEARTBEAT nếu slave đánh dấu bit FLOAT_DATA_VALID_BIT
 *           (tương thích ngược 100% với slave cũ chưa nâng cấp).
 *
 * THAY ĐỔI SO VỚI v15.8.0:
 *  [FIX-10] Thêm biến lastMasterFloatSent để theo dõi trạng thái
 *           phao Master tại thời điểm gửi lệnh H gần nhất.
 *           Khi phao Master đổi trạng thái (dù targetSpeedH không
 *           đổi), buộc gửi lại lệnh xác nhận cho Slave H.
 *           Kịch bản lỗi: Master CẠN trong khi H đang BẬT do
 *           M1/M2 → logicChanged=false → không gửi lệnh → bơm H
 *           không phản ứng với phao Master cạn.
 *  [FIX-11] lastPumpHCmdTime chỉ cập nhật khi queueCommand thành
 *           công. Trước đây cập nhật kể cả khi thất bại → phải
 *           chờ 60s mới retry thay vì retry ngay chu kỳ sau.
 *  [FIX-12] Dùng snapshot curMasterFloat trong processControl()
 *           thay vì đọc trực tiếp biến volatile masterFloat nhiều
 *           lần → tránh race condition giữa readMasterFloat() và
 *           processControl() chạy cùng loop().
 *****************************************************************/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <SPI.h>
#include <LoRa.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <Preferences.h>
#include <queue>
#include "time.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//======================================================
// CẤU HÌNH MẠNG & TELEGRAM MẶC ĐỊNH
//======================================================
#define WIFI_SSID_DEFAULT "MO DA"
#define WIFI_PASS_DEFAULT "88888888"

#define BOT_TOKEN_DEFAULT "8777904636:AAGMBfWJDtN8WvsUtXhYT9SEXBfAnKL9ySE"
#define CHAT_ID_DEFAULT   "8752050398"

//======================================================
// OTA UPDATE QUA GITHUB
// Repo public: https://github.com/Trong99x/trambom
// Quy trình cập nhật: build .bin mới -> đổi số trong version.txt ->
// upload đè cả 2 file (firmware.bin + version.txt) lên nhánh main ->
// gửi lệnh /update qua Telegram. Master chỉ kiểm tra & cập nhật khi
// NHẬN LỆNH, không tự động định kỳ.
//======================================================
#define FW_VERSION        "15.11.2"
#define OTA_GITHUB_OWNER  "Trong99x"
#define OTA_GITHUB_REPO   "trambom"
#define OTA_GITHUB_BRANCH "main"
#define OTA_VERSION_URL   "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/version.txt"
#define OTA_FIRMWARE_URL  "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/firmware.bin"
#define OTA_HTTP_TIMEOUT_MS 15000UL

//======================================================
// ĐỊNH DANH NODE
//======================================================
#define MASTER_ID    0
#define SLAVE_M1_ID  1
#define SLAVE_M2_ID  2
#define SLAVE_H_ID   3

//======================================================
// CHÂN SPI / LORA
//======================================================
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS     5
#define LORA_RST   14
#define LORA_DIO0  26

//======================================================
// THÔNG SỐ LORA
//======================================================
#define LORA_FREQ  433E6
#define LORA_BW    125E3
#define LORA_SF    12
#define LORA_CR    5
#define TX_POWER   20
#define LORA_INIT_MAX_RETRY  5

//======================================================
// CHÂN GPIO
//======================================================
#define FLOAT_MASTER_PIN 32
#define RELAY_M1_PIN     27
#define RELAY_M2_PIN     25

//======================================================
// TIMING HỆ THỐNG
//======================================================
#define HEARTBEAT_MS              600000UL
#define HEARTBEAT_JITTER_MS         3000UL
#define SLAVE_TIMEOUT_MS         600000UL
#define FLOAT_DEBOUNCE_MS          5000UL
#define ACK_DELAY_MS               500UL
#define CMD_DELAY_MS              1000UL
#define PUMP_H_RESYNC_MS         60000UL

// [FIX] Bit cao nhất trong payload ACK/HEARTBEAT của SLAVE_H_ID mang
// trạng thái relay VẬT LÝ THẬT (đọc từ tiếp điểm phụ NO tại Trạm Hồ,
// đã debounce ở slave) — phải khớp định nghĩa RELAY_STATE_BIT bên
// slave_h_v1_2.ino.
#define RELAY_STATE_BIT         0x80
// Thời gian ân hạn cho phép slave H kịp ACK (+ tối đa 5 lần retry mỗi
// 1.5s theo cơ chế queueCommand), relay cơ khí kịp đóng/mở, và debounce
// kịp ổn định — trước khi coi mismatch là thật sự.
#define RELAY_H_CONFIRM_GRACE_MS 10000UL
#define TELEGRAM_POLL_MS           7000UL
#define WDT_TIMEOUT_SEC               60
#define LORA_MUTEX_TIMEOUT_MS      300UL
#define WIFI_RECONNECT_MS         50000UL
#define WIFI_OFFLINE_FAILSAFE_MS 300000UL
#define WIFI_WAKE_INTERVAL_MS   3600000UL
#define WIFI_WAKE_TRY_MS          8000UL
#define TELEGRAM_HTTP_TIMEOUT_SEC    10
#define STATUS_CACHE_TTL_MS       20000UL
#define CMD_MAX_RETRIES               5
#define WIFI_SWITCH_DELAY_MS       3000UL
#define BOOT_GRACE_MS            120000UL

#define SLAVE_FLOAT_DEBOUNCE_MS    5000UL

#define RESOURCE_MONITOR_MS      620000UL
#define HEAP_WARN_THRESHOLD_BYTES  30000UL

#define AUTO_REBOOT_INTERVAL_MS 259200000UL
#define AUTO_REBOOT_CHECK_MS      660000UL

// [FIX-16] Đồng bộ giờ NTP + tự khởi động lại theo lịch cố định 00:00
// hàng ngày (độc lập với AUTO_REBOOT_INTERVAL_MS theo uptime ở trên).
#define NTP_SERVER1            "pool.ntp.org"
#define NTP_SERVER2            "time.google.com"
#define GMT_OFFSET_SEC         (7 * 3600)   // Việt Nam UTC+7, không DST
#define DAYLIGHT_OFFSET_SEC    0
#define DAILY_REBOOT_HOUR      0
#define DAILY_REBOOT_MINUTE    0
#define DAILY_REBOOT_CHECK_MS  20000UL

//======================================================
// LOGIC CONSTANTS
//======================================================
#define FLOAT_FULL    1
#define FLOAT_LOW     0
// [FIX-14] Trạng thái "chưa xác định" — dùng khi chưa từng nhận được
// dữ liệu phao thật (chưa có trong NVS, chưa có gói MSG_FLOAT nào).
// KHÔNG được coi là FLOAT_FULL để tránh giả định sai "Đầy".
#define FLOAT_UNKNOWN 2
#define RELAY_ON    HIGH
#define RELAY_OFF   LOW
#define MODE_AUTO   0
#define MODE_MANUAL 1

// [FIX-15] Payload đặc biệt trong MSG_COMMAND gửi tới SLAVE_M1_ID /
// SLAVE_M2_ID để yêu cầu slave báo cáo lại trạng thái phao ngay lập
// tức (không cần đợi trạng thái phao thay đổi). Giá trị 0xFE nằm
// ngoài dải tốc độ bơm hợp lệ (0-100%) nên không xung đột với lệnh
// điều khiển tốc độ bơm Hồ hiện có.
#define CMD_REQUEST_STATUS 0xFE

// [FIX-15] Bit đánh dấu "payload có chứa dữ liệu phao hợp lệ" trong
// gói MSG_HEARTBEAT do slave gửi. Chỉ áp dụng cho MSG_HEARTBEAT (để
// không phá vỡ giao thức MSG_FLOAT hiện có) và yêu cầu slave nâng
// cấp firmware để bật bit này khi nhúng trạng thái phao vào heartbeat.
#define FLOAT_DATA_VALID_BIT 0x80

//======================================================
// UI CONSTANTS
//======================================================
#define UI_DIVIDER   "━━━━━━━━━━━━━━━━━━━━"
#define ICON_OK      "🟢"
#define ICON_BAD     "🔴"
#define ICON_WARN    "⚠️"
#define ICON_CRIT    "🚨"
#define ICON_SUCCESS "✅"
#define ICON_INFO    "ℹ️"

#define ALERT_LOG_SIZE 8
portMUX_TYPE alertMux = portMUX_INITIALIZER_UNLOCKED;
String       alertLog[ALERT_LOG_SIZE];
int          alertLogIdx = 0;

//======================================================
// CẤU TRÚC GÓI TIN PACKET
//======================================================
enum MessageType : uint8_t {
    MSG_HEARTBEAT = 1,
    MSG_FLOAT     = 2,
    MSG_COMMAND   = 3,
    MSG_ACK       = 4
};

typedef struct __attribute__((packed)) {
    uint8_t  sender;
    uint8_t  receiver;
    uint8_t  msgType;
    uint16_t seq;
    uint8_t  payload;
    uint32_t uptime;
} Packet;

//======================================================
// PENDING TABLE
//======================================================
#define QUEUE_SIZE 64
struct PendingEntry {
    Packet        pkt;
    bool          used;
    uint8_t       retries;
    unsigned long lastSentAt;
};
PendingEntry pendingTable[QUEUE_SIZE];
portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;

//======================================================
// MẠNG VÀ ĐỐI TƯỢNG TELEGRAM / PREFERENCES
//======================================================
String                currentBotToken = BOT_TOKEN_DEFAULT;
String                currentChatId   = CHAT_ID_DEFAULT;
Preferences           tgPrefs;
Preferences           configPrefs;
Preferences           floatPrefs;   // [FIX-13] namespace riêng lưu trạng thái phao slave qua reboot

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

volatile bool          pendingWifiSwitch     = false;
volatile unsigned long wifiSwitchRequestedAt = 0;

//======================================================
// BIẾN TRẠNG THÁI LIÊN CORE
//======================================================
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool          slaveM1Online         = false;
volatile bool          slaveM2Online         = false;
volatile bool          slaveHOnline          = false;
// [FIX-13/14] Giá trị khởi tạo thật sự được nạp lại trong setup() từ
// NVS (trạng thái đã lưu trước khi reboot). Nếu NVS chưa có dữ liệu
// (lần đầu flash), giữ FLOAT_UNKNOWN thay vì giả định FLOAT_FULL.
volatile uint8_t       slaveM1Float          = FLOAT_UNKNOWN;
volatile uint8_t       slaveM2Float          = FLOAT_UNKNOWN;
volatile unsigned long slaveM1LastSeen        = 0;
volatile unsigned long slaveM2LastSeen        = 0;
volatile unsigned long slaveHLastSeen         = 0;

volatile uint8_t       rawSlaveM1Float        = FLOAT_UNKNOWN;
volatile unsigned long slaveM1FloatChangeTime = 0;
volatile uint8_t       rawSlaveM2Float        = FLOAT_UNKNOWN;
volatile unsigned long slaveM2FloatChangeTime = 0;

// [FIX-15] Cờ báo cần gửi yêu cầu resync trạng thái phao khi slave
// vừa chuyển từ OFFLINE → ONLINE (đặt trong processLoRa, xử lý ở
// vòng lặp vLoRaRealtimeTask để tránh gọi queueCommand bên trong
// critical section).
volatile bool          needResyncM1 = false;
volatile bool          needResyncM2 = false;

// [FIX-14] Đảm bảo chỉ cảnh báo "phao chưa xác định" một lần sau boot,
// tránh spam Telegram.
bool floatUnknownAlerted = false;

volatile bool          m1AutoRunning         = false;
volatile bool          m2AutoRunning         = false;

volatile int           lastRssiM1 = -120;
volatile int           lastRssiM2 = -120;
volatile int           lastRssiH  = -120;
volatile unsigned long bootTime   = 0;

// [FIX] Trạng thái relay Trạm Hồ do CHÍNH slave H báo lại — nay là xác
// nhận VẬT LÝ THẬT qua tiếp điểm phụ NO (không còn là echo phần mềm),
// phân biệt với `pumpH` là trạng thái Master RA LỆNH. Hai biến này phải
// khớp nhau sau một khoảng ân hạn ngắn; nếu lệch dai dẳng nghĩa là relay
// vật lý không đóng/mở đúng như lệnh (kẹt tiếp điểm, cháy cuộn hút, đứt
// dây feedback...).
volatile bool          slaveHRelayConfirmed   = false;
volatile bool          slaveHRelayHasData     = false; // chưa nhận gói nào từ H thì chưa biết gì
volatile unsigned long slaveHRelayConfirmTime = 0;
bool                    relayHMismatchAlerted  = false;

//======================================================
// BIẾN TẬP LỆNH / LOGIC
//======================================================
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

volatile uint8_t       telegramSpeed = 100;

uint8_t       masterFloat     = FLOAT_FULL;
uint8_t       rawMasterFloat  = FLOAT_FULL;
unsigned long floatChangeTime = 0;

// [FIX-10] Theo dõi trạng thái phao Master tại thời điểm gửi lệnh H
// gần nhất. Khởi tạo FLOAT_FULL để lần đầu phao cạn luôn trigger gửi.
uint8_t       lastMasterFloatSent = FLOAT_FULL;

unsigned long lastReconnectAttempt = 0;
unsigned long lastPumpHCmdTime     = 0;
unsigned long lastTelegramPoll     = 0;

//======================================================
// CACHE VÀ THỐNG KÊ LORA
//======================================================
String        cachedStatusShort = "";
String        cachedStatusFull  = "";
unsigned long lastStatusBuild   = 0;

bool          wifiWasDown             = false;
unsigned long wifiDownSince           = 0;
volatile bool wifiDisabledPermanently = false;
unsigned long lastWifiWakeAttempt     = 0;

volatile uint32_t loraRxOk      = 0;
volatile uint32_t loraCrcErr    = 0;
volatile uint32_t loraRxBadSize = 0;

unsigned long lastResourceCheck  = 0;
uint32_t      minFreeHeapEver    = UINT32_MAX;

unsigned long lastAutoRebootCheck = 0;

// [FIX-16] Trạng thái đồng bộ NTP + chống lặp reset trong cùng 1 ngày.
bool          ntpTimeSynced       = false;
unsigned long lastNtpSyncAttempt  = 0;
unsigned long lastDailyRebootCheck = 0;
int           lastDailyRebootYday = -1;

String bootResetReason = "";

//======================================================
// HÀNG ĐỢI LORA & TELEGRAM
//======================================================
struct AckItem { Packet pkt; } ackQueue[QUEUE_SIZE];
volatile uint8_t ackHead = 0, ackTail = 0;
unsigned long    lastAckSend = 0;

struct CmdItem { Packet pkt; } cmdQueue[QUEUE_SIZE];
volatile uint8_t cmdHead = 0, cmdTail = 0;
unsigned long    lastCmdSend = 0;

std::queue<String> telegramQueue;
const size_t       MAX_TELEGRAM_QUEUE_SIZE = 15;

SemaphoreHandle_t loraMutex          = NULL;
SemaphoreHandle_t telegramQueueMutex = NULL;
SemaphoreHandle_t coreSyncMutex      = NULL;
TaskHandle_t      loraTaskHandle     = NULL;
TaskHandle_t      telegramTaskHandle = NULL;

//======================================================
// HỖ TRỢ GIAO DIỆN - FORMAT TEXT
//======================================================
String divider() { return String(UI_DIVIDER) + "\n"; }

String floatText(uint8_t state) {
    // [FIX-14] Phân biệt rõ "chưa xác định" với "Đầy" — tránh hiển thị
    // sai lệch khiến người vận hành tưởng nhầm tank đang đầy.
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

String formatTimeSpan(unsigned long ms) {
    unsigned long s = ms / 1000;
    unsigned long m = s / 60; s %= 60;
    unsigned long h = m / 60; m %= 60;
    char buf[20];
    snprintf(buf, sizeof(buf), "%luh %02lum %02lus", h, m, s);
    return String(buf);
}

void logAlert(const String &msg) {
    portENTER_CRITICAL(&alertMux);
    alertLog[alertLogIdx] = msg;
    alertLogIdx = (alertLogIdx + 1) % ALERT_LOG_SIZE;
    portEXIT_CRITICAL(&alertMux);
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

void sendTelegramDirect(const String &msg) {
    logAlert(msg);
    bool md = (msg.indexOf('*') != -1);
    bot.sendMessage(currentChatId, msg, md ? "Markdown" : "");
}

//======================================================
// OTA: KIỂM TRA & CẬP NHẬT FIRMWARE TỪ GITHUB
// Chỉ được gọi khi user gõ /update qua Telegram (không tự động).
//======================================================
void performOTAUpdate() {
    if (WiFi.status() != WL_CONNECTED) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Chưa kết nối WiFi, không thể kiểm tra bản mới."));
        return;
    }

    // --- Bước 1: Đọc version.txt trên GitHub ---
    WiFiClientSecure verClient;
    verClient.setInsecure(); // raw.githubusercontent.com dùng CA chuẩn, setInsecure để đơn giản như các nơi khác trong code
    HTTPClient http;
    http.setTimeout(OTA_HTTP_TIMEOUT_MS);

    if (!http.begin(verClient, OTA_VERSION_URL)) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Không mở được kết nối tới version.txt trên GitHub."));
        return;
    }

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại",
            "Không tải được version.txt (HTTP " + String(code) + ").\nKiểm tra lại repo/branch/tên file có đúng không."));
        http.end();
        return;
    }

    String remoteVersion = http.getString();
    remoteVersion.trim();
    http.end();

    if (remoteVersion.length() == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "version.txt rỗng hoặc không đọc được."));
        return;
    }

    if (remoteVersion == FW_VERSION) {
        sendTelegramDirect(alertBlock(ICON_OK, "Đã là bản mới nhất",
            "Phiên bản hiện tại: " + String(FW_VERSION)));
        return;
    }

    // --- Bước 2: Có bản mới -> tải firmware.bin và flash ---
    sendTelegramDirect(alertBlock(ICON_WARN, "Phát hiện bản mới, đang cập nhật...",
        "Hiện tại: " + String(FW_VERSION) + " → Mới: " + remoteVersion +
        "\n⏳ Thiết bị sẽ tự khởi động lại sau khi cập nhật xong. Vui lòng đợi ~1-2 phút."));

    WiFiClientSecure fwClient;
    fwClient.setInsecure();
    httpUpdate.rebootOnUpdate(true);

    t_httpUpdate_return ret = httpUpdate.update(fwClient, OTA_FIRMWARE_URL);

    // Nếu update thành công, dòng dưới đây sẽ KHÔNG chạy tới vì thiết bị
    // đã tự reboot. Chỉ còn chạy tới khi có lỗi.
    switch (ret) {
        case HTTP_UPDATE_FAILED:
            sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại",
                "Lỗi #" + String(httpUpdate.getLastError()) + ": " + httpUpdate.getLastErrorString() +
                "\nKiểm tra: firmware.bin có tồn tại đúng repo/branch, dung lượng bin có vượt quá phân vùng flash không."));
            break;
        case HTTP_UPDATE_NO_UPDATES:
            sendTelegramDirect(alertBlock(ICON_INFO, "OTA: Không có gì để cập nhật (server báo không đổi)."));
            break;
        default:
            break;
    }
}

//======================================================
// GHI NHẬN LÝ DO RESET LẦN KHỞI ĐỘNG GẦN NHẤT
//======================================================
String getResetReasonText() {
    esp_reset_reason_t r = esp_reset_reason();
    switch (r) {
        case ESP_RST_POWERON:   return "Cấp nguồn (Power On)";
        case ESP_RST_EXT:       return "Reset từ chân EXT/nút nhấn";
        case ESP_RST_SW:        return "Reset phần mềm (ESP.restart)";
        case ESP_RST_PANIC:     return "🚨 PANIC / Lỗi phần mềm nghiêm trọng";
        case ESP_RST_INT_WDT:   return "🚨 WATCHDOG NGẮT (Interrupt WDT)";
        case ESP_RST_TASK_WDT:  return "🚨 WATCHDOG TASK - Có task bị treo";
        case ESP_RST_WDT:       return "🚨 WATCHDOG khác (RTC/XTAL)";
        case ESP_RST_DEEPSLEEP: return "Thức dậy từ Deep Sleep";
        case ESP_RST_BROWNOUT:  return "🚨 SỤT ÁP NGUỒN (Brownout)";
        case ESP_RST_SDIO:      return "Reset qua SDIO";
        default:                return "Không xác định (mã " + String((int)r) + ")";
    }
}

void logResetReason() {
    bootResetReason = getResetReasonText();
    Serial.println("[BOOT] Lý do khởi động/reset lần trước: " + bootResetReason);
}

//======================================================
// LƯU TRỮ WIFI NVS ĐỘNG MẠNG
//======================================================
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
    if (targetIndex < 0 || targetIndex >= savedWifiCount) {
        Serial.println("❌ Số WiFi không hợp lệ.");
        return;
    }
    for (int i = targetIndex; i < savedWifiCount - 1; i++) {
        savedWifiList[i].ssid = savedWifiList[i + 1].ssid;
        savedWifiList[i].pass = savedWifiList[i + 1].pass;
    }
    savedWifiList[savedWifiCount - 1].ssid = "";
    savedWifiList[savedWifiCount - 1].pass = "";
    savedWifiCount--;
    if (savedWifiCount == 0) {
        savedWifiList[0].ssid = WIFI_SSID_DEFAULT;
        savedWifiList[0].pass = WIFI_PASS_DEFAULT;
        savedWifiCount = 1;
    }
    saveWifiListToNVS();
    Serial.printf("ℹ️ Đã xóa WiFi số %d thành công.\n", index);
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
    delete testClient;
    return result;
}

String buildAlertSummary() {
    String out = "";
    portENTER_CRITICAL(&alertMux);
    int count = 0;
    for (int i = 0; i < ALERT_LOG_SIZE; i++) {
        int idx = (alertLogIdx - 1 - i + ALERT_LOG_SIZE) % ALERT_LOG_SIZE;
        if (alertLog[idx].length() > 0 && count < 5) {
            out += "├─ " + alertLog[idx] + "\n";
            count++;
        }
    }
    portEXIT_CRITICAL(&alertMux);
    if (out.length() == 0) out = ICON_OK " Không có cảnh báo gần đây.\n";
    return out;
}

String buildPendingListMessage() {
    String m = "📋 *LỆNH CHỜ XÁC NHẬN*\n" + divider();
    int count = 0;
    unsigned long now = millis();
    portENTER_CRITICAL(&pendingMux);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (pendingTable[i].used) {
            String dest;
            switch (pendingTable[i].pkt.receiver) {
                case SLAVE_M1_ID: dest = "Moong 1"; break;
                case SLAVE_M2_ID: dest = "Moong 2"; break;
                case SLAVE_H_ID:  dest = "Trạm Hồ"; break;
                default:          dest = "ID " + String(pendingTable[i].pkt.receiver);
            }
            m += "├─ #" + String(pendingTable[i].pkt.seq) + " → " + dest +
                 " | Thử: " + String(pendingTable[i].retries) +
                 " | Chờ: " + String((now - pendingTable[i].lastSentAt) / 1000) + "s\n";
            count++;
        }
    }
    portEXIT_CRITICAL(&pendingMux);
    if (count == 0) m += ICON_OK " Không có lệnh nào đang chờ.\n";
    return m;
}

String buildHelpMessage() {
    String h = "🇻🇳 *TRẠM BƠM MỎ ĐÁ — HƯỚNG DẪN*\n" + divider();
    h += "📊 *GIÁM SÁT*\n├─ `/status` — Tổng quan hệ thống\n├─ `/status_full` — Chi tiết cấu hình + Log lỗi\n├─ `/pending` — Lệnh LoRa chưa nhận ACK\n\n";
    h += "⚙️ *VẬN HÀNH*\n├─ `/auto` — Chế độ TỰ ĐỘNG\n└─ `/manual` — Chế độ THỦ CÔNG\n\n";
    h += "🛠️ *THỦ CÔNG* (Gõ /manual trước)\n├─ `/bat_m1` `/tat_m1` — Bơm Moong 1\n├─ `/bat_m2` `/tat_m2` — Bơm Moong 2\n└─ `/bat_h` `/tat_h` — Bơm Trạm Hồ\n\n";
    h += "📐 *CẤU HÌNH TỐC ĐỘ*\n└─ `/seth [0-100]` — Đặt tốc độ nền khi chạy 1 bơm (%)\n";
    h += "📶 *MẠNG & BOT*\n├─ `/list_wifi` — Xem DS WiFi đã lưu\n├─ `/del_wifi <số>` — Xóa WiFi theo STT\n├─ `/set_wifi <SSID>;<PASS>` — Thêm WiFi\n└─ `/set_token <TOKEN>;<ID>` — Đổi Token Bot\n";
    h += "🚀 *FIRMWARE*\n├─ `/version` — Xem phiên bản hiện tại\n└─ `/update` — Kiểm tra & cập nhật firmware từ GitHub\n";
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
    // [FIX-18] Snapshot bit "AutoRunning" mà slave M1/M2 tự báo về qua
    // LoRa, để hiển thị lên /status thay vì bỏ phí như trước đây.
    bool    m1Auto = m1AutoRunning; bool m2Auto = m2AutoRunning;
    portEXIT_CRITICAL(&stateMux);

    bool   wifiOk  = (WiFi.status() == WL_CONNECTED);
    String ip      = wifiOk ? WiFi.localIP().toString() : "n/a";
    bool   allGood = sM1On && sM2On && sHOn && wifiOk;

    String msg = "🇻🇳 *TRẠM BƠM MỎ ĐÁ*\n" + divider();
    msg += String(allGood ? ICON_OK : ICON_WARN) + " *" + String(allGood ? "Hệ thống Bình Thường" : "Cảnh báo kết nối") + "*\n";
    msg += "⏱️ Uptime: " + formatTimeSpan(millis()) + "\n⚙️ Chế độ: " + String(controlMode == MODE_AUTO ? "🔄 TỰ ĐỘNG" : "🛠️ THỦ CÔNG") + "\n\n";
    msg += "💧 *MỰC NƯỚC TANK*\n├─ Master TANK    :  " + floatText(mf) + "\n├─ Moong 1 TANK:   " + floatText(sM1F) + "\n└─ Moong 2 TANK:   " + floatText(sM2F) + "\n\n";
    msg += "🔌 *TRẠNG THÁI BƠM*\n├─ Bơm Moong 1: " + pumpBadge(pumpM1) + "\n";
    msg += "├─ Bơm Moong 2: " + pumpBadge(pumpM2) + "\n";
    msg += "└─ BƠM HỒ     :      " + pumpBadge(pumpH) + (pumpH ? " (" + String(speedH) + "%)" : "") + "\n\n";
    // [FIX-18] Trạng thái "AutoRunning" do CHÍNH slave M1/M2 tự báo về —
    // đây là dữ liệu do slave gửi lên (khác với pumpM1/pumpM2 phía trên
    // là lệnh Master tự ra cho relay tại chỗ). Lưu ý: cần xem code slave
    // M1/M2 để biết chính xác slave dùng bit này báo cáo cái gì.
    msg += "📟 *SLAVE TỰ BÁO CÁO (M1/M2)*\n├─ Moong 1: " + String(sM1On ? (m1Auto ? ICON_OK " Đang chạy" : ICON_BAD " Không chạy") : "n/a (offline)") + "\n";
    msg += "└─ Moong 2: " + String(sM2On ? (m2Auto ? ICON_OK " Đang chạy" : ICON_BAD " Không chạy") : "n/a (offline)") + "\n\n";
    msg += "📐 Cài đặt công suất bơm Hồ: *" + String(telegramSpeed) + "%*\n\n";
    msg += "📡 *TÍN HIỆU LORA*\n├─ Moong 1: " + statusBadge(sM1On) + (sM1On ? " " + signalBadge(rM1) : "") + "\n├─ Moong 2: " + statusBadge(sM2On) + (sM2On ? " " + signalBadge(rM2) : "") + "\n└─ Trạm Hồ: " + statusBadge(sHOn) + (sHOn ? " " + signalBadge(rH) : "") + "\n\n";
    msg += "🌐 *WIFI:* " + String(wifiOk ? ICON_OK : ICON_BAD) + " " + (wifiOk ? WiFi.SSID() + " (" + ip + ")" : "MẤT KẾT NỐI");

    if (strcmp(mode, "full") == 0) {
        msg += "\n" + divider() + "📊 *THỐNG KÊ LORA*\n├─ Nhận OK: " + String(loraRxOk) + "\n├─ Lỗi CRC: " + String(loraCrcErr) + "\n└─ Sai cỡ: " + String(loraRxBadSize) + "\n";
        msg += "\n" + divider() + "🧠 *TÀI NGUYÊN HỆ THỐNG*\n";
        uint32_t minHeapKB = (minFreeHeapEver == UINT32_MAX ? ESP.getFreeHeap() : minFreeHeapEver) / 1024;
        msg += "├─ Free Heap hiện tại: " + String(ESP.getFreeHeap() / 1024) + " KB\n";
        msg += "├─ Free Heap thấp nhất: " + String(minHeapKB) + " KB\n";
        msg += "└─ Lý do khởi động lần trước: " + bootResetReason + "\n";
        msg += "\n" + divider() + "📋 *LOG CẢNH BÁO MỚI*\n" + buildAlertSummary();
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

// [FIX-16] Đồng bộ giờ thực qua NTP. Gọi khi WiFi vừa kết nối (lần
// đầu và mỗi lần reconnect sau khi rớt mạng) để đảm bảo checkDailyReboot()
// luôn có giờ chính xác dùng so sánh mốc 00:00 hàng ngày.
void trySyncNtpTime() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 3000)) {
        ntpTimeSynced = true;
        Serial.printf("[NTP] Đồng bộ giờ thành công: %02d:%02d:%02d %02d/%02d/%04d\n",
                      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                      timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    } else {
        Serial.println("[NTP] Đồng bộ giờ thất bại, sẽ thử lại sau.");
    }
}

// [FIX-16] Tự khởi động lại đúng 00:00 mỗi ngày (giờ Việt Nam), độc
// lập với AUTO_REBOOT_INTERVAL_MS (reboot theo uptime + yêu cầu bơm
// tắt) đã có sẵn ở trên. Dùng tm_yday để đảm bảo CHỈ reset một lần
// trong ngày dù hàm này được gọi lặp lại nhiều lần trong khung phút 00:00.
void checkDailyReboot() {
    unsigned long now = millis();
    if (now - lastDailyRebootCheck < DAILY_REBOOT_CHECK_MS) return;
    lastDailyRebootCheck = now;

    // Nếu chưa từng đồng bộ được giờ (mất mạng lâu), thử lại định kỳ
    // nhưng không chặn hoạt động bình thường của hệ thống.
    if (!ntpTimeSynced) {
        if (WiFi.status() == WL_CONNECTED && now - lastNtpSyncAttempt > 60000UL) {
            lastNtpSyncAttempt = now;
            trySyncNtpTime();
        }
        return;
    }

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 50)) return;

    if (timeinfo.tm_yday == lastDailyRebootYday) return; // đã reset trong ngày hôm nay rồi

    if (timeinfo.tm_hour == DAILY_REBOOT_HOUR && timeinfo.tm_min == DAILY_REBOOT_MINUTE) {
        lastDailyRebootYday = timeinfo.tm_yday;
        sendTelegramDirect(alertBlock(ICON_INFO, "Khởi động lại định kỳ 00:00 hàng ngày",
                            "Đã chạy liên tục " + formatTimeSpan(now) + ". Tiến hành khởi động lại theo lịch."));
        delay(500);
        ESP.restart();
    }
}

void setupWiFiSingle() {
    WiFi.mode(WIFI_STA);
    if (!scanAndPickBestWifi()) Serial.println("[WIFI] Dùng cấu hình mặc định ban đầu.");
    WiFi.begin(currentSsid.c_str(), currentPass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) { delay(200); }

    if (WiFi.status() != WL_CONNECTED) {
        for (int i = 0; i < savedWifiCount; i++) {
            if (savedWifiList[i].ssid == currentSsid) continue;
            WiFi.disconnect();
            currentSsid = savedWifiList[i].ssid; currentPass = savedWifiList[i].pass;
            WiFi.begin(currentSsid.c_str(), currentPass.c_str());
            unsigned long ts = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - ts < 5000) { delay(200); }
            if (WiFi.status() == WL_CONNECTED) break;
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        secured_client.setInsecure(); secured_client.setTimeout(TELEGRAM_HTTP_TIMEOUT_SEC);
        trySyncNtpTime(); // [FIX-16] đồng bộ giờ ngay khi có mạng lần đầu
        sendTelegramAlert(alertBlock(ICON_SUCCESS, "Hệ thống đã kết nối", "Lý do khởi động: " + bootResetReason));
    }
}

void initLoRa() {
    // 1. Khởi tạo kết nối SPI
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    // [FIX-17] THỰC HIỆN RESET CỨNG CHIP LORA (giật chân RST vật lý
    // LOW → HIGH) TƯỜNG MINH ngay tại đây, trước khi gọi LoRa.begin().
    // Thư viện LoRa gốc vốn đã tự làm việc này ngầm bên trong begin(),
    // nhưng làm rõ ràng ở đây đảm bảo 100% chip SX1278 luôn được đưa
    // về trạng thái sạch (xuất xưởng) mỗi khi Master khởi động lại —
    // bất kể do Watchdog, sụt áp (Brownout), nạp code, hay bất kỳ lý
    // do nào khác — mà không phụ thuộc vào chi tiết cài đặt bên trong
    // của thư viện LoRa đang dùng.
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(20);
    digitalWrite(LORA_RST, HIGH);
    delay(50);

    // 2. Khai báo chân cho thư viện LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    g_loraReady = LoRa.begin(LORA_FREQ);
    if(!g_loraReady){Serial.println("[LORA] Missing, continue."); return;}
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setTxPower(TX_POWER);
    LoRa.enableCrc();
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
}

void handleNewMessages(int n) {
    for (int i = 0; i < n; i++) {
        if (bot.messages[i].chat_id != currentChatId) continue;
        String text = bot.messages[i].text; String user = bot.messages[i].from_name;
        text.trim();

        if (text.startsWith("/start") || text.startsWith("/help")) {
            String outMsg = buildHelpMessage();
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(20)) == pdTRUE) { telegramQueue.push(outMsg); xSemaphoreGive(telegramQueueMutex); }
        }
        else if (text == "/status") {
            cachedStatusShort = "";
            String outMsg = buildStatus("short");
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(20)) == pdTRUE) { telegramQueue.push(outMsg); xSemaphoreGive(telegramQueueMutex); }
        }
        else if (text == "/status_full") {
            cachedStatusFull = "";
            String outMsg = buildStatus("full");
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(20)) == pdTRUE) { telegramQueue.push(outMsg); xSemaphoreGive(telegramQueueMutex); }
        }
        else if (text == "/pending") {
            String outMsg = buildPendingListMessage();
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(20)) == pdTRUE) { telegramQueue.push(outMsg); xSemaphoreGive(telegramQueueMutex); }
        }
        else if (text == "/list_wifi") {
            String wifiListMsg = "📋 *DANH SÁCH WIFI ĐÃ LƯU*\n" + divider();
            for (int j = 0; j < savedWifiCount; j++) {
                wifiListMsg += "├─ " + String(j + 1) + ". SSID: `" + savedWifiList[j].ssid + "` | PASS: `" + savedWifiList[j].pass + "`\n";
            }
            if (savedWifiCount == 0) wifiListMsg += "❌ Chưa lưu WiFi nào.\n";
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(20)) == pdTRUE) { telegramQueue.push(wifiListMsg); xSemaphoreGive(telegramQueueMutex); }
        }
        else if (text.startsWith("/del_wifi")) {
            String replyMsg = "";
            int spaceIndex = text.indexOf(' ');
            if (spaceIndex != -1) {
                String indexStr = text.substring(spaceIndex + 1); indexStr.trim();
                int wifiIndex = indexStr.toInt();
                if (wifiIndex > 0 && wifiIndex <= savedWifiCount) {
                    String deletedSsid = savedWifiList[wifiIndex - 1].ssid;
                    deleteWifiByIndex(wifiIndex);
                    replyMsg = "✅ *THÀNH CÔNG*\n" + divider();
                    replyMsg += "🗑️ Đã xóa WiFi số *" + String(wifiIndex) + "* (SSID: `" + deletedSsid + "`) ra khỏi bộ nhớ.\n";
                    replyMsg += "💡 Gõ `/list_wifi` để xem lại danh sách cập nhật.";
                } else {
                    replyMsg = "❌ *THẤT BẠI*: Số thứ tự WiFi không tồn tại.\nℹ️ Vui lòng gõ `/list_wifi` để xem chính xác số thứ tự.";
                }
            } else {
                replyMsg = "⚠️ *SAI CÚ PHÁP*\nℹ️ Vui lòng nhập theo định dạng: `/del_wifi <số_thứ_tự>`\n_Ví dụ: /del_wifi 2_";
            }
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(20)) == pdTRUE) { telegramQueue.push(replyMsg); xSemaphoreGive(telegramQueueMutex); }
        }
        else if (text == "/auto") {
            portENTER_CRITICAL(&stateMux);
            controlMode = MODE_AUTO;
            portEXIT_CRITICAL(&stateMux);
            sendTelegramAlert(alertBlock(ICON_SUCCESS, "CHUYỂN CHẾ ĐỘ: TỰ ĐỘNG", "User: " + user));
        }
        else if (text == "/manual") {
            portENTER_CRITICAL(&stateMux);
            controlMode = MODE_MANUAL;
            portEXIT_CRITICAL(&stateMux);
            sendTelegramAlert(alertBlock(ICON_WARN, "CHUYỂN CHẾ ĐỘ: THỦ CÔNG", "User: " + user));
        }
        else if (text.startsWith("/seth ")) {
            int val = text.substring(6).toInt();
            if (val < 0) val = 0; if (val > 100) val = 100;
            portENTER_CRITICAL(&stateMux);
            telegramSpeed = val;
            portEXIT_CRITICAL(&stateMux);
            configPrefs.begin("pump_cfg", false);
            configPrefs.putInt("h_speed", val);
            configPrefs.end();
            sendTelegramAlert(alertBlock(ICON_INFO, "Cài đặt công suất bơm Hồ", String(val) + "% bởi " + user));
        }
        else if (text.startsWith("/set_wifi")) {
            String args = text.substring(9); args.trim();
            String newSsid, newPass; int semiIdx = args.indexOf(';');
            if (semiIdx != -1) { newSsid = args.substring(0, semiIdx); newPass = args.substring(semiIdx + 1); }
            newSsid.trim(); newPass.trim();
            if (newSsid.length() > 0 && newPass.length() > 0) {
                int res = addOrUpdateWifiNetwork(newSsid, newPass);
                if (res != 0) {
                    sendTelegramDirect(alertBlock(ICON_SUCCESS, "Lưu WiFi mới", "SSID: " + newSsid));
                    pendingWifiSwitch = true; wifiSwitchRequestedAt = millis();
                }
            }
        }
        else if (text == "/version") {
            sendTelegramDirect(alertBlock(ICON_INFO, "Phiên bản firmware hiện tại", String(FW_VERSION)));
        }
        else if (text == "/update") {
            sendTelegramDirect(alertBlock(ICON_INFO, "Đang kiểm tra bản mới trên GitHub..."));
            performOTAUpdate();
        }
        else if (text.startsWith("/set_token")) {
            String args = text.substring(10); args.trim();
            String newToken, newChatId; int semiIdx = args.indexOf(';');
            if (semiIdx != -1) { newToken = args.substring(0, semiIdx); newChatId = args.substring(semiIdx + 1); }
            newToken.trim(); newChatId.trim();
            if (newToken.length() > 0 && newChatId.length() > 0) {
                if (testTelegramToken(newToken, newChatId)) {
                    currentBotToken = newToken; currentChatId = newChatId;
                    saveTelegramCredentialsToNVS(newToken, newChatId);
                    bot.updateToken(newToken);
                    sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đổi Token Bot hoàn tất."));
                }
            }
        }
        else if (controlMode == MODE_MANUAL) {
            bool    doAlert  = false;
            bool    turnOn   = false;
            String  pumpName = "";

            portENTER_CRITICAL(&stateMux);
            if      (text == "/bat_m1") { manualPumpM1 = true;  doAlert = true; turnOn = true;  pumpName = "Moong 1"; }
            else if (text == "/tat_m1") { manualPumpM1 = false; doAlert = true; turnOn = false; pumpName = "Moong 1"; }
            else if (text == "/bat_m2") { manualPumpM2 = true;  doAlert = true; turnOn = true;  pumpName = "Moong 2"; }
            else if (text == "/tat_m2") { manualPumpM2 = false; doAlert = true; turnOn = false; pumpName = "Moong 2"; }
            else if (text == "/bat_h")  { manualPumpH  = true;  doAlert = true; turnOn = true;  pumpName = "Trạm Hồ"; }
            else if (text == "/tat_h")  { manualPumpH  = false; doAlert = true; turnOn = false; pumpName = "Trạm Hồ"; }
            portEXIT_CRITICAL(&stateMux);

            if (doAlert) {
                sendTelegramAlert(alertBlock(turnOn ? ICON_OK : ICON_BAD,
                                             String("Lệnh ") + (turnOn ? "BẬT " : "TẮT ") + pumpName));
            }
        }

        bot.messages[i].text = "";
        bot.messages[i].from_name = "";
    }
}

bool sendPacket(Packet &pkt) {
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(LORA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&pkt, sizeof(Packet));
        bool ok = LoRa.endPacket(true);
        xSemaphoreGive(loraMutex);
        return ok;
    }
    return false;
}

void sendAck(uint8_t receiver, uint16_t seq) {
    if (xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t next = (ackTail + 1) % QUEUE_SIZE;
        if (next == ackHead) {
            Serial.println("🚨 Hàng đợi ACK đầy! Bỏ qua ACK mới.");
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
            Serial.println("🚨 Hàng đợi LỆNH đầy! Từ chối nạp lệnh mới.");
            xSemaphoreGive(coreSyncMutex);
            return false;
        }
        Packet p = {MASTER_ID, receiver, MSG_COMMAND, txSeq++, payload, (uint32_t)millis()};
        cmdQueue[cmdTail].pkt = p; cmdTail = next;
        pendingAdd(p);
        queued = true;
        xSemaphoreGive(coreSyncMutex);
    }
    return queued;
}

void processAckQueue() {
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
    if(!g_loraReady) return;
    int sz = LoRa.parsePacket();
    if (sz == 0) return;

    if (sz != sizeof(Packet)) {
        while (LoRa.available()) LoRa.read();
        loraRxBadSize++;
        return;
    }

    Packet pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(Packet));
    int rssi = LoRa.packetRssi();
    loraRxOk++;
    unsigned long now = millis();

    bool     needAck     = false;
    uint8_t  ackReceiver = 0;
    uint16_t ackSeq      = 0;
    bool     isAck       = (pkt.msgType == MSG_ACK);

    portENTER_CRITICAL(&stateMux);
    switch (pkt.sender) {
        case SLAVE_M1_ID: {
            // [FIX-15] Phát hiện slave vừa (re)kết nối — bao gồm cả lần
            // đầu sau khi Master reboot (slaveM1Online khởi tạo = false).
            bool wasOfflineM1 = !slaveM1Online;
            slaveM1Online = true; slaveM1LastSeen = now; lastRssiM1 = rssi;
            if (pkt.msgType == MSG_FLOAT) {
                // Hành vi gốc, KHÔNG đổi: MSG_FLOAT luôn được tin tưởng.
                uint8_t newRaw = (pkt.payload & 0x01);
                if (newRaw != rawSlaveM1Float) { rawSlaveM1Float = newRaw; slaveM1FloatChangeTime = now; }
                m1AutoRunning = ((pkt.payload >> 1) & 0x01);
            } else if (pkt.msgType == MSG_HEARTBEAT && (pkt.payload & FLOAT_DATA_VALID_BIT)) {
                // [FIX-15] Chỉ nhận nếu slave (đã nâng cấp) đánh dấu bit
                // hợp lệ — tránh hiểu nhầm payload=0 mặc định thành CẠN.
                uint8_t newRaw = (pkt.payload & 0x01);
                if (newRaw != rawSlaveM1Float) { rawSlaveM1Float = newRaw; slaveM1FloatChangeTime = now; }
                m1AutoRunning = ((pkt.payload >> 1) & 0x01);
            }
            if (wasOfflineM1) needResyncM1 = true;
            if (!isAck) { needAck = true; ackReceiver = pkt.sender; ackSeq = pkt.seq; }
            break;
        }

        case SLAVE_M2_ID: {
            bool wasOfflineM2 = !slaveM2Online;
            slaveM2Online = true; slaveM2LastSeen = now; lastRssiM2 = rssi;
            if (pkt.msgType == MSG_FLOAT) {
                uint8_t newRaw = (pkt.payload & 0x01);
                if (newRaw != rawSlaveM2Float) { rawSlaveM2Float = newRaw; slaveM2FloatChangeTime = now; }
                m2AutoRunning = ((pkt.payload >> 1) & 0x01);
            } else if (pkt.msgType == MSG_HEARTBEAT && (pkt.payload & FLOAT_DATA_VALID_BIT)) {
                uint8_t newRaw = (pkt.payload & 0x01);
                if (newRaw != rawSlaveM2Float) { rawSlaveM2Float = newRaw; slaveM2FloatChangeTime = now; }
                m2AutoRunning = ((pkt.payload >> 1) & 0x01);
            }
            if (wasOfflineM2) needResyncM2 = true;
            if (!isAck) { needAck = true; ackReceiver = pkt.sender; ackSeq = pkt.seq; }
            break;
        }

        case SLAVE_H_ID:
            slaveHOnline = true; slaveHLastSeen = now; lastRssiH = rssi;
            // [FIX] MSG_ACK và MSG_HEARTBEAT từ slave H đều mang
            // RELAY_STATE_BIT — trạng thái relay thật slave vừa áp dụng.
            // Trước đây payload bị bỏ qua hoàn toàn ở nhánh này.
            if (pkt.msgType == MSG_ACK || pkt.msgType == MSG_HEARTBEAT) {
                slaveHRelayConfirmed   = (pkt.payload & RELAY_STATE_BIT) != 0;
                slaveHRelayHasData     = true;
                slaveHRelayConfirmTime = now;
            }
            if (!isAck) { needAck = true; ackReceiver = pkt.sender; ackSeq = pkt.seq; }
            break;
    }
    portEXIT_CRITICAL(&stateMux);

    if (isAck) pendingRemove(pkt.sender, pkt.seq);
    if (needAck) sendAck(ackReceiver, ackSeq);

    // [FIX-15] Gửi yêu cầu resync NGOÀI critical section (queueCommand
    // dùng xSemaphoreTake, không được gọi trong portENTER_CRITICAL).
    // Lưu ý: cần slave nâng cấp để xử lý MSG_COMMAND payload=CMD_REQUEST_STATUS
    // và phản hồi ngay bằng một gói MSG_FLOAT chứa trạng thái hiện tại.
    if (needResyncM1) {
        needResyncM1 = false;
        if (queueCommand(SLAVE_M1_ID, CMD_REQUEST_STATUS)) {
            sendTelegramAlert(alertBlock(ICON_INFO, "Slave Moong 1 vừa kết nối lại",
                               "Đã gửi yêu cầu xác nhận lại trạng thái phao."));
        }
    }
    if (needResyncM2) {
        needResyncM2 = false;
        if (queueCommand(SLAVE_M2_ID, CMD_REQUEST_STATUS)) {
            sendTelegramAlert(alertBlock(ICON_INFO, "Slave Moong 2 vừa kết nối lại",
                               "Đã gửi yêu cầu xác nhận lại trạng thái phao."));
        }
    }
}

// [FIX-14] Cảnh báo MỘT LẦN nếu sau thời gian ân hạn khởi động vẫn
// chưa xác định được trạng thái phao thật của M1/M2 (không có dữ liệu
// NVS lẫn chưa nhận được gói MSG_FLOAT/MSG_HEARTBEAT hợp lệ nào).
// Bơm tương ứng vẫn được giữ TẮT (an toàn) — mục đích của hàm này chỉ
// là để admin biết cần kiểm tra tay thay vì hệ thống âm thầm sai.
void checkFloatUnknownAlert() {
    if (floatUnknownAlerted) return;
    if (millis() - bootTime < BOOT_GRACE_MS) return;

    portENTER_CRITICAL(&stateMux);
    uint8_t f1 = slaveM1Float, f2 = slaveM2Float;
    portEXIT_CRITICAL(&stateMux);

    if (f1 == FLOAT_UNKNOWN || f2 == FLOAT_UNKNOWN) {
        String detail = "";
        if (f1 == FLOAT_UNKNOWN) detail += " ▪️ Moong 1: chưa có dữ liệu phao (bơm đang giữ TẮT để an toàn)\n";
        if (f2 == FLOAT_UNKNOWN) detail += " ▪️ Moong 2: chưa có dữ liệu phao (bơm đang giữ TẮT để an toàn)";
        sendTelegramAlert(alertBlock(ICON_WARN, "Chưa xác định trạng thái phao sau khởi động", detail));
        floatUnknownAlerted = true;
    }
}

void debounceSlaveFloats() {
    unsigned long now = millis();

    portENTER_CRITICAL(&stateMux);
    uint8_t rawM1 = rawSlaveM1Float; unsigned long chM1 = slaveM1FloatChangeTime; uint8_t curM1 = slaveM1Float;
    uint8_t rawM2 = rawSlaveM2Float; unsigned long chM2 = slaveM2FloatChangeTime; uint8_t curM2 = slaveM2Float;
    portEXIT_CRITICAL(&stateMux);

    bool alertM1 = false, alertM2 = false;
    uint8_t newM1 = curM1, newM2 = curM2;

    if (curM1 != rawM1 && now - chM1 >= SLAVE_FLOAT_DEBOUNCE_MS) { newM1 = rawM1; alertM1 = true; }
    if (curM2 != rawM2 && now - chM2 >= SLAVE_FLOAT_DEBOUNCE_MS) { newM2 = rawM2; alertM2 = true; }

    if (alertM1 || alertM2) {
        portENTER_CRITICAL(&stateMux);
        if (alertM1) slaveM1Float = newM1;
        if (alertM2) slaveM2Float = newM2;
        portEXIT_CRITICAL(&stateMux);
    }

    // [FIX-13] Lưu trạng thái phao mới nhất vào NVS ngay khi commit,
    // để nếu Master bị treo/reset ngay sau đó, lần boot kế tiếp vẫn
    // biết đúng trạng thái gần nhất thay vì mặc định "Đầy"/"Unknown".
    if (alertM1 || alertM2) {
        floatPrefs.begin("float_cfg", false);
        if (alertM1) floatPrefs.putUChar("m1", newM1);
        if (alertM2) floatPrefs.putUChar("m2", newM2);
        floatPrefs.end();
    }

    if (alertM1) sendTelegramAlert(alertBlock(ICON_INFO, "Phao Moong 1 đổi trạng thái", floatText(newM1)));
    if (alertM2) sendTelegramAlert(alertBlock(ICON_INFO, "Phao Moong 2 đổi trạng thái", floatText(newM2)));
}

//======================================================
// SYSTEM REALTIME TASK - CORE 0
//======================================================
void vLoRaRealtimeTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    unsigned long lastHeartbeat     = millis();
    unsigned long lastRetryCheck    = millis();
    unsigned long heartbeatInterval = HEARTBEAT_MS;

    for (;;) {
        esp_task_wdt_reset();
        processLoRa();
        processAckQueue();
        processCommandQueue();

        if (millis() - lastRetryCheck > 5000UL) { processPendingRetries(); lastRetryCheck = millis(); }

        if (millis() - lastHeartbeat > heartbeatInterval) {
            uint16_t seq = 0;
            if (xSemaphoreTake(coreSyncMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                seq = txSeq++;
                xSemaphoreGive(coreSyncMutex);
            }
            Packet hb = {MASTER_ID, 255, MSG_HEARTBEAT, seq, 0, (uint32_t)millis()};
            sendPacket(hb); lastHeartbeat = millis();
            long jitter = random(-(long)HEARTBEAT_JITTER_MS, (long)HEARTBEAT_JITTER_MS + 1);
            heartbeatInterval = HEARTBEAT_MS + jitter;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//======================================================
// TELEGRAM ASYNC TASK - CORE 1
//======================================================
void vTelegramTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();
        if (pendingWifiSwitch && millis() - wifiSwitchRequestedAt >= WIFI_SWITCH_DELAY_MS) {
            pendingWifiSwitch = false; WiFi.disconnect(true, true); vTaskDelay(pdMS_TO_TICKS(200)); WiFi.mode(WIFI_STA);
            scanAndPickBestWifi(); WiFi.begin(currentSsid.c_str(), currentPass.c_str());
            wifiWasDown = true; wifiDownSince = millis(); wifiDisabledPermanently = false; lastReconnectAttempt = millis();
        }

        if (wifiDisabledPermanently) {
            if (millis() - lastWifiWakeAttempt >= WIFI_WAKE_INTERVAL_MS) {
                lastWifiWakeAttempt = millis(); WiFi.mode(WIFI_STA); scanAndPickBestWifi(); WiFi.begin(currentSsid.c_str(), currentPass.c_str());
                unsigned long ws = millis();
                while (WiFi.status() != WL_CONNECTED && millis() - ws < WIFI_WAKE_TRY_MS) { esp_task_wdt_reset(); vTaskDelay(pdMS_TO_TICKS(200)); }
                if (WiFi.status() == WL_CONNECTED) { wifiDisabledPermanently = false; wifiDownSince = 0; wifiWasDown = true; }
                else { WiFi.disconnect(true, true); WiFi.mode(WIFI_OFF); }
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }

        if (WiFi.status() == WL_CONNECTED) {
            if (wifiWasDown) {
                wifiWasDown = false; wifiDownSince = 0;
                sendTelegramAlert(alertBlock(ICON_SUCCESS, "Hệ thống khôi phục kết nối WiFi"));
                if (!ntpTimeSynced) trySyncNtpTime(); // [FIX-16] tranh thủ đồng bộ giờ khi vừa có mạng lại
            }
            String msg = "";
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (!telegramQueue.empty()) { msg = telegramQueue.front(); telegramQueue.pop(); }
                xSemaphoreGive(telegramQueueMutex);
            }
            if (msg.length() > 0) {
                bool md = (msg.indexOf('*') != -1);
                esp_task_wdt_reset();
                bot.sendMessage(currentChatId, msg, md ? "Markdown" : "");
                esp_task_wdt_reset();
            }
            if (millis() - lastTelegramPoll > TELEGRAM_POLL_MS) {
                esp_task_wdt_reset();
                int n = bot.getUpdates(bot.last_message_received + 1);
                esp_task_wdt_reset();
                if (n > 0) handleNewMessages(n);
                lastTelegramPoll = millis();
            }
        } else {
            wifiWasDown = true; if (wifiDownSince == 0) wifiDownSince = millis();
            if (millis() - wifiDownSince >= WIFI_OFFLINE_FAILSAFE_MS) {
                WiFi.disconnect(true, true);
                WiFi.mode(WIFI_OFF);
                wifiDisabledPermanently = true;
                lastWifiWakeAttempt = millis();
            }
            else if (millis() - lastReconnectAttempt > WIFI_RECONNECT_MS) {
                lastReconnectAttempt = millis(); WiFi.disconnect(); scanAndPickBestWifi(); WiFi.begin(currentSsid.c_str(), currentPass.c_str());
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
    if (slaveM1Online && now - slaveM1LastSeen > SLAVE_TIMEOUT_MS) { slaveM1Online = false; aM1 = true; m1AutoRunning = false; }
    if (slaveM2Online && now - slaveM2LastSeen > SLAVE_TIMEOUT_MS) { slaveM2Online = false; aM2 = true; m2AutoRunning = false; }
    if (slaveHOnline  && now - slaveHLastSeen  > SLAVE_TIMEOUT_MS) { slaveHOnline  = false; aH  = true; }
    portEXIT_CRITICAL(&stateMux);
    // [FIX-18] Trước đây text cảnh báo ghi cứng "(>10p)" trong khi
    // SLAVE_TIMEOUT_MS thực tế = 6.000.000 ms = 100 phút → gây hiểu
    // nhầm khi đọc log/Telegram. Nay hiển thị đúng số phút thật, tự
    // động khớp theo SLAVE_TIMEOUT_MS, không lệch dù sau này đổi hằng số.
    String timeoutMinStr = "(>" + String(SLAVE_TIMEOUT_MS / 60000UL) + "p)";
    if (aM1) sendTelegramAlert(alertBlock(ICON_CRIT, "Hệ thống ngắt LoRa Moong 1 " + timeoutMinStr));
    if (aM2) sendTelegramAlert(alertBlock(ICON_CRIT, "Hệ thống ngắt LoRa Moong 2 " + timeoutMinStr));
    if (aH)  sendTelegramAlert(alertBlock(ICON_CRIT, "Hệ thống ngắt LoRa Trạm Hồ " + timeoutMinStr));
}

void readMasterFloat() {
    uint8_t cur = (digitalRead(FLOAT_MASTER_PIN) == LOW) ? FLOAT_LOW : FLOAT_FULL;
    if (cur != rawMasterFloat) { rawMasterFloat = cur; floatChangeTime = millis(); }
    if (millis() - floatChangeTime >= FLOAT_DEBOUNCE_MS && masterFloat != rawMasterFloat) {
        masterFloat = rawMasterFloat;
        sendTelegramAlert(alertBlock(ICON_INFO, "Phao Master đổi trạng thái", floatText(masterFloat)));
    }
}

void monitorSystemResources() {
    unsigned long now = millis();
    if (now - lastResourceCheck < RESOURCE_MONITOR_MS) return;
    lastResourceCheck = now;

    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < minFreeHeapEver) minFreeHeapEver = freeHeap;

    UBaseType_t loraStackFree = loraTaskHandle ? uxTaskGetStackHighWaterMark(loraTaskHandle) : 0;
    UBaseType_t tgStackFree   = telegramTaskHandle ? uxTaskGetStackHighWaterMark(telegramTaskHandle) : 0;

    Serial.printf("[RES] FreeHeap=%u KB (min=%u KB) | Stack LoRa=%u | Stack TG=%u\n",
                  freeHeap / 1024,
                  (minFreeHeapEver == UINT32_MAX ? freeHeap : minFreeHeapEver) / 1024,
                  (unsigned)loraStackFree, (unsigned)tgStackFree);

    if (freeHeap < HEAP_WARN_THRESHOLD_BYTES) {
        sendTelegramAlert(alertBlock(ICON_WARN, "Cảnh báo bộ nhớ Heap thấp",
                           "Free Heap: " + String(freeHeap / 1024) + " KB\nHệ thống có thể cần khởi động lại."));
    }
    if (loraStackFree > 0 && loraStackFree < 256) {
        sendTelegramAlert(alertBlock(ICON_WARN, "Cảnh báo Stack Task LoRa thấp", "Còn lại: " + String((unsigned)loraStackFree) + " word"));
    }
    if (tgStackFree > 0 && tgStackFree < 256) {
        sendTelegramAlert(alertBlock(ICON_WARN, "Cảnh báo Stack Task Telegram thấp", "Còn lại: " + String((unsigned)tgStackFree) + " word"));
    }
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
        sendTelegramDirect(alertBlock(ICON_INFO, "Khởi động lại định kỳ (bảo trì)",
                            "Đã chạy liên tục " + formatTimeSpan(now) + ". Tất cả bơm đang TẮT, tiến hành khởi động lại an toàn."));
        delay(500);
        ESP.restart();
    }
}

//======================================================
// CORE LOGIC - processControl() với FIX-10, FIX-11, FIX-12
//======================================================
void processControl() {
    unsigned long now = millis();

    // [FIX-12] Snapshot toàn bộ biến volatile một lần duy nhất trong
    // critical section, kể cả masterFloat. Tránh race condition nếu
    // readMasterFloat() cập nhật masterFloat giữa 2 lần đọc trong hàm.
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
    uint8_t curMasterFloat = masterFloat;   // [FIX-12] snapshot ở đây
    portEXIT_CRITICAL(&stateMux);

    // --- Tính trạng thái bơm Moong ---
    bool localPumpM1 = false;
    bool localPumpM2 = false;

    if (currentMode == MODE_AUTO) {
        if (curMasterFloat == FLOAT_LOW) {
            // Tank Master cạn: dừng bơm Moong, ưu tiên bơm H hút nước vào
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

    // --- Tính trạng thái bơm H ---
    bool    targetPumpH  = false;
    uint8_t targetSpeedH = 0;

    if (currentMode == MODE_AUTO) {
        if (curMasterFloat == FLOAT_LOW) {
            // Tank Master cạn: bơm H chạy 100% để hút nước vào tank
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

    // --- Quyết định có gửi lệnh cho Slave H không ---
    //
    // [FIX-10] Thêm điều kiện masterFloatChanged:
    // Kịch bản lỗi gốc: H đang BẬT 100% do M1/M2 cạn, sau đó Master
    // chuyển CẠN. targetSpeedH vẫn = 100, pumpH vẫn = true
    // → logicChanged = FALSE → không gửi lệnh xác nhận mới cho Slave H.
    // Nếu Slave H vừa reset hoặc lệnh cũ bị drop, bơm H sẽ không bật.
    // Fix: khi phao Master đổi trạng thái so với lần gửi lệnh trước,
    // buộc gửi lệnh lại dù payload không thay đổi.
    bool masterFloatChanged = (curMasterFloat != lastMasterFloatSent);
    bool logicChanged       = (targetPumpH != pumpH)
                           || (targetSpeedH != lastSpeedH)
                           || masterFloatChanged;
    bool timeoutResync      = (now - lastPumpHCmdTime > PUMP_H_RESYNC_MS);

    if (logicChanged || timeoutResync) {
        // [FIX-11] lastPumpHCmdTime và lastMasterFloatSent CHỈ cập nhật
        // khi queueCommand thành công. Trước đây cập nhật kể cả khi thất
        // bại → phải chờ PUMP_H_RESYNC_MS (60s) mới retry thay vì retry
        // ngay chu kỳ loop 20ms tiếp theo.
        if (queueCommand(SLAVE_H_ID, targetSpeedH)) {
            lastPumpHCmdTime    = now;
            lastMasterFloatSent = curMasterFloat;  // ghi nhận phao tại lúc gửi thành công
            portENTER_CRITICAL(&stateMux);
            pumpH  = targetPumpH;
            speedH = targetSpeedH;
            portEXIT_CRITICAL(&stateMux);
            lastSpeedH = targetSpeedH;
        } else {
            // Queue thất bại: KHÔNG cập nhật lastPumpHCmdTime, KHÔNG cập nhật
            // lastMasterFloatSent → vòng lặp 20ms sau sẽ thử lại ngay lập tức.
            Serial.printf("[CTRL] queueCommand H thất bại! targetPumpH=%d speed=%d heap=%u\n",
                          targetPumpH, targetSpeedH, ESP.getFreeHeap());
        }
    }

    // --- [FIX] Giám sát xác nhận relay Trạm Hồ (tiếp điểm phụ VẬT LÝ) ---
    // So sánh trạng thái Master RA LỆNH (`pumpH`) với trạng thái relay
    // THẬT slave H đọc từ tiếp điểm phụ (`slaveHRelayConfirmed`). Cho một
    // khoảng ân hạn để ACK/retry, relay cơ khí đóng/mở, và debounce phía
    // slave kịp hoàn tất trước khi coi là lệch thật.
    bool relayHMismatchNow = slaveHRelayHasData
                          && (pumpH != slaveHRelayConfirmed)
                          && (now - lastPumpHCmdTime > RELAY_H_CONFIRM_GRACE_MS);

    if (relayHMismatchNow && !relayHMismatchAlerted) {
        relayHMismatchAlerted = true;
        sendTelegramAlert(alertBlock(ICON_WARN, "Trạm Hồ: relay chưa xác nhận đúng trạng thái",
            "Master đã ra lệnh relay " + String(pumpH ? "BẬT" : "TẮT")
            + " nhưng tiếp điểm phụ tại Trạm Hồ báo lại đang " + String(slaveHRelayConfirmed ? "BẬT" : "TẮT") + ".\n"
            + "Khả năng: relay kẹt tiếp điểm, cháy cuộn hút, đứt dây feedback, mất gói ACK, hoặc Slave H reset/treo.\n"
            + "(Đây là xác nhận từ tiếp điểm phụ vật lý tại chỗ.)"));
    } else if (!relayHMismatchNow && relayHMismatchAlerted) {
        relayHMismatchAlerted = false; // đã khớp lại → cho phép cảnh báo lần sau nếu tái diễn
        sendTelegramAlert(alertBlock(ICON_OK, "Trạm Hồ: relay đã xác nhận khớp trạng thái", ""));
    }

    // --- Thông báo Telegram khi trạng thái bơm thay đổi ---
    if (localPumpM1 != lastPumpM1 || localPumpM2 != lastPumpM2
        || targetPumpH != lastPumpH || targetSpeedH != lastNotifiedSpeedH) {
        String detail = " ▪️ Moong 1: " + pumpBadge(localPumpM1)
                      + "\n ▪️ Moong 2: " + pumpBadge(localPumpM2)
                      + "\n ▪️ Trạm Hồ: " + pumpBadge(targetPumpH)
                      + (targetPumpH ? " (" + String(targetSpeedH) + "%)" : "");
        sendTelegramAlert(alertBlock(ICON_INFO, "Cập nhật trạng thái bơm", detail));
        lastPumpM1 = localPumpM1; lastPumpM2 = localPumpM2;
        lastPumpH = targetPumpH; lastNotifiedSpeedH = targetSpeedH;
        cachedStatusShort = ""; cachedStatusFull = "";
    }

    // --- Cập nhật relay vật lý ---
    portENTER_CRITICAL(&stateMux);
    pumpM1 = localPumpM1;
    pumpM2 = localPumpM2;
    portEXIT_CRITICAL(&stateMux);

    digitalWrite(RELAY_M1_PIN, localPumpM1 ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_M2_PIN, localPumpM2 ? RELAY_ON : RELAY_OFF);
}

//======================================================
// MAIN SETUP & LOOP
//======================================================
void setup() {
    // [FIX-16] GHI CHÚ QUAN TRỌNG: KHÔNG tắt Brown-Out Detector (BOD)
    // của ESP32 nữa. Dòng WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0)
    // trước đây từng nằm ở đây đã bị GỠ BỎ.
    //
    // Lý do: hệ thống này liên tục đóng/ngắt relay bơm công suất lớn
    // (RELAY_M1_PIN, RELAY_M2_PIN) — dòng khởi động (inrush) của relay
    // và động cơ bơm rất dễ gây sụt áp tức thời trên đường nguồn nuôi
    // ESP32. Khi BOD bị tắt, những cú sụt áp này KHÔNG kích hoạt reset
    // sạch của chip mà khiến chip rơi vào trạng thái điện áp "lấp lửng":
    // logic chạy sai, ngắt (interrupt) có thể bị khóa cứng trên cả 2
    // core → lúc đó Task Watchdog (dựa vào ngắt để kích hoạt) CŨNG
    // không cứu được → hệ thống treo hoàn toàn, chỉ ấn nút RST vật lý
    // (cắt nguồn hoàn toàn qua chân EN) mới thoát ra được.
    //
    // Khi để BOD hoạt động bình thường (mặc định của ESP32), mỗi lần
    // sụt áp sẽ kích hoạt một reset phần cứng mức thấp — về hành vi
    // GẦN NHƯ TƯƠNG ĐƯƠNG với việc nhấn nút reset vật lý (setup() chạy
    // lại từ đầu, mọi ngoại vi được khởi tạo lại sạch sẽ) thay vì treo
    // cứng cần can thiệp tay.
    Serial.begin(115200);
    delay(100);

    logResetReason();

    randomSeed(esp_random());
    telegramQueueMutex = xSemaphoreCreateMutex();
    loraMutex          = xSemaphoreCreateMutex();
    coreSyncMutex      = xSemaphoreCreateMutex();

    for (int i = 0; i < QUEUE_SIZE; i++) pendingTable[i].used = false;
    for (int i = 0; i < ALERT_LOG_SIZE; i++) alertLog[i] = "";

    loadWifiCredentials();
    loadTelegramCredentials();

    configPrefs.begin("pump_cfg", true);
    telegramSpeed = configPrefs.getInt("h_speed", 100);
    configPrefs.end();

    // [FIX-13] Nạp lại trạng thái phao slave đã lưu trước khi reboot,
    // thay vì mặc định "Đầy". Nếu chưa từng lưu (lần đầu flash), giữ
    // FLOAT_UNKNOWN — [FIX-14] sẽ cảnh báo cho admin thay vì im lặng.
    floatPrefs.begin("float_cfg", true);
    uint8_t savedM1Float = floatPrefs.getUChar("m1", FLOAT_UNKNOWN);
    uint8_t savedM2Float = floatPrefs.getUChar("m2", FLOAT_UNKNOWN);
    floatPrefs.end();
    slaveM1Float = rawSlaveM1Float = savedM1Float;
    slaveM2Float = rawSlaveM2Float = savedM2Float;

    bot.updateToken(currentBotToken);

    initGPIO();
    initLoRa();
    setupWiFiSingle();
    initWatchdog();

    esp_task_wdt_add(NULL);

    bot.longPoll = 0;
    unsigned long ms = millis();
    slaveM1LastSeen = ms; slaveM2LastSeen = ms; slaveHLastSeen = ms; bootTime = ms;
    lastResourceCheck = ms; lastAutoRebootCheck = ms; lastDailyRebootCheck = ms; lastNtpSyncAttempt = ms;

    // lastMasterFloatSent = FLOAT_FULL (đã khởi tạo khi khai báo)
    // → lần đầu phao cạn sẽ luôn trigger gửi lệnh H

    xTaskCreatePinnedToCore(vLoRaRealtimeTask, "LoRaTask",     4096, NULL, 5, &loraTaskHandle,     0);
    xTaskCreatePinnedToCore(vTelegramTask,     "TelegramTask", 8192, NULL, 1, &telegramTaskHandle, 1);
}

void loop() {
    esp_task_wdt_reset();
    checkSlaveTimeout();
    readMasterFloat();
    debounceSlaveFloats();
    checkFloatUnknownAlert();
    processControl();
    monitorSystemResources();
    checkScheduledReboot();
    checkDailyReboot();
    vTaskDelay(pdMS_TO_TICKS(20));
}
