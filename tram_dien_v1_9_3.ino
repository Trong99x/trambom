/*****************************************************************
 * TRẠM ĐIỆN - ESP32 + 2x CHINT DTSU666 (RS485/MODBUS) + TELEGRAM
 
 *
 * (Xem chi tiết chân đấu nối, cài đặt công tơ, và toàn bộ hướng dẫn
 * sử dụng trong file HUONG_DAN_TRAM_DIEN.md đi kèm.)
 *****************************************************************/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>      // [PHASE-LINK v1.9.2] server nội bộ trả trạng thái mất pha cho Master
#include <PubSubClient.h>   // [MQTT-DISPLAY v1.9.3] đẩy số liệu cho màn hình LVGL ở nhà
#include <ESPmDNS.h>         // [PHASE-LINK v1.9.2] để Master gọi được qua tramdien.local, không phụ
                              // thuộc IP do DHCP cấp (có thể đổi mỗi lần reconnect/reboot router)
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//======================================================
// FORWARD DECLARATIONS — bắt buộc để Arduino IDE không
// sinh prototype sai khi các hàm dùng struct làm tham số
//======================================================
struct TouSet;
struct EnergyStat;
struct MeterData;
struct AlertState;

//======================================================
// CẤU HÌNH MẠNG & TELEGRAM MẶC ĐỊNH
//======================================================
#define WIFI_SSID_DEFAULT "VuThiMai"
#define WIFI_PASS_DEFAULT "09121958"

#define BOT_TOKEN_DEFAULT "8901149730:AAH97gj5ZhNlyfMBKYCA-QWleCsfx0_Y6_A"
#define CHAT_ID_DEFAULT   "8752050398"   // Chat ID này tự động được nạp làm ADMIN đầu tiên khi lần đầu boot

//======================================================
// OTA UPDATE QUA GITHUB
//======================================================
#define FW_VERSION        "1.9.3"
#define OTA_GITHUB_OWNER  "Trong99x"
#define OTA_GITHUB_REPO   "trambom"
#define OTA_GITHUB_BRANCH "main"
#define OTA_MANIFEST_URL  "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/energy_versions.json"
#define OTA_RAW_BASE_URL  "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/"
#define OTA_HTTP_TIMEOUT_MS 60000UL
#define OTA_MAX_VERSIONS  15
#define OTA_CONFIRM_TIMEOUT_MS 600000UL

//======================================================
// CHÂN RS485 (MODBUS RTU) - UART2
//======================================================
#define RS485_RX_PIN  16
#define RS485_TX_PIN  17
#define RS485_DE_RE_PIN 4
#define RS485_BAUD    9600

#define METER_ADDR_M1  1
#define METER_ADDR_M2  2

//======================================================
// [ALARM] CHÂN CÒI/ĐÈN BÁO ĐỘNG VẬT LÝ TẠI TỦ ĐIỆN
// Dùng relay/buzzer 5V hoặc 12V kích qua relay module (KHÔNG kéo trực
// tiếp còi công suất lớn vào chân GPIO). HIGH = bật còi.
//======================================================
#define BUZZER_PIN 27

//======================================================
// ĐỊA CHỈ THANH GHI MODBUS CỦA CHINT DTSU666
//======================================================
#define REG_BLOCK1_START  0x2006
#define REG_BLOCK1_COUNT  14
#define REG_FREQ          0x2044
#define REG_PF            0x202A   // [PF] PFt - hệ số công suất tổng (float, có thể âm nếu tải dung kháng)
#define REG_ENERGY_TOTAL  0x101E
#define REG_Qt            0x201A   // [FIX v1.9.1] Công suất phản kháng tổng Qt (var) — trước đây SAI ở 0x2026
                                    // (rơi vào vùng chưa công bố, đọc rác). Địa chỉ đúng theo tài liệu CHINT.
#define REG_St            0x2022   // [FIX v1.9.1] Công suất biểu kiến tổng St (VA) — trước đây SAI ở 0x202E
                                    // (đó thực ra là PFb - hệ số công suất pha B, không phải St). Địa chỉ
                                    // 0x2022 suy ra từ khoảng trống giữa Qc(0x2020) và PFt(0x202A) trong bảng
                                    // CHINT — CẦN xác nhận lại bằng số đọc thực tế sau khi nạp firmware.
// [FIX v1.9.1] REG_ENERGY_REACT (điện năng phản kháng cộng dồn) đã bị XOÁ.
// Địa chỉ cũ 0x1026 thực ra là NetImpEp (điện năng TÁC DỤNG net, kWh), không phải kvarh — dữ liệu
// hiển thị trước đây hoàn toàn sai. Chưa xác định được địa chỉ đúng của điện năng phản kháng tổng
// trong tài liệu CHINT công khai (cần liên hệ hãng lấy "detailed communication protocol").
// TODO: xác định đúng thanh ghi rồi bật lại tính năng này.

// [FIX v1.9.1] Hệ số quy đổi (scale factor) CHINT quy định cho từng thanh ghi — trước đây bị THIẾU,
// khiến toàn bộ số liệu hiển thị gấp/lệch nhiều lần so với thực tế (VD Ua=2070V thay vì 207.0V,
// Freq=5016Hz thay vì 50.16Hz, PF=1000.00 thay vì 1.000).
#define SCALE_VOLTAGE   0.1f     // Ua, Ub, Uc (V)
#define SCALE_CURRENT   0.001f   // Ia, Ib, Ic (A)
#define SCALE_POWER     0.1f     // Pt, Qt, St (W / var / VA)
#define SCALE_PF        0.001f   // PFt
#define SCALE_FREQ      0.01f    // Freq (Hz)

//======================================================
// TIMING HỆ THỐNG
//======================================================
#define METER_POLL_MS            10000UL
#define TELEGRAM_POLL_MS          7000UL
#define WDT_TIMEOUT_SEC              60

//======================================================
// [WATCHDOG][SELF-HEAL] Ngưỡng tự phục hồi khi mất mạng/lỗi nghiêm trọng
//======================================================
#define NET_DOWN_HARD_RESET_MS    (10UL * 60UL * 1000UL)   // mất mạng liên tục 10 phút -> tự khởi động lại ESP32
// (Đã bỏ NET_DOWN_RESET_RETRY_MS / NET_DOWN_ESP_RESTART_MS — xem [BUGFIX]
// ở checkNetworkHealth(): giờ chỉ còn đúng 1 ngưỡng NET_DOWN_HARD_RESET_MS.)
#define HEAP_CRITICAL_BYTES        20000UL                 // RAM khả dụng dưới ngưỡng này -> chủ động restart phòng crash

//======================================================
// [HEAP-TREND] Theo dõi xu hướng RAM giảm dần (nghi rò rỉ bộ nhớ)
//======================================================
#define HEAP_SAMPLE_INTERVAL_MS   (60UL * 60UL * 1000UL)   // lấy mẫu heap mỗi 1 giờ
#define HEAP_SAMPLE_COUNT         24                        // giữ 24 mẫu gần nhất (~1 ngày)
#define HEAP_TREND_DROP_BYTES     15000UL                   // heap giảm >= mức này liên tục 24h -> cảnh báo nghi rò rỉ

//======================================================
// [ANTI-SPAM] Giới hạn tốc độ gửi Telegram + gộp tin trùng lặp
//======================================================
#define TELEGRAM_MAX_MSG_PER_MIN  15      // tối đa số tin gửi ra mỗi phút
#define TELEGRAM_DEDUPE_MS        120000UL // 2 phút — gộp tin có nội dung giống hệt gửi liên tiếp trong khoảng này
#define TELEGRAM_HTTP_TIMEOUT_SEC    10
#define ENERGY_CHECKPOINT_MS     600000UL   // [NVS-SAFE] 10 phút (trước là 5 phút) — giảm hao mòn flash về lâu dài
#define ALERT_DEBOUNCE_MS         15000UL
#define ALERT_REPEAT_MS          1800000UL   // 30 phút - nhịp nhắc lại bình thường
#define ALERT_ESCALATE_AFTER_MS  3600000UL   // [ESCALATE] Sau 1 giờ vẫn còn lỗi -> tăng tần suất nhắc
#define ALERT_REPEAT_ESCALATED_MS 900000UL   // [ESCALATE] 15 phút/lần khi đã kéo dài >1 giờ
#define WIFI_RECONNECT_MS         30000UL
#define BOOT_GRACE_MS             60000UL
#define DAILY_REPORT_WINDOW_MIN       5      // [REPORT] cửa sổ phút để không lỡ mốc giờ báo cáo

//======================================================
// NGƯỠNG CẢNH BÁO MẶC ĐỊNH (chỉnh được qua Telegram, lưu NVS)
//======================================================
#define DEFAULT_U_MIN   180.0f
#define DEFAULT_U_MAX   250.0f
#define DEFAULT_I_MAX_M1 250.0f
#define DEFAULT_I_MAX_M2 250.0f
#define PHASE_LOSS_V     50.0f   // [PHASE] Điện áp pha dưới mức này coi là MẤT PHA (không phải chỉ thấp áp)
#define DEFAULT_IMBALANCE_PCT 20.0f   // [IMBAL] % lệch dòng cho phép trước khi cảnh báo
#define IMBALANCE_MIN_CURRENT_A 3.0f  // [IMBAL] Chỉ xét lệch pha khi dòng trung bình >= giá trị này (đang có tải)
#define DEFAULT_PF_MIN   0.70f        // [PF] Hệ số công suất dưới mức này (trị tuyệt đối) coi là thấp
#define PF_MIN_POWER_W   500.0f       // [PF] Chỉ xét PF khi công suất đủ lớn, tránh báo nhiễu lúc gần không tải
// [DOT-STATUS v1.9.1] Ngưỡng hợp lệ của tần số theo dải đo của DTSU666 (45-65Hz) — dùng để tô
// chấm đỏ khi Freq đọc về vô lý (VD lỗi thanh ghi/CRC), KHÔNG phải ngưỡng chất lượng điện lưới.
#define FREQ_SANE_MIN    45.0f
#define FREQ_SANE_MAX    65.0f
// [DOT-STATUS v1.9.1] Dung sai khi kiểm tra "tam giác công suất" S >= |P| và S >= |Q| (VA/W/var).
// Nếu S nhỏ hơn |P| hoặc |Q| quá mức dung sai này -> số liệu P/Q/S không nhất quán về vật lý,
// khả năng cao đang đọc sai thanh ghi (như trường hợp St từng trỏ nhầm sang PFb).
#define POWER_TRIANGLE_TOLERANCE_W  1.0f
#define DEFAULT_REPORT_HOUR 10        // [REPORT] Giờ gửi báo cáo hằng ngày mặc định (0-23)
#define DEFAULT_REBOOT_HOUR   0        // [REBOOT] Giờ tự khởi động lại hằng ngày mặc định (0-23) - 0 = 00h00
#define DEFAULT_REBOOT_ENABLED true    // [REBOOT] Mặc định BẬT tự khởi động lại hằng ngày
#define DEFAULT_SPIKE_MULT 1.6f       // [SPIKE] Bội số so với trung bình động để coi là "đột biến"
#define SPIKE_MIN_SAMPLES  3          // [SPIKE] Cần ít nhất bấy nhiêu ngày dữ liệu mới bắt đầu so sánh

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
#define ICON_BOLT    "⚡"

//======================================================
// BIT CẢNH BÁO NỘI BỘ (dùng để bật còi vật lý tại tủ điện — xem
// physicalAlarmActive). EBIT_OTHER dùng chung cho mất pha/lệch
// pha/PF thấp.
//======================================================
#define EBIT_M1_OFFLINE   0x01
#define EBIT_M2_OFFLINE   0x02
#define EBIT_M1_VOLT      0x04
#define EBIT_M2_VOLT      0x08
#define EBIT_M1_CURRENT   0x10
#define EBIT_M2_CURRENT   0x20
#define EBIT_GRID_DOWN    0x40
#define EBIT_OTHER        0x80   // [PHASE][IMBAL][PF] cảnh báo phụ (xem trên) đang hoạt động

//======================================================
// ĐỐI TƯỢNG TOÀN CỤC
//======================================================
Preferences          wifiPrefs, tgPrefs, alertCfgPrefs, energyPrefs, tariffPrefs, usersPrefs, netPrefs;

// [NETMGR] Trạng thái đường truyền Internet đang dùng làm default route.
// Trạm giờ chỉ còn 1 đường uplink duy nhất là WiFi STA (đã xoá hoàn toàn
// 4G) — giữ lại enum để phần còn lại của code (so sánh currentNet ==
// NET_WIFI/NET_NONE) không phải sửa nhiều chỗ.
enum NetMode { NET_NONE = 0, NET_WIFI = 2 };
NetMode currentNet = NET_NONE;

WiFiClientSecure       secured_client;       // Client SSL dùng cho Telegram/HTTP qua WiFi STA

String                currentBotToken = BOT_TOKEN_DEFAULT;
String                currentChatId   = CHAT_ID_DEFAULT;   // giữ lại làm fallback nếu chưa nạp được danh sách user
UniversalTelegramBot  bot(BOT_TOKEN_DEFAULT, secured_client);

// [WATCHDOG][SELF-HEAL]
unsigned long netDownSince       = 0;   // 0 = đang có mạng / chưa từng mất kể từ lần cuối lên mạng
int           netRestartCount = 0;      // đếm lũy kế số lần đã tự restart do mất mạng quá lâu (lưu NVS, hiện trong /status)

// [WIFI-MULTI] Danh sách nhiều mạng WiFi đã lưu, giống hệt cơ chế bên
// Master (master_v15_15.ino) — trước đây bản này chỉ lưu ĐÚNG 1 cặp
// ssid/pass trong NVS, nên mỗi lần /wifi_set là GHI ĐÈ, mất luôn WiFi
// cũ đã lưu trước đó. Nay chuyển sang danh sách tối đa
// MAX_WIFI_NETWORKS mạng, thêm mới không xoá mạng cũ, và khi kết nối
// tự quét sóng để chọn mạng có tín hiệu mạnh nhất trong số các mạng
// đã lưu (scanAndPickBestWifi()).
#define MAX_WIFI_NETWORKS 5
struct WifiCred {
    String ssid;
    String pass;
};
WifiCred savedWifiList[MAX_WIFI_NETWORKS];
int      savedWifiCount = 0;

String  currentSsid = WIFI_SSID_DEFAULT;
String  currentPass = WIFI_PASS_DEFAULT;

HardwareSerial rs485Serial(2);
ModbusMaster   modbusM1;
ModbusMaster   modbusM2;

unsigned long lastMeterPoll   = 0;
unsigned long lastTelegramPoll = 0;
unsigned long lastEnergyCheckpoint = 0;
unsigned long bootTime = 0;
bool          otaRunning = false;

//======================================================
// [MULTIUSER] QUẢN LÝ NGƯỜI DÙNG (ADMIN / USER) - giống cơ chế Master
//======================================================
#define MAX_USERS   10
#define ROLE_USER   0
#define ROLE_ADMIN  1
String  userIds[MAX_USERS];
uint8_t userRoles[MAX_USERS];
int     userCount = 0;

int findUserIndex(const String &chatId) {
    for (int i = 0; i < userCount; i++) if (userIds[i] == chatId) return i;
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
        userIds[0] = CHAT_ID_DEFAULT; userRoles[0] = ROLE_ADMIN; userCount = 1;
        saveUsersToNVS();
    }
}
bool addUserToList(const String &chatId, uint8_t role) {
    if (chatId.length() == 0 || findUserIndex(chatId) >= 0 || userCount >= MAX_USERS) return false;
    userIds[userCount] = chatId; userRoles[userCount] = role; userCount++;
    saveUsersToNVS();
    return true;
}
bool removeUserFromList(const String &chatId) {
    int idx = findUserIndex(chatId);
    if (idx < 0) return false;
    for (int i = idx; i < userCount - 1; i++) { userIds[i] = userIds[i+1]; userRoles[i] = userRoles[i+1]; }
    userCount--; saveUsersToNVS();
    return true;
}
String buildUsersListMessage() {
    String m = "👥 *DANH SÁCH NGƯỜI DÙNG TRẠM ĐIỆN*\n";
    for (int i = 0; i < userCount; i++)
        m += (userRoles[i] == ROLE_ADMIN ? "🔑 ADMIN: " : "👤 USER: ") + String("`") + userIds[i] + "`\n";
    m += "\nTổng: " + String(userCount) + "/" + String(MAX_USERS);
    return m;
}

//======================================================
// DỮ LIỆU CÔNG TƠ (RAM)
//======================================================
struct MeterData {
    float Ua = 0, Ub = 0, Uc = 0;
    float Ia = 0, Ib = 0, Ic = 0;
    float Pt = 0;
    float Qt = 0;              // Công suất phản kháng tổng (var)
    float St = 0;              // Công suất biểu kiến tổng (VA)
    float Freq = 0;
    float PFt = 1.0f;          // [PF]
    float EnergyTotal = 0;
    // [FIX v1.9.1] EnergyReactive đã bỏ — xem TODO ở REG_ENERGY_REACT phía trên.
    bool  online = false;
    unsigned long lastOkMs = 0;
    uint8_t consecutiveFail = 0;
};
MeterData m1, m2;

//======================================================
// [PHASE-LINK v1.9.2] SERVER NỘI BỘ BÁO MẤT PHA CHO MASTER
// Master (chạy chung mạng WiFi) gọi GET http://tramdien.local/phase_status
// mỗi vài giây để biết có đang mất pha hay không -> nếu có, Master tự ép
// TẮT toàn bộ bơm để bảo vệ động cơ. Trạm điện CHỈ trả lời (server), không
// tự quyết định gì phía bơm — an toàn kể cả khi 2 firmware độc lập nhau.
//======================================================
WebServer phaseServer(80);
volatile bool anyPhaseLossGlobal = false;   // cập nhật trong checkAlerts(), đọc trong handlePhaseStatus()
volatile bool m1PhaseLossGlobal  = false;
volatile bool m2PhaseLossGlobal  = false;

//======================================================
// THỐNG KÊ SỐ ĐIỆN NGÀY/THÁNG/NĂM + [SPIKE] TRUNG BÌNH ĐỘNG
//======================================================
// [TOU] Nhóm 3 giá trị kWh theo 3 khung giờ Bình thường/Thấp điểm/Cao
// điểm — khai báo TRƯỚC EnergyStat vì EnergyStat dùng kiểu này làm trường.
struct TouSet { double bt = 0, td = 0, cd = 0; };

struct EnergyStat {
    double  dayStartKwh = -1;
    int     dayMarker   = -1;
    double  monthStartKwh = -1;
    int     monthMarker = -1;
    double  yearStartKwh = -1;
    int     yearMarker  = -1;
    double  yesterdayKwh = 0;
    double  lastMonthKwh = 0;
    double  lastYearKwh  = 0;
    double  dailyAvgKwh  = -1;   // [SPIKE] trung bình động (EMA) kWh/ngày
    int     avgSampleCount = 0;  // [SPIKE] số ngày đã học được
    // [TOU] Tích luỹ kWh theo khung giờ — dayTou/monthTou cộng dồn dần
    // mỗi chu kỳ đọc công tơ; yestTou/lastMonthTou là ảnh chụp lúc chốt sổ.
    TouSet  dayTou, monthTou, yestTou, lastMonthTou;
};
EnergyStat statM1, statM2;
bool ntpSynced = false;
int  lastReportSentDay = -1;    // [REPORT]
int  lastRebootDay     = -1;    // [REBOOT] yday của lần tự khởi động lại gần nhất, tránh reboot 2 lần/ngày

//======================================================
// NGƯỠNG CẢNH BÁO (chỉnh được, lưu NVS)
//======================================================
float alertUMin = DEFAULT_U_MIN;
float alertUMax = DEFAULT_U_MAX;
float alertIMaxM1 = DEFAULT_I_MAX_M1;
float alertIMaxM2 = DEFAULT_I_MAX_M2;
float alertImbalancePct = DEFAULT_IMBALANCE_PCT;   // [IMBAL]
float alertPFMin = DEFAULT_PF_MIN;                 // [PF]
int   dailyReportHour = DEFAULT_REPORT_HOUR;        // [REPORT]
int   dailyRebootHour    = DEFAULT_REBOOT_HOUR;     // [REBOOT]
bool  dailyRebootEnabled = DEFAULT_REBOOT_ENABLED;  // [REBOOT]
float spikeMultiplier = DEFAULT_SPIKE_MULT;         // [SPIKE]

//======================================================
// [TOU] GIÁ ĐIỆN SẢN XUẤT 3 PHA — THEO KHUNG GIỜ, KHÔNG PHẢI BẬC THANG
// Mặc định dưới đây chỉ là VÍ DỤ tham khảo mức hạ áp (<6kV), theo tra
// cứu tại thời điểm viết firmware (giữa 2026), CHƯA gồm VAT — PHẢI tự
// kiểm tra lại và chỉnh bằng /set_gia cho đúng cấp điện áp + hợp đồng
// mua điện thực tế, vì giá có thể đã thay đổi hoặc khác theo khu vực.
//======================================================
#define DEFAULT_PRICE_BT   1811.0   // Giờ Bình Thường (đ/kWh, ví dụ hạ áp <6kV, chưa VAT)
#define DEFAULT_PRICE_TD   1146.0   // Giờ Thấp Điểm
#define DEFAULT_PRICE_CD   3266.0   // Giờ Cao Điểm
#define DEFAULT_VAT_PCT       8.0   // VAT mặc định 8%
// Khung giờ mặc định theo Quyết định 963/QĐ-BCT (hiệu lực 22/4/2026):
// Thấp điểm 00:00-06:00 MỌI ngày; Cao điểm 17:30-22:30 Thứ 2 - Thứ 7
// (Chủ nhật KHÔNG có cao điểm); còn lại là Bình thường.
#define DEFAULT_TOU_LOW_START_MIN   0     // 00:00
#define DEFAULT_TOU_LOW_END_MIN   360     // 06:00
#define DEFAULT_TOU_PEAK_START_MIN 1050   // 17:30
#define DEFAULT_TOU_PEAK_END_MIN  1350    // 22:30
#define DEFAULT_TOU_SUNDAY_HAS_PEAK false
// [TOU] Bộ 3 giá hiện hành (đ/kWh, CHƯA VAT) + % VAT, chỉnh bằng /set_gia, /set_vat.
double priceBT = DEFAULT_PRICE_BT, priceTD = DEFAULT_PRICE_TD, priceCD = DEFAULT_PRICE_CD;
double vatPercent = DEFAULT_VAT_PCT;

// [TOU] Ranh giới khung giờ (tính bằng phút trong ngày, 0-1439), chỉnh
// bằng /set_tou nếu quy định nhà nước thay đổi lịch khung giờ.
int  touLowStartMin  = DEFAULT_TOU_LOW_START_MIN;
int  touLowEndMin    = DEFAULT_TOU_LOW_END_MIN;
int  touPeakStartMin = DEFAULT_TOU_PEAK_START_MIN;
int  touPeakEndMin   = DEFAULT_TOU_PEAK_END_MIN;
bool touSundayHasPeak = DEFAULT_TOU_SUNDAY_HAS_PEAK;

void loadTariffFromNVS() {
    tariffPrefs.begin("tariff_cfg", true);
    priceBT = tariffPrefs.getDouble("priceBT", DEFAULT_PRICE_BT);
    priceTD = tariffPrefs.getDouble("priceTD", DEFAULT_PRICE_TD);
    priceCD = tariffPrefs.getDouble("priceCD", DEFAULT_PRICE_CD);
    vatPercent = tariffPrefs.getDouble("vat", DEFAULT_VAT_PCT);
    touLowStartMin  = tariffPrefs.getInt("touLS", DEFAULT_TOU_LOW_START_MIN);
    touLowEndMin    = tariffPrefs.getInt("touLE", DEFAULT_TOU_LOW_END_MIN);
    touPeakStartMin = tariffPrefs.getInt("touPS", DEFAULT_TOU_PEAK_START_MIN);
    touPeakEndMin   = tariffPrefs.getInt("touPE", DEFAULT_TOU_PEAK_END_MIN);
    touSundayHasPeak= tariffPrefs.getBool("touSunPeak", DEFAULT_TOU_SUNDAY_HAS_PEAK);
    tariffPrefs.end();
}
void saveTariffToNVS() {
    tariffPrefs.begin("tariff_cfg", false);
    tariffPrefs.putDouble("priceBT", priceBT);
    tariffPrefs.putDouble("priceTD", priceTD);
    tariffPrefs.putDouble("priceCD", priceCD);
    tariffPrefs.putDouble("vat", vatPercent);
    tariffPrefs.putInt("touLS", touLowStartMin);
    tariffPrefs.putInt("touLE", touLowEndMin);
    tariffPrefs.putInt("touPS", touPeakStartMin);
    tariffPrefs.putInt("touPE", touPeakEndMin);
    tariffPrefs.putBool("touSunPeak", touSundayHasPeak);
    tariffPrefs.end();
}

// [TOU] Trả về khung giờ hiện tại: 0=Bình thường, 1=Thấp điểm, 2=Cao điểm.
int getTouBand(struct tm &t) {
    int hm = t.tm_hour * 60 + t.tm_min;
    bool sunday = (t.tm_wday == 0);
    if (hm >= touLowStartMin && hm < touLowEndMin) return 1;
    if (sunday && !touSundayHasPeak) return 0;
    if (hm >= touPeakStartMin && hm < touPeakEndMin) return 2;
    return 0;
}
String touBandName(int band) { return band == 1 ? "Thấp điểm" : (band == 2 ? "Cao điểm" : "Bình thường"); }

// [TOU] Tổng kWh và tiền ước tính từ 1 bộ TouSet (dùng chung cho ngày/tháng/kỳ trước).
double touTotal(TouSet &t) { return t.bt + t.td + t.cd; }
double computeTouCost(TouSet &t) {
    double cost = t.bt * priceBT + t.td * priceTD + t.cd * priceCD;
    return cost * (1.0 + vatPercent / 100.0);
}
// [FIX v1.3.2] Định dạng "Hh MM" với phút LUÔN 2 chữ số (trước đây phút
// 1-9 bị thiếu số 0 phía trước, vd 6h05 từng hiển thị sai thành "6h5").
String hhmmLabel(int minutesOfDay) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%dh%02d", minutesOfDay / 60, minutesOfDay % 60);
    return String(buf);
}
String buildTariffMessage() {
    String m = "💰 *BIỂU GIÁ ĐIỆN SẢN XUẤT ĐANG ÁP DỤNG*\n";
    m += " ▪️ Bình thường: " + String(priceBT, 0) + " đ/kWh\n";
    m += " ▪️ Thấp điểm: " + String(priceTD, 0) + " đ/kWh (" + hhmmLabel(touLowStartMin) + " - " + hhmmLabel(touLowEndMin) + ")\n";
    m += " ▪️ Cao điểm: " + String(priceCD, 0) + " đ/kWh (" + hhmmLabel(touPeakStartMin) + " - " + hhmmLabel(touPeakEndMin) + ", " + (touSundayHasPeak ? "kể cả Chủ nhật" : "trừ Chủ nhật") + ")\n";
    m += " ▪️ VAT: " + String(vatPercent, 1) + "%\n";
    m += "\n⚠️ Giá trên (CHƯA gồm VAT) chỉ là VÍ DỤ tham khảo mức hạ áp <6kV tại thời điểm viết firmware — PHẢI đối chiếu và chỉnh lại bằng /set_gia cho đúng cấp điện áp + hợp đồng mua điện thực tế, vì giá/khung giờ nhà nước có thể đã thay đổi.";
    m += "\nĐây cũng là ước tính dựa trên số điện của từng công tơ phụ (không phải công tơ chính EVN) — số tiền thật có thể khác nếu 2 Moong dùng chung 1 hợp đồng điện với các tải khác.";
    return m;
}


//======================================================
// TRẠNG THÁI CẢNH BÁO HIỆN HÀNH (debounce + [ESCALATE] + chống spam)
//======================================================
struct AlertState {
    bool active = false;
    unsigned long sinceMs = 0;
    unsigned long lastNotifyMs = 0;
    bool notified = false;
};
AlertState alM1Volt, alM2Volt, alM1Cur, alM2Cur, alM1Off, alM2Off;
AlertState alM1Phase, alM2Phase;         // [PHASE]
AlertState alM1Imbalance, alM2Imbalance; // [IMBAL]
AlertState alM1PF, alM2PF;               // [PF]

uint8_t currentAlertBits = 0;
bool    physicalAlarmActive = false;     // [ALARM]

//======================================================
// TIỆN ÍCH CHUNG
//======================================================
String divider() { return String(UI_DIVIDER) + "\n"; }

String alertBlock(const char *icon, const String &title, const String &detail = "") {
    String m = String(icon) + " *" + title + "*";
    if (detail.length() > 0) m += "\n" + detail;
    return m;
}
String escapeMarkdown(const String &raw) {
    String out; out.reserve(raw.length() + 8);
    for (unsigned int i = 0; i < raw.length(); i++) {
        char c = raw.charAt(i);
        if (c == '_' || c == '*' || c == '`' || c == '[') out += '\\';
        out += c;
    }
    return out;
}
// [ESCALATE] Định dạng thời lượng kiểu "Xh YYm" giống Master.
String formatTimeSpan(unsigned long ms) {
    unsigned long s = ms / 1000;
    unsigned long m = s / 60; s %= 60;
    unsigned long h = m / 60; m %= 60;
    char buf[24];
    snprintf(buf, sizeof(buf), "%luh%02lum", h, m);
    return String(buf);
}

// [MULTIUSER] Gửi tới TOÀN BỘ user đã đăng ký, thay vì 1 chat id cố định.
// [1-BOT] Chỉ 1 `bot` duy nhất, dùng thẳng secured_client qua WiFi STA.
// [ANTI-SPAM] Giới hạn tối đa TELEGRAM_MAX_MSG_PER_MIN tin/phút, và gộp
// (bỏ qua) tin có nội dung giống hệt tin gửi gần nhất trong vòng
// TELEGRAM_DEDUPE_MS — tránh dồn dập khi 1 lỗi lặp đi lặp lại liên tục.
String        lastSentMsg;
unsigned long lastSentMsgAt = 0;
unsigned long msgTimestamps[TELEGRAM_MAX_MSG_PER_MIN] = {0};
int           msgTimestampIdx = 0;

void sendTelegramDirect(const String &msg) {
    if (currentNet == NET_NONE) return;   // chưa có mạng nào — gửi cũng thất bại, bỏ qua để tránh treo hàm HTTP

    unsigned long nowMs = millis();
    // Gộp tin trùng lặp nội dung gửi liên tiếp trong khoảng ngắn.
    if (msg == lastSentMsg && (nowMs - lastSentMsgAt) < TELEGRAM_DEDUPE_MS) return;

    // Giới hạn tốc độ: đếm số tin đã gửi trong 60s gần nhất (ring buffer).
    int sentInLastMinute = 0;
    for (int i = 0; i < TELEGRAM_MAX_MSG_PER_MIN; i++) {
        if (msgTimestamps[i] != 0 && (nowMs - msgTimestamps[i]) < 60000UL) sentInLastMinute++;
    }
    if (sentInLastMinute >= TELEGRAM_MAX_MSG_PER_MIN) {
        Serial.println("[ANTI-SPAM] Đã đạt giới hạn " + String(TELEGRAM_MAX_MSG_PER_MIN) + " tin/phút — tạm bỏ qua 1 tin để chống spam.");
        return;
    }
    msgTimestamps[msgTimestampIdx] = nowMs;
    msgTimestampIdx = (msgTimestampIdx + 1) % TELEGRAM_MAX_MSG_PER_MIN;
    lastSentMsg = msg; lastSentMsgAt = nowMs;

    bool md = (msg.indexOf('*') != -1);
    if (userCount == 0) { bot.sendMessage(currentChatId, msg, md ? "Markdown" : ""); return; }
    for (int i = 0; i < userCount; i++) bot.sendMessage(userIds[i], msg, md ? "Markdown" : "");
}

//======================================================
// MODBUS
//======================================================
float regsToFloat(uint16_t hi, uint16_t lo) {
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    float f; memcpy(&f, &raw, sizeof(f));
    return f;
}
void rs485PreTransmit()  { digitalWrite(RS485_DE_RE_PIN, HIGH); delayMicroseconds(50); }
void rs485PostTransmit() { delayMicroseconds(50); digitalWrite(RS485_DE_RE_PIN, LOW); }

bool readMeter(ModbusMaster &mb, MeterData &d) {
    uint8_t r1 = mb.readHoldingRegisters(REG_BLOCK1_START, REG_BLOCK1_COUNT);
    if (r1 != mb.ku8MBSuccess) return false;
    // [FIX v1.9.1] Áp hệ số quy đổi CHINT cho từng đại lượng — trước đây thiếu hoàn toàn.
    d.Ua = regsToFloat(mb.getResponseBuffer(0),  mb.getResponseBuffer(1))  * SCALE_VOLTAGE;
    d.Ub = regsToFloat(mb.getResponseBuffer(2),  mb.getResponseBuffer(3))  * SCALE_VOLTAGE;
    d.Uc = regsToFloat(mb.getResponseBuffer(4),  mb.getResponseBuffer(5))  * SCALE_VOLTAGE;
    d.Ia = regsToFloat(mb.getResponseBuffer(6),  mb.getResponseBuffer(7))  * SCALE_CURRENT;
    d.Ib = regsToFloat(mb.getResponseBuffer(8),  mb.getResponseBuffer(9))  * SCALE_CURRENT;
    d.Ic = regsToFloat(mb.getResponseBuffer(10), mb.getResponseBuffer(11)) * SCALE_CURRENT;
    d.Pt = regsToFloat(mb.getResponseBuffer(12), mb.getResponseBuffer(13)) * SCALE_POWER;

    delay(30);
    uint8_t r2 = mb.readHoldingRegisters(REG_FREQ, 2);
    if (r2 == mb.ku8MBSuccess) d.Freq = regsToFloat(mb.getResponseBuffer(0), mb.getResponseBuffer(1)) * SCALE_FREQ;

    delay(30);
    uint8_t r3 = mb.readHoldingRegisters(REG_PF, 2);   // [PF]
    if (r3 == mb.ku8MBSuccess) d.PFt = regsToFloat(mb.getResponseBuffer(0), mb.getResponseBuffer(1)) * SCALE_PF;

    delay(30);
    uint8_t r4 = mb.readHoldingRegisters(REG_ENERGY_TOTAL, 2);
    if (r4 != mb.ku8MBSuccess) return false;
    d.EnergyTotal = regsToFloat(mb.getResponseBuffer(0), mb.getResponseBuffer(1));  // đã là kWh, không cần nhân

    delay(30);
    uint8_t r5 = mb.readHoldingRegisters(REG_Qt, 2);   // [FIX v1.9.1] địa chỉ đúng 0x201A
    if (r5 == mb.ku8MBSuccess) d.Qt = regsToFloat(mb.getResponseBuffer(0), mb.getResponseBuffer(1)) * SCALE_POWER;

    delay(30);
    uint8_t r6 = mb.readHoldingRegisters(REG_St, 2);   // [FIX v1.9.1] địa chỉ đúng (tạm) 0x2022 — cần xác nhận lại
    if (r6 == mb.ku8MBSuccess) d.St = regsToFloat(mb.getResponseBuffer(0), mb.getResponseBuffer(1)) * SCALE_POWER;

    // [FIX v1.9.1] Đã bỏ đọc REG_ENERGY_REACT — xem TODO ở phần define phía trên.

    d.online = true; d.lastOkMs = millis(); d.consecutiveFail = 0;
    return true;
}
void pollMeters() {
    modbusM1.begin(METER_ADDR_M1, rs485Serial);
    modbusM1.preTransmission(rs485PreTransmit);
    modbusM1.postTransmission(rs485PostTransmit);
    if (!readMeter(modbusM1, m1)) { m1.consecutiveFail++; if (m1.consecutiveFail >= 3) m1.online = false; }
    esp_task_wdt_reset();   // [MODBUS] tránh timeout Modbus kéo dài ở M1 ảnh hưởng ngân sách watchdog trước khi đọc M2
    delay(50);
    modbusM2.begin(METER_ADDR_M2, rs485Serial);
    modbusM2.preTransmission(rs485PreTransmit);
    modbusM2.postTransmission(rs485PostTransmit);
    if (!readMeter(modbusM2, m2)) { m2.consecutiveFail++; if (m2.consecutiveFail >= 3) m2.online = false; }
    esp_task_wdt_reset();   // [MODBUS]
}

//======================================================
// NTP / NGÀY-THÁNG-NĂM
//======================================================
bool getNow(struct tm &out) {
    time_t now = time(nullptr);
    if (now < 1700000000) return false;
    localtime_r(&now, &out);
    return true;
}

//======================================================
// LƯU / NẠP THỐNG KÊ ĐIỆN NĂNG TỪ NVS
//======================================================
// [NVS-SAFE] retryAttempt=true để tránh đệ quy vô hạn nếu bản thân lần ghi
// lại cũng bị lỗi (khi đó chấp nhận, sẽ có cơ hội ghi đúng ở checkpoint kế
// tiếp). Tách riêng saveEnergyStatImpl() (không dùng default argument) để
// tránh lỗi Arduino IDE tự sinh prototype trùng default argument với hàm gốc.
void saveEnergyStatImpl(const char *prefix, EnergyStat &s, bool retryAttempt) {
    energyPrefs.begin("energy_cfg", false);
    energyPrefs.putDouble((String(prefix) + "dS").c_str(), s.dayStartKwh);
    energyPrefs.putInt   ((String(prefix) + "dM").c_str(), s.dayMarker);
    energyPrefs.putDouble((String(prefix) + "mS").c_str(), s.monthStartKwh);
    energyPrefs.putInt   ((String(prefix) + "mM").c_str(), s.monthMarker);
    energyPrefs.putDouble((String(prefix) + "yS").c_str(), s.yearStartKwh);
    energyPrefs.putInt   ((String(prefix) + "yM").c_str(), s.yearMarker);
    energyPrefs.putDouble((String(prefix) + "yd").c_str(), s.yesterdayKwh);
    energyPrefs.putDouble((String(prefix) + "lm").c_str(), s.lastMonthKwh);
    energyPrefs.putDouble((String(prefix) + "ly").c_str(), s.lastYearKwh);
    energyPrefs.putDouble((String(prefix) + "av").c_str(), s.dailyAvgKwh);   // [SPIKE]
    energyPrefs.putInt   ((String(prefix) + "ac").c_str(), s.avgSampleCount);
    // [TOU]
    energyPrefs.putDouble((String(prefix) + "dtb").c_str(), s.dayTou.bt);
    energyPrefs.putDouble((String(prefix) + "dtt").c_str(), s.dayTou.td);
    energyPrefs.putDouble((String(prefix) + "dtc").c_str(), s.dayTou.cd);
    energyPrefs.putDouble((String(prefix) + "mtb").c_str(), s.monthTou.bt);
    energyPrefs.putDouble((String(prefix) + "mtt").c_str(), s.monthTou.td);
    energyPrefs.putDouble((String(prefix) + "mtc").c_str(), s.monthTou.cd);
    energyPrefs.putDouble((String(prefix) + "ytb").c_str(), s.yestTou.bt);
    energyPrefs.putDouble((String(prefix) + "ytt").c_str(), s.yestTou.td);
    energyPrefs.putDouble((String(prefix) + "ytc").c_str(), s.yestTou.cd);
    energyPrefs.putDouble((String(prefix) + "ltb").c_str(), s.lastMonthTou.bt);
    energyPrefs.putDouble((String(prefix) + "ltt").c_str(), s.lastMonthTou.td);
    energyPrefs.putDouble((String(prefix) + "ltc").c_str(), s.lastMonthTou.cd);
    energyPrefs.end();

    // [NVS-SAFE] Đọc lại 2 trường đại diện (tổng kWh trong ngày + BT của
    // tháng) để xác nhận ghi đúng — nếu lệch (nghi mất điện/nhiễu giữa
    // lúc ghi) thì tự ghi lại 1 lần duy nhất.
    if (!retryAttempt) {
        energyPrefs.begin("energy_cfg", true);
        double checkDs  = energyPrefs.getDouble((String(prefix) + "dS").c_str(), NAN);
        double checkMtb = energyPrefs.getDouble((String(prefix) + "mtb").c_str(), NAN);
        energyPrefs.end();
        bool mismatch = isnan(checkDs) || isnan(checkMtb) ||
                         fabs(checkDs - s.dayStartKwh) > 0.001 || fabs(checkMtb - s.monthTou.bt) > 0.001;
        if (mismatch) {
            Serial.println(String("[NVS-SAFE] Phát hiện sai lệch sau khi ghi NVS (") + prefix + ") — ghi lại 1 lần.");
            saveEnergyStatImpl(prefix, s, true);
        }
    }
}
void saveEnergyStat(const char *prefix, EnergyStat &s) { saveEnergyStatImpl(prefix, s, false); }
void loadEnergyStat(const char *prefix, EnergyStat &s) {
    energyPrefs.begin("energy_cfg", true);
    s.dayStartKwh   = energyPrefs.getDouble((String(prefix) + "dS").c_str(), -1);
    s.dayMarker     = energyPrefs.getInt   ((String(prefix) + "dM").c_str(), -1);
    s.monthStartKwh = energyPrefs.getDouble((String(prefix) + "mS").c_str(), -1);
    s.monthMarker   = energyPrefs.getInt   ((String(prefix) + "mM").c_str(), -1);
    s.yearStartKwh  = energyPrefs.getDouble((String(prefix) + "yS").c_str(), -1);
    s.yearMarker    = energyPrefs.getInt   ((String(prefix) + "yM").c_str(), -1);
    s.yesterdayKwh  = energyPrefs.getDouble((String(prefix) + "yd").c_str(), 0);
    s.lastMonthKwh  = energyPrefs.getDouble((String(prefix) + "lm").c_str(), 0);
    s.lastYearKwh   = energyPrefs.getDouble((String(prefix) + "ly").c_str(), 0);
    s.dailyAvgKwh   = energyPrefs.getDouble((String(prefix) + "av").c_str(), -1);
    s.avgSampleCount= energyPrefs.getInt   ((String(prefix) + "ac").c_str(), 0);
    // [TOU]
    s.dayTou.bt      = energyPrefs.getDouble((String(prefix) + "dtb").c_str(), 0);
    s.dayTou.td      = energyPrefs.getDouble((String(prefix) + "dtt").c_str(), 0);
    s.dayTou.cd      = energyPrefs.getDouble((String(prefix) + "dtc").c_str(), 0);
    s.monthTou.bt    = energyPrefs.getDouble((String(prefix) + "mtb").c_str(), 0);
    s.monthTou.td    = energyPrefs.getDouble((String(prefix) + "mtt").c_str(), 0);
    s.monthTou.cd    = energyPrefs.getDouble((String(prefix) + "mtc").c_str(), 0);
    s.yestTou.bt     = energyPrefs.getDouble((String(prefix) + "ytb").c_str(), 0);
    s.yestTou.td     = energyPrefs.getDouble((String(prefix) + "ytt").c_str(), 0);
    s.yestTou.cd     = energyPrefs.getDouble((String(prefix) + "ytc").c_str(), 0);
    s.lastMonthTou.bt= energyPrefs.getDouble((String(prefix) + "ltb").c_str(), 0);
    s.lastMonthTou.td= energyPrefs.getDouble((String(prefix) + "ltt").c_str(), 0);
    s.lastMonthTou.cd= energyPrefs.getDouble((String(prefix) + "ltc").c_str(), 0);
    energyPrefs.end();
}

// [SPIKE] Kiểm tra đột biến so với trung bình động NGAY TRƯỚC khi cập
// nhật EMA bằng giá trị hôm qua vừa chốt — để so sánh "hôm qua" với
// "trung bình các ngày TRƯỚC hôm qua", không tự so với chính nó.
void checkSpikeAndUpdateAvg(EnergyStat &s, const char *moongName) {
    double oldAvg = s.dailyAvgKwh;
    bool haveBaseline = (oldAvg > 0) && (s.avgSampleCount >= SPIKE_MIN_SAMPLES);
    if (haveBaseline && s.yesterdayKwh > oldAvg * spikeMultiplier) {
        sendTelegramDirect(alertBlock(ICON_WARN, String(moongName) + ": điện năng hôm qua TĂNG ĐỘT BIẾN",
            "Hôm qua: " + String(s.yesterdayKwh, 2) + " kWh, so với trung bình gần đây: " + String(oldAvg, 2) + " kWh"
            " (gấp " + String(s.yesterdayKwh / oldAvg, 2) + " lần)."
            "\nKiểm tra rò rỉ điện, thiết bị chạy sai giờ, hoặc tải bất thường."));
    }
    if (oldAvg < 0) s.dailyAvgKwh = s.yesterdayKwh;
    else            s.dailyAvgKwh = oldAvg * 0.7 + s.yesterdayKwh * 0.3;
    s.avgSampleCount++;
}

void updateEnergyRollover(EnergyStat &s, float currentTotalKwh, const char *prefix, const char *moongName, bool &changed) {
    struct tm t;
    if (!getNow(t)) return;
    ntpSynced = true;

    // [FIX v1.3.2] dayMarker trước đây chỉ lưu tm_yday (0-365) — nếu thiết bị
    // offline đúng khoảng thời gian là bội số ~365 ngày, hệ thống có thể hiểu
    // nhầm là "vẫn cùng ngày" và bỏ lỡ rollover. Nay gắn thêm năm vào key,
    // giống cách monthKey/yearKey đã làm, để luôn là giá trị tuyệt đối.
    int dayOfYear = (t.tm_year + 1900) * 1000 + t.tm_yday;
    int monthKey  = (t.tm_year + 1900) * 12 + t.tm_mon;
    int yearKey   = t.tm_year + 1900;

    if (s.dayMarker   == -1) { s.dayMarker = dayOfYear;   s.dayStartKwh   = currentTotalKwh; changed = true; }
    if (s.monthMarker == -1) { s.monthMarker = monthKey;  s.monthStartKwh = currentTotalKwh; changed = true; }
    if (s.yearMarker  == -1) { s.yearMarker = yearKey;    s.yearStartKwh  = currentTotalKwh; changed = true; }

    if (dayOfYear != s.dayMarker) {
        s.yesterdayKwh = currentTotalKwh - s.dayStartKwh;
        s.dayStartKwh  = currentTotalKwh;
        s.dayMarker    = dayOfYear;
        checkSpikeAndUpdateAvg(s, moongName);   // [SPIKE]
        s.yestTou = s.dayTou;                   // [TOU] chốt sổ ngày
        s.dayTou  = TouSet();
        changed = true;
    }
    if (monthKey != s.monthMarker) {
        s.lastMonthKwh  = currentTotalKwh - s.monthStartKwh;
        s.monthStartKwh = currentTotalKwh;
        s.monthMarker   = monthKey;
        s.lastMonthTou = s.monthTou;             // [TOU] chốt sổ tháng
        s.monthTou     = TouSet();
        changed = true;
    }
    if (yearKey != s.yearMarker) {
        s.lastYearKwh  = currentTotalKwh - s.yearStartKwh;
        s.yearStartKwh = currentTotalKwh;
        s.yearMarker   = yearKey;
        changed = true;
    }
    if (changed) saveEnergyStat(prefix, s);
}

double kwhToday(EnergyStat &s, float currentTotalKwh) { double v = (s.dayStartKwh   < 0) ? 0 : currentTotalKwh - s.dayStartKwh;   return v < 0 ? 0 : v; }
double kwhMonth(EnergyStat &s, float currentTotalKwh) { double v = (s.monthStartKwh < 0) ? 0 : currentTotalKwh - s.monthStartKwh; return v < 0 ? 0 : v; }
double kwhYear (EnergyStat &s, float currentTotalKwh) { double v = (s.yearStartKwh  < 0) ? 0 : currentTotalKwh - s.yearStartKwh;  return v < 0 ? 0 : v; }

// [TOU] Mốc điện năng ở lần đọc TRƯỚC (RAM, -1 = chưa có mốc). Mỗi
// chu kỳ đọc công tơ, phần tăng thêm (delta) được cộng vào đúng khung
// giờ hiện tại. Reset về -1 khi công tơ vừa offline rồi online lại
// (tránh cộng nhầm khoảng thời gian không đọc được vào 1 khung giờ).
float lastPollEnergyM1 = -1, lastPollEnergyM2 = -1;

void accumulateTou(EnergyStat &s, float currentTotalKwh, float &lastPollEnergy) {
    if (lastPollEnergy < 0) { lastPollEnergy = currentTotalKwh; return; }
    double delta = currentTotalKwh - lastPollEnergy;
    lastPollEnergy = currentTotalKwh;
    if (delta <= 0 || delta > 50) return;   // <=0: đồng hồ không lùi; >50kWh/10s là bất thường (lỗi đọc) -> bỏ qua
    struct tm t;
    if (!getNow(t)) return;   // chưa có giờ NTP thì chưa thể phân loại khung giờ, bỏ qua chu kỳ này
    int band = getTouBand(t);
    if (band == 1)      { s.dayTou.td += delta; s.monthTou.td += delta; }
    else if (band == 2) { s.dayTou.cd += delta; s.monthTou.cd += delta; }
    else                { s.dayTou.bt += delta; s.monthTou.bt += delta; }
}

//======================================================
// WIFI / TELEGRAM / ALERT CONFIG NVS
//======================================================
// [WIFI-MULTI] Port nguyên bản từ master_v15_15.ino để hành vi WiFi
// giống hệt Master: nhiều mạng, thêm không mất mạng cũ, tự chọn mạng
// mạnh nhất khi kết nối.
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
    currentBotToken = tgPrefs.getString("token", BOT_TOKEN_DEFAULT);
    currentChatId   = tgPrefs.getString("chatid", CHAT_ID_DEFAULT);
    tgPrefs.end();
}
void loadAlertConfig() {
    alertCfgPrefs.begin("alert_cfg", true);
    alertUMin        = alertCfgPrefs.getFloat("uMin", DEFAULT_U_MIN);
    alertUMax        = alertCfgPrefs.getFloat("uMax", DEFAULT_U_MAX);
    alertIMaxM1      = alertCfgPrefs.getFloat("iM1",  DEFAULT_I_MAX_M1);
    alertIMaxM2      = alertCfgPrefs.getFloat("iM2",  DEFAULT_I_MAX_M2);
    alertImbalancePct= alertCfgPrefs.getFloat("imb",  DEFAULT_IMBALANCE_PCT);
    alertPFMin       = alertCfgPrefs.getFloat("pf",   DEFAULT_PF_MIN);
    dailyReportHour  = alertCfgPrefs.getInt  ("rphour",DEFAULT_REPORT_HOUR);
    dailyRebootHour  = alertCfgPrefs.getInt  ("rbhour",DEFAULT_REBOOT_HOUR);      // [REBOOT]
    dailyRebootEnabled = alertCfgPrefs.getBool("rben", DEFAULT_REBOOT_ENABLED);   // [REBOOT]
    spikeMultiplier  = alertCfgPrefs.getFloat("spike",DEFAULT_SPIKE_MULT);
    alertCfgPrefs.end();
}
void saveAlertConfig() {
    alertCfgPrefs.begin("alert_cfg", false);
    alertCfgPrefs.putFloat("uMin", alertUMin);
    alertCfgPrefs.putFloat("uMax", alertUMax);
    alertCfgPrefs.putFloat("iM1",  alertIMaxM1);
    alertCfgPrefs.putFloat("iM2",  alertIMaxM2);
    alertCfgPrefs.putFloat("imb",  alertImbalancePct);
    alertCfgPrefs.putFloat("pf",   alertPFMin);
    alertCfgPrefs.putInt  ("rphour",dailyReportHour);
    alertCfgPrefs.putInt  ("rbhour",dailyRebootHour);      // [REBOOT]
    alertCfgPrefs.putBool ("rben", dailyRebootEnabled);    // [REBOOT]
    alertCfgPrefs.putFloat("spike",spikeMultiplier);
    alertCfgPrefs.end();
}

//======================================================
// WIFI STA CONNECT — đường truyền Internet duy nhất của trạm.
// Không tự cấu hình NTP/gửi tin nhắn ở đây — việc đó do ensureNetwork()
// điều phối chung (xem [NETMGR] bên dưới).
//======================================================
bool connectWiFiSTA(unsigned long totalTimeoutMs) {
    // [WIFI-MULTI] Quét sóng để chọn mạng ĐÃ LƯU có tín hiệu mạnh nhất
    // trước, giống hệt Master — nếu không quét được (lần đầu, hoặc
    // không thấy mạng nào), dùng mạng đầu tiên trong danh sách.
    if (!scanAndPickBestWifi()) Serial.println("[WIFI] Dùng cấu hình mặc định ban đầu.");
    WiFi.begin(currentSsid.c_str(), currentPass.c_str());
    unsigned long t0 = millis();
    unsigned long perTry = min(totalTimeoutMs, 15000UL);
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < perTry) { delay(200); esp_task_wdt_reset(); }

    // [WIFI-MULTI] Nếu mạng được chọn không kết nối được, thử lần lượt
    // các mạng còn lại đã lưu — giống hệt Master.
    if (WiFi.status() != WL_CONNECTED) {
        for (int i = 0; i < savedWifiCount && millis() - t0 < totalTimeoutMs; i++) {
            if (savedWifiList[i].ssid == currentSsid) continue;
            WiFi.disconnect();
            currentSsid = savedWifiList[i].ssid; currentPass = savedWifiList[i].pass;
            WiFi.begin(currentSsid.c_str(), currentPass.c_str());
            unsigned long ts = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - ts < 8000) { delay(200); esp_task_wdt_reset(); }
            if (WiFi.status() == WL_CONNECTED) break;
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        secured_client.setInsecure();
        secured_client.setTimeout(TELEGRAM_HTTP_TIMEOUT_SEC);
        secured_client.setHandshakeTimeout(10);
        return true;
    }
    return false;
}

//======================================================
// [NETMGR] ĐIỀU PHỐI MẠNG — CHỈ CÒN WIFI STA (đã xoá hoàn toàn 4G)
//======================================================
void setupNetwork() {
    // Không phát WiFi AP cục bộ — chỉ dùng WIFI_STA để kết nối ra ngoài.
    WiFi.mode(WIFI_STA);

    Serial.println("[NETMGR] Đang kết nối WiFi STA...");
    if (connectWiFiSTA(15000)) {
        currentNet = NET_WIFI;
        configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
        // [PHASE-LINK v1.9.2] Đăng ký mDNS "tramdien.local" + khởi động server nội bộ để
        // Master gọi GET /phase_status. Chỉ cần làm 1 lần lúc có WiFi lần đầu — server tiếp
        // tục hoạt động qua các lần WiFi reconnect sau đó (không cần gọi lại begin()).
        if (MDNS.begin("tramdien")) {
            Serial.println("[PHASE-LINK] mDNS sẵn sàng: http://tramdien.local/phase_status");
        } else {
            Serial.println("[PHASE-LINK] CẢNH BÁO: khởi động mDNS thất bại — Master cần gọi thẳng bằng IP.");
        }
        phaseServer.on("/phase_status", handlePhaseStatus);
        phaseServer.begin();
        sendTelegramDirect(alertBlock(ICON_SUCCESS, "Trạm Điện đã kết nối (WiFi)", "Firmware: " + escapeMarkdown(String(FW_VERSION)) + "\nĐịa chỉ IP: " + WiFi.localIP().toString() + "\nKhởi động lại lần trước: " + escapeMarkdown(bootReasonText())));
        return;
    }

    currentNet = NET_NONE;
    Serial.println("[NETMGR] Chưa kết nối được WiFi STA — chưa có Internet.");
}

// Gọi mỗi vòng lặp: theo dõi & tự phục hồi kết nối WiFi, không chặn
// (blocking) quá lâu để không ảnh hưởng watchdog/đọc công tơ.
void ensureNetwork() {
    static unsigned long lastTryWifi = 0, lastTimeRetry = 0;

    bool netWifiUp = (WiFi.status() == WL_CONNECTED);

    if (currentNet == NET_WIFI && netWifiUp) {
        netDownSince = 0;
        if (!ntpSynced && millis() - lastTimeRetry >= 30000UL) {
            lastTimeRetry = millis();
            configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
        }
        return;
    }

    // WiFi đã rớt (hoặc chưa từng có) — thử kết nối lại.
    if (!netWifiUp && millis() - lastTryWifi >= WIFI_RECONNECT_MS) {
        lastTryWifi = millis();
        WiFi.disconnect();
        scanAndPickBestWifi();
        WiFi.begin(currentSsid.c_str(), currentPass.c_str());
    }
    if (netWifiUp) {
        bool wasNone = (currentNet == NET_NONE);
        currentNet = NET_WIFI;
        netDownSince = 0;
        secured_client.setInsecure();
        secured_client.setTimeout(TELEGRAM_HTTP_TIMEOUT_SEC);
        secured_client.setHandshakeTimeout(10);
        if (!ntpSynced) configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
        if (wasNone) sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã khôi phục kết nối (WiFi)"));
        return;
    }

    // [WATCHDOG] WiFi STA vẫn chưa lên được — ghi nhận thời điểm bắt đầu
    // mất mạng để checkNetworkHealth() theo dõi và tự phục hồi.
    currentNet = NET_NONE;
    if (netDownSince == 0) netDownSince = millis();
}

//======================================================
// [WATCHDOG][SELF-HEAL] TỰ KHỞI ĐỘNG LẠI ESP32 KHI MẤT MẠNG QUÁ LÂU
// Số lần đã tự restart do mất mạng được lưu NVS để vẫn đếm được cộng dồn
// qua các lần restart (không bị về 0 mỗi lần).
//======================================================
void restartDueToNetworkDown() {
    netRestartCount++;
    netPrefs.begin("net_cfg", false);
    netPrefs.putInt("netRstCnt", netRestartCount);
    netPrefs.end();
    Serial.println("[SELF-HEAL] Mất mạng quá lâu — RESTART ESP32 (lần thứ " + String(netRestartCount) + ").");
    sendTelegramDirect(alertBlock(ICON_WARN, "Mất mạng quá lâu — tự khởi động lại", "Lần thứ " + String(netRestartCount) + " kể từ lúc lắp đặt."));
    delay(500);
    ESP.restart();
}

// Gọi mỗi vòng lặp: theo dõi thời gian mất mạng liên tục để tự khởi động
// lại ESP32 nếu sự cố kéo dài bất thường — tránh phải chờ người vận hành
// ra tận nơi khởi động lại tay.
void checkNetworkHealth() {
    if (netDownSince == 0) return;   // đang có mạng, không có gì phải xử lý
    unsigned long downFor = millis() - netDownSince;
    if (downFor >= NET_DOWN_HARD_RESET_MS) {
        restartDueToNetworkDown();
    }
}

// Gọi mỗi vòng lặp: chủ động restart nếu RAM khả dụng tụt xuống mức nguy
// hiểm (nghi rò rỉ bộ nhớ sau thời gian dài chạy) — restart có kiểm soát
// (kèm cảnh báo) còn hơn để hệ thống crash ngẫu nhiên không rõ nguyên nhân.
void checkHeapHealth() {
    if (ESP.getFreeHeap() >= HEAP_CRITICAL_BYTES) return;
    Serial.println("[SELF-HEAL] RAM khả dụng quá thấp (" + String(ESP.getFreeHeap()) + " bytes) — RESTART ESP32 phòng ngừa.");
    sendTelegramDirect(alertBlock(ICON_WARN, "RAM thấp bất thường — tự khởi động lại", "Free heap: " + String(ESP.getFreeHeap()) + " bytes."));
    delay(500);
    ESP.restart();
}

//======================================================
// [HEAP-TREND] Theo dõi xu hướng RAM giảm dần đều (nghi rò rỉ bộ nhớ)
// Khác checkHeapHealth() (ngưỡng tuyệt đối) — cái này phát hiện SỚM khi
// heap cứ giảm dần đều suốt nhiều giờ, dù chưa chạm ngưỡng nguy hiểm.
//======================================================
uint32_t      heapSamples[HEAP_SAMPLE_COUNT] = {0};
int           heapSampleCount = 0;   // số mẫu đã có (tăng dần tới HEAP_SAMPLE_COUNT)
int           heapSampleIdx   = 0;   // vị trí ghi kế tiếp (ring buffer)
unsigned long lastHeapSampleAt = 0;
bool          heapTrendWarned  = false;   // chỉ cảnh báo 1 lần, tránh spam — reset khi heap phục hồi

void checkHeapTrend() {
    if (millis() - lastHeapSampleAt < HEAP_SAMPLE_INTERVAL_MS && lastHeapSampleAt != 0) return;
    lastHeapSampleAt = millis();

    heapSamples[heapSampleIdx] = ESP.getFreeHeap();
    heapSampleIdx = (heapSampleIdx + 1) % HEAP_SAMPLE_COUNT;
    if (heapSampleCount < HEAP_SAMPLE_COUNT) heapSampleCount++;
    if (heapSampleCount < HEAP_SAMPLE_COUNT) return;   // chưa đủ dữ liệu 1 vòng (~1 ngày)

    // Mẫu cũ nhất trong ring buffer chính là vị trí ghi kế tiếp hiện tại.
    uint32_t oldest = heapSamples[heapSampleIdx];
    uint32_t newest = heapSamples[(heapSampleIdx + HEAP_SAMPLE_COUNT - 1) % HEAP_SAMPLE_COUNT];

    // Kiểm tra xu hướng GIẢM ĐỀU: mỗi mẫu kế tiếp không lớn hơn mẫu trước
    // quá nhiều (cho phép dao động nhỏ do cấp phát/giải phóng String tạm).
    bool monotonicDown = true;
    for (int i = 1; i < HEAP_SAMPLE_COUNT; i++) {
        uint32_t prev = heapSamples[(heapSampleIdx + i - 1) % HEAP_SAMPLE_COUNT];
        uint32_t cur  = heapSamples[(heapSampleIdx + i) % HEAP_SAMPLE_COUNT];
        if (cur > prev + 2000UL) { monotonicDown = false; break; }   // có nhịp phục hồi rõ rệt -> không phải rò rỉ
    }

    if (monotonicDown && oldest > newest && (oldest - newest) >= HEAP_TREND_DROP_BYTES) {
        if (!heapTrendWarned) {
            Serial.println("[HEAP-TREND] Nghi rò rỉ bộ nhớ: heap giảm đều từ " + String(oldest) + " xuống " + String(newest) + " bytes trong ~" + String(HEAP_SAMPLE_COUNT) + " giờ.");
            sendTelegramDirect(alertBlock(ICON_WARN, "Nghi rò rỉ bộ nhớ (heap giảm dần)", "Từ " + String(oldest) + " -> " + String(newest) + " bytes trong ~" + String(HEAP_SAMPLE_COUNT) + " giờ qua.\nHệ thống vẫn tự restart phòng ngừa nếu chạm ngưỡng nguy hiểm."));
            heapTrendWarned = true;
        }
    } else {
        heapTrendWarned = false;   // heap đã phục hồi/ổn định -> reset cờ để lần sau vẫn cảnh báo được nếu tái diễn
    }
}

//======================================================
// [BOOT-LOG] Đọc lý do khởi động lại lần trước, dịch sang tiếng Việt
//======================================================
String bootReasonText() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "Bật nguồn bình thường";
        case ESP_RST_SW:        return "Restart do lệnh phần mềm (VD /reset, OTA, tự reboot hằng ngày)";
        case ESP_RST_PANIC:     return "⚠️ Panic/lỗi phần mềm nghiêm trọng (exception)";
        case ESP_RST_INT_WDT:   return "⚠️ Watchdog nội bộ (interrupt WDT) — có tác vụ bị treo";
        case ESP_RST_TASK_WDT:  return "⚠️ Task Watchdog — vòng lặp chính bị treo quá " + String(WDT_TIMEOUT_SEC) + "s";
        case ESP_RST_WDT:       return "⚠️ Watchdog khác (RTC WDT)";
        case ESP_RST_BROWNOUT:  return "⚠️ Sụt áp nguồn (brown-out) — kiểm tra nguồn cấp cho ESP32";
        case ESP_RST_DEEPSLEEP: return "Thức dậy từ deep sleep";
        default:                return "Không xác định (mã " + String((int)esp_reset_reason()) + ")";
    }
}

//======================================================
// KIỂM TRA NGƯỠNG & CẢNH BÁO (debounce + [ESCALATE])
//======================================================
// Trả về 0 = không cần báo, 1 = báo lỗi mới/nhắc lại, 2 = báo đã khôi phục.
int evalAlert(AlertState &al, bool conditionNow, unsigned long now) {
    if (conditionNow) {
        if (!al.active) { al.active = true; al.sinceMs = now; al.notified = false; }
        unsigned long repeatMs = (now - al.sinceMs > ALERT_ESCALATE_AFTER_MS) ? ALERT_REPEAT_ESCALATED_MS : ALERT_REPEAT_MS;
        if (!al.notified && now - al.sinceMs >= ALERT_DEBOUNCE_MS) { al.notified = true; al.lastNotifyMs = now; return 1; }
        if (al.notified && now - al.lastNotifyMs >= repeatMs)      { al.lastNotifyMs = now; return 1; }
        return 0;
    } else {
        if (al.active && al.notified) { al.active = false; al.notified = false; return 2; }
        al.active = false;
        return 0;
    }
}
// [ESCALATE] Hậu tố thời lượng, chỉ hiển thị nếu đã có thời điểm bắt đầu hợp lệ.
String durationSuffix(AlertState &al, unsigned long now) {
    if (al.sinceMs == 0) return "";
    return "\n⏱ Đã kéo dài: " + formatTimeSpan(now - al.sinceMs);
}

String phaseVoltProblem(MeterData &m, float uMin, float uMax) {
    String s = "";
    if (m.Ua < uMin || m.Ua > uMax) s += "Pha A: " + String(m.Ua, 1) + "V  ";
    if (m.Ub < uMin || m.Ub > uMax) s += "Pha B: " + String(m.Ub, 1) + "V  ";
    if (m.Uc < uMin || m.Uc > uMax) s += "Pha C: " + String(m.Uc, 1) + "V  ";
    return s;
}
// [PHASE] Liệt kê đúng (các) pha đang mất (điện áp < PHASE_LOSS_V).
String phaseLossList(MeterData &m) {
    String s = "";
    if (m.Ua < PHASE_LOSS_V) s += "A ";
    if (m.Ub < PHASE_LOSS_V) s += "B ";
    if (m.Uc < PHASE_LOSS_V) s += "C ";
    return s;
}

void checkAlerts() {
    unsigned long now = millis();
    uint8_t bits = 0;

    bool m1Off = !m1.online, m2Off = !m2.online;

    // [PHASE] Mất pha: ưu tiên kiểm tra trước, TÁCH khỏi "điện áp bất thường" thông thường.
    bool m1PhaseLoss = m1.online && (m1.Ua < PHASE_LOSS_V || m1.Ub < PHASE_LOSS_V || m1.Uc < PHASE_LOSS_V);
    bool m2PhaseLoss = m2.online && (m2.Ua < PHASE_LOSS_V || m2.Ub < PHASE_LOSS_V || m2.Uc < PHASE_LOSS_V);
    // [PHASE-LINK v1.9.2] Cập nhật cờ toàn cục cho handlePhaseStatus() — Master đọc qua HTTP.
    // Lưu ý: chỉ dựa vào công tơ ĐANG ONLINE; công tơ mất kết nối KHÔNG được coi là mất pha
    // (tránh Master ép dừng bơm oan chỉ vì lỗi Modbus/RS485 tạm thời, không phải sự cố điện thật).
    m1PhaseLossGlobal  = m1PhaseLoss;
    m2PhaseLossGlobal  = m2PhaseLoss;
    anyPhaseLossGlobal = m1PhaseLoss || m2PhaseLoss;

    // Điện áp bất thường "thông thường" = ngoài ngưỡng NHƯNG KHÔNG PHẢI mất pha hẳn.
    bool m1VoltBad = m1.online && !m1PhaseLoss && (m1.Ua < alertUMin || m1.Ua > alertUMax || m1.Ub < alertUMin || m1.Ub > alertUMax || m1.Uc < alertUMin || m1.Uc > alertUMax);
    bool m2VoltBad = m2.online && !m2PhaseLoss && (m2.Ua < alertUMin || m2.Ua > alertUMax || m2.Ub < alertUMin || m2.Ub > alertUMax || m2.Uc < alertUMin || m2.Uc > alertUMax);

    bool m1CurBad  = m1.online && (m1.Ia > alertIMaxM1 || m1.Ib > alertIMaxM1 || m1.Ic > alertIMaxM1);
    bool m2CurBad  = m2.online && (m2.Ia > alertIMaxM2 || m2.Ib > alertIMaxM2 || m2.Ic > alertIMaxM2);

    // [IMBAL] Lệch pha / mất cân bằng tải.
    float m1Avg = (m1.Ia + m1.Ib + m1.Ic) / 3.0f;
    float m2Avg = (m2.Ia + m2.Ib + m2.Ic) / 3.0f;
    bool m1Imb = m1.online && m1Avg >= IMBALANCE_MIN_CURRENT_A &&
                 (fabs(m1.Ia - m1Avg) / m1Avg * 100.0f > alertImbalancePct ||
                  fabs(m1.Ib - m1Avg) / m1Avg * 100.0f > alertImbalancePct ||
                  fabs(m1.Ic - m1Avg) / m1Avg * 100.0f > alertImbalancePct);
    bool m2Imb = m2.online && m2Avg >= IMBALANCE_MIN_CURRENT_A &&
                 (fabs(m2.Ia - m2Avg) / m2Avg * 100.0f > alertImbalancePct ||
                  fabs(m2.Ib - m2Avg) / m2Avg * 100.0f > alertImbalancePct ||
                  fabs(m2.Ic - m2Avg) / m2Avg * 100.0f > alertImbalancePct);

    // [PF] Hệ số công suất thấp, chỉ xét khi có tải đáng kể.
    bool m1PFBad = m1.online && fabs(m1.Pt) > PF_MIN_POWER_W && fabs(m1.PFt) < alertPFMin;
    bool m2PFBad = m2.online && fabs(m2.Pt) > PF_MIN_POWER_W && fabs(m2.PFt) < alertPFMin;

    int r;
    r = evalAlert(alM1Off, m1Off, now);
    if (r) sendTelegramDirect(m1Off ? alertBlock(ICON_BAD, "Mất kết nối công tơ Moong 1", "Kiểm tra dây RS485 / nguồn công tơ / địa chỉ Modbus." + durationSuffix(alM1Off, now))
                                    : alertBlock(ICON_SUCCESS, "Công tơ Moong 1 đã kết nối lại"));
    r = evalAlert(alM2Off, m2Off, now);
    if (r) sendTelegramDirect(m2Off ? alertBlock(ICON_BAD, "Mất kết nối công tơ Moong 2", "Kiểm tra dây RS485 / nguồn công tơ / địa chỉ Modbus." + durationSuffix(alM2Off, now))
                                    : alertBlock(ICON_SUCCESS, "Công tơ Moong 2 đã kết nối lại"));

    r = evalAlert(alM1Phase, m1PhaseLoss, now);   // [PHASE]
    if (r) sendTelegramDirect(m1PhaseLoss
        ? alertBlock(ICON_CRIT, "🚨 MẤT PHA Moong 1", "Mất pha: " + phaseLossList(m1) + "\nNGUY CƠ CHÁY ĐỘNG CƠ/THIẾT BỊ — nên tắt máy ngay nếu đang chạy!" + durationSuffix(alM1Phase, now))
        : alertBlock(ICON_SUCCESS, "Moong 1: đã có đủ điện áp cả 3 pha trở lại"));
    r = evalAlert(alM2Phase, m2PhaseLoss, now);
    if (r) sendTelegramDirect(m2PhaseLoss
        ? alertBlock(ICON_CRIT, "🚨 MẤT PHA Moong 2", "Mất pha: " + phaseLossList(m2) + "\nNGUY CƠ CHÁY ĐỘNG CƠ/THIẾT BỊ — nên tắt máy ngay nếu đang chạy!" + durationSuffix(alM2Phase, now))
        : alertBlock(ICON_SUCCESS, "Moong 2: đã có đủ điện áp cả 3 pha trở lại"));

    r = evalAlert(alM1Volt, m1VoltBad, now);
    if (r) sendTelegramDirect(m1VoltBad
        ? alertBlock(ICON_CRIT, "Điện áp Moong 1 bất thường", phaseVoltProblem(m1, alertUMin, alertUMax) + "\nNgưỡng: " + String(alertUMin,0) + "V - " + String(alertUMax,0) + "V" + durationSuffix(alM1Volt, now))
        : alertBlock(ICON_SUCCESS, "Điện áp Moong 1 đã bình thường trở lại"));
    r = evalAlert(alM2Volt, m2VoltBad, now);
    if (r) sendTelegramDirect(m2VoltBad
        ? alertBlock(ICON_CRIT, "Điện áp Moong 2 bất thường", phaseVoltProblem(m2, alertUMin, alertUMax) + "\nNgưỡng: " + String(alertUMin,0) + "V - " + String(alertUMax,0) + "V" + durationSuffix(alM2Volt, now))
        : alertBlock(ICON_SUCCESS, "Điện áp Moong 2 đã bình thường trở lại"));

    r = evalAlert(alM1Cur, m1CurBad, now);
    if (r) sendTelegramDirect(m1CurBad
        ? alertBlock(ICON_CRIT, "Dòng điện Moong 1 vượt ngưỡng", "Ia=" + String(m1.Ia,1) + "A Ib=" + String(m1.Ib,1) + "A Ic=" + String(m1.Ic,1) + "A\nNgưỡng: " + String(alertIMaxM1,1) + "A" + durationSuffix(alM1Cur, now))
        : alertBlock(ICON_SUCCESS, "Dòng điện Moong 1 đã bình thường trở lại"));
    r = evalAlert(alM2Cur, m2CurBad, now);
    if (r) sendTelegramDirect(m2CurBad
        ? alertBlock(ICON_CRIT, "Dòng điện Moong 2 vượt ngưỡng", "Ia=" + String(m2.Ia,1) + "A Ib=" + String(m2.Ib,1) + "A Ic=" + String(m2.Ic,1) + "A\nNgưỡng: " + String(alertIMaxM2,1) + "A" + durationSuffix(alM2Cur, now))
        : alertBlock(ICON_SUCCESS, "Dòng điện Moong 2 đã bình thường trở lại"));

    r = evalAlert(alM1Imbalance, m1Imb, now);   // [IMBAL]
    if (r) { sendTelegramDirect(m1Imb
        ? alertBlock(ICON_WARN, "Lệch pha / mất cân bằng tải Moong 1", "Ia=" + String(m1.Ia,1) + "A Ib=" + String(m1.Ib,1) + "A Ic=" + String(m1.Ic,1) + "A (TB=" + String(m1Avg,1) + "A)\nNgưỡng lệch: " + String(alertImbalancePct,0) + "%" + durationSuffix(alM1Imbalance, now))
        : alertBlock(ICON_SUCCESS, "Moong 1: dòng điện 3 pha đã cân bằng trở lại")); }
    r = evalAlert(alM2Imbalance, m2Imb, now);
    if (r) { sendTelegramDirect(m2Imb
        ? alertBlock(ICON_WARN, "Lệch pha / mất cân bằng tải Moong 2", "Ia=" + String(m2.Ia,1) + "A Ib=" + String(m2.Ib,1) + "A Ic=" + String(m2.Ic,1) + "A (TB=" + String(m2Avg,1) + "A)\nNgưỡng lệch: " + String(alertImbalancePct,0) + "%" + durationSuffix(alM2Imbalance, now))
        : alertBlock(ICON_SUCCESS, "Moong 2: dòng điện 3 pha đã cân bằng trở lại")); }

    r = evalAlert(alM1PF, m1PFBad, now);   // [PF]
    if (r) { sendTelegramDirect(m1PFBad
        ? alertBlock(ICON_WARN, "Hệ số công suất Moong 1 thấp", "PF=" + String(m1.PFt,2) + " (ngưỡng " + String(alertPFMin,2) + ")\nKiểm tra động cơ/thiết bị: kẹt cơ, mòn bạc đạn, non tải..." + durationSuffix(alM1PF, now))
        : alertBlock(ICON_SUCCESS, "Moong 1: hệ số công suất đã bình thường trở lại")); }
    r = evalAlert(alM2PF, m2PFBad, now);
    if (r) { sendTelegramDirect(m2PFBad
        ? alertBlock(ICON_WARN, "Hệ số công suất Moong 2 thấp", "PF=" + String(m2.PFt,2) + " (ngưỡng " + String(alertPFMin,2) + ")\nKiểm tra động cơ/thiết bị: kẹt cơ, mòn bạc đạn, non tải..." + durationSuffix(alM2PF, now))
        : alertBlock(ICON_SUCCESS, "Moong 2: hệ số công suất đã bình thường trở lại")); }

    if (m1Off)      bits |= EBIT_M1_OFFLINE;
    if (m2Off)      bits |= EBIT_M2_OFFLINE;
    if (m1VoltBad || m1PhaseLoss) bits |= EBIT_M1_VOLT;
    if (m2VoltBad || m2PhaseLoss) bits |= EBIT_M2_VOLT;
    if (m1CurBad)   bits |= EBIT_M1_CURRENT;
    if (m2CurBad)   bits |= EBIT_M2_CURRENT;
    if (m1Off && m2Off) bits |= EBIT_GRID_DOWN;
    if (m1Imb || m2Imb || m1PFBad || m2PFBad) bits |= EBIT_OTHER;
    currentAlertBits = bits;

    // [ALARM] Còi vật lý bật khi có BẤT KỲ cảnh báo nghiêm trọng nào
    // đang HOẠT ĐỘNG (không chờ debounce 15s để phản ứng nhanh tại chỗ).
    physicalAlarmActive = m1Off || m2Off || m1PhaseLoss || m2PhaseLoss || m1VoltBad || m2VoltBad || m1CurBad || m2CurBad;
}

//======================================================
// [PHASE-LINK v1.9.2] HANDLER CHO MASTER — GET /phase_status
// Trả JSON gọn, đọc từ cờ đã tính sẵn trong checkAlerts() (không tính lại
// ở đây để tránh 2 nơi có thể ra kết quả lệch nhau).
//======================================================
void handlePhaseStatus() {
    StaticJsonDocument<192> doc;
    doc["anyPhaseLoss"] = anyPhaseLossGlobal;   // trường Master thực sự dùng để ép dừng bơm
    doc["m1PhaseLoss"]  = m1PhaseLossGlobal;    // thêm chi tiết để debug qua trình duyệt/curl nếu cần
    doc["m2PhaseLoss"]  = m2PhaseLossGlobal;
    doc["m1Online"]     = m1.online;
    doc["m2Online"]     = m2.online;
    String out;
    serializeJson(doc, out);
    phaseServer.send(200, "application/json", out);
}

//======================================================
// XÂY DỰNG TIN NHẮN
//======================================================
String meterBlock(const char *name, MeterData &m, EnergyStat &s, float alertIMax) {
    String out = "*" + String(name) + "*  " + (m.online ? String(ICON_OK) + " Online" : String(ICON_BAD) + " Mất kết nối") + "\n";
    if (!m.online) return out;

    // [DOT-STATUS v1.9.1] Chấm 🟢/🔴 trước từng thông số: xanh = trong ngưỡng bình thường,
    // đỏ = bất thường hoặc mất pha — dùng lại đúng các ngưỡng đang áp dụng trong checkAlerts().
    bool lossA = m.Ua < PHASE_LOSS_V, lossB = m.Ub < PHASE_LOSS_V, lossC = m.Uc < PHASE_LOSS_V;
    bool uBadA = !lossA && (m.Ua < alertUMin || m.Ua > alertUMax);
    bool uBadB = !lossB && (m.Ub < alertUMin || m.Ub > alertUMax);
    bool uBadC = !lossC && (m.Uc < alertUMin || m.Uc > alertUMax);
    String dUa = (lossA || uBadA) ? ICON_BAD : ICON_OK;
    String dUb = (lossB || uBadB) ? ICON_BAD : ICON_OK;
    String dUc = (lossC || uBadC) ? ICON_BAD : ICON_OK;
    out += " ▪️ Điện áp: " + dUa + "A=" + String(m.Ua,1) + "V  " + dUb + "B=" + String(m.Ub,1) + "V  " + dUc + "C=" + String(m.Uc,1) + "V\n";

    bool iBadA = m.Ia > alertIMax, iBadB = m.Ib > alertIMax, iBadC = m.Ic > alertIMax;
    String dIa = iBadA ? ICON_BAD : ICON_OK;
    String dIb = iBadB ? ICON_BAD : ICON_OK;
    String dIc = iBadC ? ICON_BAD : ICON_OK;
    out += " ▪️ Dòng điện: " + dIa + "A=" + String(m.Ia,1) + "A  " + dIb + "B=" + String(m.Ib,1) + "A  " + dIc + "C=" + String(m.Ic,1) + "A\n";

    // Kiểm tra "tam giác công suất" S >= |P| và S >= |Q| — vi phạm nghĩa là P/Q/S không nhất
    // quán vật lý (đọc sai thanh ghi), tô đỏ cả 3 dòng P/Q/S vì cùng một lô dữ liệu đọc lên.
    bool powerBad = (m.St + POWER_TRIANGLE_TOLERANCE_W < fabs(m.Pt)) ||
                    (m.St + POWER_TRIANGLE_TOLERANCE_W < fabs(m.Qt));
    String dPower = powerBad ? ICON_BAD : ICON_OK;
    out += " ▪️ " + dPower + " Công suất tác dụng P: " + String(m.Pt/1000.0, 2) + " kW\n";
    out += " ▪️ " + dPower + " Công suất phản kháng Q: " + String(m.Qt/1000.0, 2) + " kvar\n";
    out += " ▪️ " + dPower + " Công suất biểu kiến S: " + String(m.St/1000.0, 2) + " kVA\n";

    bool pfBad = (fabs(m.PFt) > 1.05f) || (fabs(m.Pt) > PF_MIN_POWER_W && fabs(m.PFt) < alertPFMin);
    bool freqBad = m.Freq < FREQ_SANE_MIN || m.Freq > FREQ_SANE_MAX;
    String dPF = pfBad ? ICON_BAD : ICON_OK;
    String dFreq = freqBad ? ICON_BAD : ICON_OK;
    out += " ▪️ " + dPF + " PF=" + String(m.PFt,3) + "    " + dFreq + " Tần số: " + String(m.Freq,2) + " Hz\n";

    out += " ▪️ Điện năng tác dụng: " + String(m.EnergyTotal, 1) + " kWh\n";
    // [FIX v1.9.1] Đã bỏ dòng "Điện năng phản kháng" — số liệu cũ sai hoàn toàn (đọc nhầm thanh ghi
    // NetImpEp). Sẽ thêm lại khi xác định được đúng địa chỉ thanh ghi kvarh của CHINT.
    out += " ▪️ Hôm nay: " + String(kwhToday(s, m.EnergyTotal), 2) + " kWh"
         + "   Tháng này: " + String(kwhMonth(s, m.EnergyTotal), 1) + " kWh"
         + "   Năm này: " + String(kwhYear(s, m.EnergyTotal), 1) + " kWh\n";
    return out;
}
// [TIME-DISPLAY] Định dạng ngày giờ hệ thống kiểu Việt Nam: HH:MM:SS dd/mm/yyyy
String formatDateTime(struct tm &t) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d %02d/%02d/%04d",
             t.tm_hour, t.tm_min, t.tm_sec, t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
    return String(buf);
}
String buildStatus() {
    String m = "⚡ *TRẠM ĐIỆN - TRẠNG THÁI*\n" + divider();
    m += meterBlock("Moong 1", m1, statM1, alertIMaxM1) + "\n";
    m += meterBlock("Moong 2", m2, statM2, alertIMaxM2) + "\n";
    m += divider();
    m += "📶 WiFi STA: " + String(WiFi.status() == WL_CONNECTED ? "🟢 Đã kết nối (" + WiFi.SSID() + ")" : "🔴 Mất kết nối") + "\n";
    if (netDownSince > 0) {
        m += "⏱️ Đang mất mạng: " + String((millis() - netDownSince) / 60000UL) + " phút\n";
    }
    m += "🔔 Còi báo động: " + String(physicalAlarmActive ? "🚨 ĐANG BẬT" : "Tắt (bình thường)") + "\n";
    struct tm tNow;
    if (ntpSynced && getNow(tNow)) m += "🕐 Giờ hệ thống: 🟢 " + formatDateTime(tNow) + "\n";
    else m += "🕐 Giờ hệ thống: ⚠️ Chưa đồng bộ\n";
    m += "⏳ Uptime: " + formatTimeSpan(millis() - bootTime) + "\n";
    m += divider();
    m += "🔧 *Ngưỡng cảnh báo đang áp dụng*\n";
    m += "⚡ Điện áp: " + String(alertUMin, 0) + "V — " + String(alertUMax, 0) + "V\n";
    m += "🔌 Dòng tối đa: M1 " + String(alertIMaxM1, 1) + "A   M2 " + String(alertIMaxM2, 1) + "A\n";
    m += "⚖️ Lệch pha/mất cân bằng tải: " + String(alertImbalancePct, 0) + "%\n";
    m += "📐 Hệ số công suất tối thiểu: " + String(alertPFMin, 2) + "\n";
    m += "📈 Bội số cảnh báo đột biến: " + String(spikeMultiplier, 2) + " lần TB\n";
    m += "💾 Firmware: " + escapeMarkdown(String(FW_VERSION));
    return m;
}
String buildEnergyReport() {
    String m = "📊 *BÁO CÁO ĐIỆN NĂNG*\n" + divider();
    m += "*Moong 1*\n";
    m += " ▪️ Hôm nay: " + String(kwhToday(statM1, m1.EnergyTotal), 2) + " kWh   (hôm qua: " + String(statM1.yesterdayKwh, 2) + " kWh, TB gần đây: " + String(statM1.dailyAvgKwh < 0 ? 0 : statM1.dailyAvgKwh, 2) + " kWh)\n";
    m += " ▪️ Tháng này: " + String(kwhMonth(statM1, m1.EnergyTotal), 1) + " kWh   (tháng trước: " + String(touTotal(statM1.lastMonthTou), 1) + " kWh) ~ " + String(computeTouCost(statM1.monthTou), 0) + " đ\n";
    m += " ▪️ Năm này: " + String(kwhYear(statM1, m1.EnergyTotal), 1) + " kWh   (năm trước: " + String(statM1.lastYearKwh, 1) + " kWh)\n\n";
    m += "*Moong 2*\n";
    m += " ▪️ Hôm nay: " + String(kwhToday(statM2, m2.EnergyTotal), 2) + " kWh   (hôm qua: " + String(statM2.yesterdayKwh, 2) + " kWh, TB gần đây: " + String(statM2.dailyAvgKwh < 0 ? 0 : statM2.dailyAvgKwh, 2) + " kWh)\n";
    m += " ▪️ Tháng này: " + String(kwhMonth(statM2, m2.EnergyTotal), 1) + " kWh   (tháng trước: " + String(touTotal(statM2.lastMonthTou), 1) + " kWh) ~ " + String(computeTouCost(statM2.monthTou), 0) + " đ\n";
    m += " ▪️ Năm này: " + String(kwhYear(statM2, m2.EnergyTotal), 1) + " kWh   (năm trước: " + String(statM2.lastYearKwh, 1) + " kWh)\n\n";
    // [SPIKE] so sánh nhanh 2 Moong để dễ đối chiếu bằng mắt
    m += "*So sánh 2 Moong (hôm nay):* M1=" + String(kwhToday(statM1, m1.EnergyTotal),2) + " kWh   M2=" + String(kwhToday(statM2, m2.EnergyTotal),2) + " kWh";
    return m;
}
// [TOU] Báo cáo tiền điện tháng này theo 3 khung giờ, tách riêng cho dễ xem.
String buildCostReport() {
    double cM1 = computeTouCost(statM1.monthTou), cM2 = computeTouCost(statM2.monthTou);
    String m = "💰 *ƯỚC TÍNH TIỀN ĐIỆN THÁNG NÀY (đã gồm VAT)*\n" + divider();
    m += "*Moong 1:* " + String(touTotal(statM1.monthTou), 1) + " kWh  ~  " + String(cM1, 0) + " đ\n";
    m += "   BT=" + String(statM1.monthTou.bt,1) + " TD=" + String(statM1.monthTou.td,1) + " CD=" + String(statM1.monthTou.cd,1) + " kWh\n";
    m += "*Moong 2:* " + String(touTotal(statM2.monthTou), 1) + " kWh  ~  " + String(cM2, 0) + " đ\n";
    m += "   BT=" + String(statM2.monthTou.bt,1) + " TD=" + String(statM2.monthTou.td,1) + " CD=" + String(statM2.monthTou.cd,1) + " kWh\n\n";
    m += "*Tổng cộng:* ~ " + String(cM1 + cM2, 0) + " đ\n\n";
    m += "Gõ /tiers để xem biểu giá/khung giờ đang dùng để tính.";
    return m;
}
// [REPORT] Nội dung báo cáo tự động hằng ngày.
String buildDailyReport() {
    String m = "🌅 *BÁO CÁO HẰNG NGÀY*\n" + divider();
    m += "*Moong 1:* hôm qua dùng " + String(statM1.yesterdayKwh, 2) + " kWh (~" + String(computeTouCost(statM1.yestTou), 0) + " đ, BT=" + String(statM1.yestTou.bt,1) + " TD=" + String(statM1.yestTou.td,1) + " CD=" + String(statM1.yestTou.cd,1) + ")\n";
    m += "*Moong 2:* hôm qua dùng " + String(statM2.yesterdayKwh, 2) + " kWh (~" + String(computeTouCost(statM2.yestTou), 0) + " đ, BT=" + String(statM2.yestTou.bt,1) + " TD=" + String(statM2.yestTou.td,1) + " CD=" + String(statM2.yestTou.cd,1) + ")\n\n";
    m += "Tháng này (tạm tính): M1 " + String(kwhMonth(statM1, m1.EnergyTotal),1) + " kWh, M2 " + String(kwhMonth(statM2, m2.EnergyTotal),1) + " kWh";
    return m;
}
String buildHelp(bool isAdmin) {
    String m = "⚡ *TRẠM ĐIỆN v" + escapeMarkdown(String(FW_VERSION)) + " — DANH SÁCH LỆNH*\n" + divider();

    // --- Xem thông tin ---
    m += "📊 *Xem thông tin*\n";
    m += "/status — Trạng thái tức thời: U, I, P, Q, S, PF, tần số\n";
    m += "/energy — Điện năng kWh hôm nay / tháng / năm + kvarh\n";
    m += "/tien — Ước tính tiền điện tháng này (3 khung giờ)\n";
    m += "/tiers — Biểu giá & khung giờ đang áp dụng\n";
    m += "/help — Hiển thị tin nhắn này\n";

    if (!isAdmin) return m;

    // --- Ngưỡng cảnh báo ---
    m += "\n" + divider();
    m += "🔔 *Ngưỡng cảnh báo* _(chỉ ADMIN)_\n";
    m += "/set\\_u `<Umin> <Umax>` — Ngưỡng điện áp (V)\n";
    m += "  `VD: /set_u 180 250`\n";
    m += "/set\\_i\\_m1 `<A>` — Ngưỡng dòng Moong 1\n";
    m += "/set\\_i\\_m2 `<A>` — Ngưỡng dòng Moong 2\n";
    m += "/set\\_imb `<%>` — Ngưỡng lệch pha / mất cân bằng tải\n";
    m += "  `VD: /set_imb 20`\n";
    m += "/set\\_pf `<0..1>` — Ngưỡng hệ số công suất thấp\n";
    m += "  `VD: /set_pf 0.70`\n";
    m += "/set\\_spike `<bội số>` — Hệ số đột biến so TB động\n";
    m += "  `VD: /set_spike 1.6`\n";

    // --- Giá điện & khung giờ ---
    m += "\n" + divider();
    m += "💰 *Giá điện & khung giờ* _(chỉ ADMIN)_\n";
    m += "/set\\_gia `<giá BT> <giá TD> <giá CD>` — 3 đơn giá (đ/kWh, chưa VAT)\n";
    m += "  `VD: /set_gia 1987 1300 3640`\n";
    m += "/set\\_vat `<%>` — Đặt % VAT\n";
    m += "  `VD: /set_vat 8`\n";
    m += "/set\\_tou `<TĐ bắt đầu hhmm> <TĐ kết thúc hhmm> <CĐ bắt đầu hhmm> <CĐ kết thúc hhmm>`\n";
    m += "  Đổi giờ thấp điểm / cao điểm — `VD: /set_tou 0000 0600 1730 2230`\n";
    m += "/set\\_report\\_hour `<0-23>` — Giờ gửi báo cáo hằng ngày\n";
    m += "  `VD: /set_report_hour 7`\n";
    m += "/set\\_reboot\\_hour `<0-23>` — Giờ tự khởi động lại hằng ngày (mặc định 0 = 00h00)\n";
    m += "/reboot\\_on — Bật tự khởi động lại hằng ngày\n";
    m += "/reboot\\_off — Tắt tự khởi động lại hằng ngày\n";

    // --- Quản lý người dùng ---
    m += "\n" + divider();
    m += "👥 *Quản lý người dùng* _(chỉ ADMIN)_\n";
    m += "/users — Xem danh sách người nhận cảnh báo\n";
    m += "/adduser `<chat_id>` `[admin]` — Thêm người dùng\n";
    m += "  `VD: /adduser 123456789 admin`\n";
    m += "/removeuser `<chat_id>` — Xoá người dùng\n";

    // --- Hệ thống ---
    m += "\n" + divider();
    m += "⚙️ *Hệ thống* _(chỉ ADMIN)_\n";
    m += "/set\\_wifi `<SSID>;<PASS>` — Thêm WiFi (không mất mạng cũ, áp dụng sau /reset)\n";
    m += "/list\\_wifi — Xem DS WiFi đã lưu\n";
    m += "/del\\_wifi `<số>` — Xoá WiFi theo STT\n";
    m += "/reset — Khởi động lại thiết bị\n";
    m += "/version — Phiên bản firmware hiện tại\n";
    m += "/update — Kiểm tra & cập nhật firmware mới nhất\n";
    m += "/update\\_list — Xem tất cả các bản firmware\n";
    m += "/update\\_to `<phiên bản>` — Chọn bản cụ thể để cài\n";
    m += "/update\\_confirm — Xác nhận cài đặt\n";
    m += "/update\\_cancel — Huỷ yêu cầu cập nhật";

    return m;
}

//======================================================
// OTA
//======================================================
struct OtaEntry { String version, file, note; };
OtaEntry otaList[OTA_MAX_VERSIONS];
int      otaListCount = 0;
String   otaPendingVersion = "", otaPendingFile = "";
unsigned long otaPendingExpireAt = 0;

bool fetchOtaManifest() {
    otaListCount = 0;
    if (WiFi.status() != WL_CONNECTED) { sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Chưa có WiFi.")); return false; }
    WiFiClientSecure client; client.setInsecure(); client.setTimeout(15000); client.setHandshakeTimeout(10);
    HTTPClient http;
    if (!http.begin(client, OTA_MANIFEST_URL)) { sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Không mở được kết nối GitHub.")); return false; }
    http.setTimeout(OTA_HTTP_TIMEOUT_MS);
    int code = http.GET();
    if (code != HTTP_CODE_OK) { sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "HTTP " + String(code))); http.end(); return false; }
    String payload = http.getString(); http.end();
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, payload)) { sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Lỗi JSON.")); return false; }
    JsonArray arr = doc["versions"].as<JsonArray>();
    for (JsonObject o : arr) {
        if (otaListCount >= OTA_MAX_VERSIONS) break;
        const char *v = o["version"] | ""; const char *f = o["file"] | ""; const char *n = o["note"] | "";
        if (strlen(v) == 0 || strlen(f) == 0) continue;
        otaList[otaListCount] = { String(v), String(f), String(n) }; otaListCount++;
    }
    if (otaListCount == 0) { sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "energy_versions.json rỗng.")); return false; }
    return true;
}
void setOtaPending(const String &version, const String &file) {
    otaPendingVersion = version; otaPendingFile = file;
    otaPendingExpireAt = millis() + OTA_CONFIRM_TIMEOUT_MS;
    sendTelegramDirect(alertBlock(ICON_WARN, "Đã chọn bản để cập nhật - cần xác nhận",
        "Hiện tại: " + escapeMarkdown(String(FW_VERSION)) + " -> Chọn: " + escapeMarkdown(version) + " (" + escapeMarkdown(file) + ")\n\n/update_confirm để cài, /update_cancel để huỷ. Hết hạn sau 5 phút."));
}
void checkOTAUpdate() {
    if (!fetchOtaManifest()) return;
    if (otaList[0].version == FW_VERSION) { otaPendingVersion=""; otaPendingFile=""; sendTelegramDirect(alertBlock(ICON_OK, "Đã là bản mới nhất", escapeMarkdown(String(FW_VERSION)))); return; }
    setOtaPending(otaList[0].version, otaList[0].file);
}
void listOtaVersions() {
    if (!fetchOtaManifest()) return;
    String msg = "Bản hiện tại: " + escapeMarkdown(String(FW_VERSION)) + "\n";
    for (int i = 0; i < otaListCount; i++) {
        msg += "\n" + String(i+1) + ". " + escapeMarkdown(otaList[i].version);
        if (otaList[i].version == FW_VERSION) msg += " (đang chạy)";
        msg += "\n    file: " + escapeMarkdown(otaList[i].file);
        if (otaList[i].note.length()) msg += "\n    " + escapeMarkdown(otaList[i].note);
    }
    msg += "\n\n/update_to <số thứ tự|version> để chọn bản.";
    sendTelegramDirect(alertBlock(ICON_INFO, "Danh sách firmware Trạm Điện", msg));
}
void selectOtaVersion(String arg) {
    arg.trim();
    if (arg.length() == 0) { sendTelegramDirect(alertBlock(ICON_WARN, "Thiếu tham số", "/update_to <số|version>")); return; }
    if (!fetchOtaManifest()) return;
    bool numeric = true;
    for (unsigned int i = 0; i < arg.length(); i++) if (!isDigit(arg.charAt(i))) { numeric = false; break; }
    int idx = -1;
    if (numeric) { int n = arg.toInt(); if (n >= 1 && n <= otaListCount) idx = n - 1; }
    if (idx == -1) for (int i = 0; i < otaListCount; i++) if (otaList[i].version == arg) { idx = i; break; }
    if (idx == -1) { sendTelegramDirect(alertBlock(ICON_BAD, "Không tìm thấy bản này", escapeMarkdown(arg))); return; }
    setOtaPending(otaList[idx].version, otaList[idx].file);
}
void performOTAUpdate() {
    String remoteVersion = otaPendingVersion, remoteFile = otaPendingFile;
    otaPendingVersion = ""; otaPendingFile = ""; otaRunning = true;
    String url = String(OTA_RAW_BASE_URL) + remoteFile;
    sendTelegramDirect(alertBlock(ICON_WARN, "Đang cập nhật firmware...", "Mới: " + escapeMarkdown(remoteVersion)));
    for (int i = 0; i < 100; i++) { esp_task_wdt_reset(); delay(10); }
    WiFiClientSecure fwClient; fwClient.setInsecure(); fwClient.setTimeout(15000); fwClient.setHandshakeTimeout(10);
    httpUpdate.rebootOnUpdate(false);
    esp_task_wdt_delete(NULL);
    t_httpUpdate_return ret = httpUpdate.update(fwClient, url);
    if (ret == HTTP_UPDATE_OK) {
        sendTelegramDirect(alertBlock(ICON_SUCCESS, "OTA thành công", "Đã lên bản: " + escapeMarkdown(remoteVersion) + "\nĐang khởi động lại..."));
        for (int i = 0; i < 100; i++) { esp_task_wdt_reset(); delay(10); }
        esp_task_wdt_add(NULL);
        bot.getUpdates(bot.last_message_received + 1);
        esp_task_wdt_delete(NULL);
        ESP.restart();
    } else {
        esp_task_wdt_add(NULL); otaRunning = false;
        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại", "Lỗi #" + String(httpUpdate.getLastError()) + ": " + escapeMarkdown(httpUpdate.getLastErrorString())));
    }
}

//======================================================
// XỬ LÝ LỆNH TELEGRAM
//======================================================
void handleNewMessages(int n, UniversalTelegramBot &tgBot) {
    for (int i = 0; i < n; i++) {
        String chatId = tgBot.messages[i].chat_id;
        String text   = tgBot.messages[i].text;

        if (!isRegisteredUser(chatId)) {
            if (text == "/start") tgBot.sendMessage(chatId, "⚠️ Bạn chưa được cấp quyền dùng Trạm Điện này. Liên hệ Admin để được thêm bằng /adduser.", "");
            continue;
        }
        bool admin = isAdminUser(chatId);

        // ---- Lệnh cho MỌI người dùng đã đăng ký ----
        if (text == "/status" || text == "/start") { sendTelegramDirect(buildStatus()); continue; }
        if (text == "/energy") { sendTelegramDirect(buildEnergyReport()); continue; }
        if (text == "/tien")   { sendTelegramDirect(buildCostReport()); continue; }
        if (text == "/tiers")  { sendTelegramDirect(buildTariffMessage()); continue; }
        if (text == "/help")   { sendTelegramDirect(buildHelp(admin)); continue; }
        if (text == "/users")  { sendTelegramDirect(buildUsersListMessage()); continue; }

        // ---- Lệnh chỉ ADMIN ----
        if (!admin) { tgBot.sendMessage(chatId, "⚠️ Lệnh này chỉ dành cho Admin.", ""); continue; }

        if (text.startsWith("/set_u ")) {
            String args = text.substring(7); args.trim(); int sp = args.indexOf(' ');
            if (sp > 0) { alertUMin = args.substring(0,sp).toFloat(); alertUMax = args.substring(sp+1).toFloat(); saveAlertConfig();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt ngưỡng điện áp", String(alertUMin,0)+"V - "+String(alertUMax,0)+"V")); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_u 180 250"));
        } else if (text.startsWith("/set_i_m1 ")) {
            String v = text.substring(10); v.trim();
            if (v.length() > 0) { alertIMaxM1 = v.toFloat(); saveAlertConfig();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt ngưỡng dòng Moong 1", String(alertIMaxM1,1)+"A")); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_i_m1 <A>\nVD: /set_i_m1 250"));
        } else if (text.startsWith("/set_i_m2 ")) {
            String v = text.substring(10); v.trim();
            if (v.length() > 0) { alertIMaxM2 = v.toFloat(); saveAlertConfig();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt ngưỡng dòng Moong 2", String(alertIMaxM2,1)+"A")); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_i_m2 <A>\nVD: /set_i_m2 250"));
        } else if (text.startsWith("/set_imb ")) {
            String v = text.substring(9); v.trim();
            if (v.length() > 0) { alertImbalancePct = v.toFloat(); saveAlertConfig();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt ngưỡng lệch pha", String(alertImbalancePct,0)+"%")); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_imb <%>\nVD: /set_imb 20"));
        } else if (text.startsWith("/set_pf ")) {
            String v = text.substring(8); v.trim();
            if (v.length() > 0) { alertPFMin = v.toFloat(); saveAlertConfig();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt ngưỡng hệ số công suất", String(alertPFMin,2))); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_pf <0..1>\nVD: /set_pf 0.70"));
        } else if (text.startsWith("/set_spike ")) {
            String v = text.substring(11); v.trim();
            if (v.length() > 0) { spikeMultiplier = v.toFloat(); saveAlertConfig();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt bội số cảnh báo đột biến", String(spikeMultiplier,2)+" lần")); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_spike <bội số>\nVD: /set_spike 1.6"));
        } else if (text.startsWith("/set_report_hour ")) {
            int h = text.substring(17).toInt();
            if (h >= 0 && h <= 23) { dailyReportHour = h; saveAlertConfig(); sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt giờ báo cáo hằng ngày", String(h)+":00")); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Giờ không hợp lệ", "Dùng 0-23"));
        } else if (text.startsWith("/set_reboot_hour ")) {   // [REBOOT]
            int h = text.substring(17).toInt();
            if (h >= 0 && h <= 23) { dailyRebootHour = h; saveAlertConfig();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt giờ tự khởi động lại hằng ngày", String(h)+":00" + String(dailyRebootEnabled ? "" : " (hiện đang TẮT, bật lại bằng /reboot_on)"))); }
            else sendTelegramDirect(alertBlock(ICON_WARN, "Giờ không hợp lệ", "Dùng 0-23"));
        } else if (text == "/reboot_on") {   // [REBOOT]
            dailyRebootEnabled = true; saveAlertConfig();
            sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã BẬT tự khởi động lại hằng ngày", "Giờ: " + String(dailyRebootHour) + ":00"));
        } else if (text == "/reboot_off") {   // [REBOOT]
            dailyRebootEnabled = false; saveAlertConfig();
            sendTelegramDirect(alertBlock(ICON_WARN, "Đã TẮT tự khởi động lại hằng ngày"));
        } else if (text.startsWith("/set_gia ")) {
            // Cú pháp: /set_gia <giá Bình thường> <giá Thấp điểm> <giá Cao điểm>  (đ/kWh, CHƯA VAT)
            String args = text.substring(9); args.trim();
            int sp1 = args.indexOf(' ');
            int sp2 = (sp1 > 0) ? args.indexOf(' ', sp1 + 1) : -1;
            if (sp1 > 0 && sp2 > 0) {
                priceBT = args.substring(0, sp1).toDouble();
                priceTD = args.substring(sp1 + 1, sp2).toDouble();
                priceCD = args.substring(sp2 + 1).toDouble();
                saveTariffToNVS();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã cập nhật giá điện", buildTariffMessage()));
            } else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_gia <giá BT> <giá TD> <giá CD>\nVD: /set_gia 1987 1300 3640"));
        } else if (text.startsWith("/set_vat ")) {
            vatPercent = text.substring(9).toDouble(); saveTariffToNVS();
            sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã đặt VAT", String(vatPercent,1) + "%"));
        } else if (text.startsWith("/set_tou ")) {
            // Cú pháp: /set_tou <TĐ bắt đầu hhmm> <TĐ kết thúc hhmm> <CĐ bắt đầu hhmm> <CĐ kết thúc hhmm>
            String args = text.substring(9); args.trim();
            int sp1 = args.indexOf(' ');
            int sp2 = (sp1 > 0) ? args.indexOf(' ', sp1 + 1) : -1;
            int sp3 = (sp2 > 0) ? args.indexOf(' ', sp2 + 1) : -1;
            if (sp1 > 0 && sp2 > 0 && sp3 > 0) {
                auto hhmmToMin = [](String s) { s.trim(); int v = s.toInt(); return (v / 100) * 60 + (v % 100); };
                touLowStartMin  = hhmmToMin(args.substring(0, sp1));
                touLowEndMin    = hhmmToMin(args.substring(sp1 + 1, sp2));
                touPeakStartMin = hhmmToMin(args.substring(sp2 + 1, sp3));
                touPeakEndMin   = hhmmToMin(args.substring(sp3 + 1));
                saveTariffToNVS();
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã cập nhật khung giờ", buildTariffMessage()));
            } else sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_tou <TĐ bắt đầu> <TĐ kết thúc> <CĐ bắt đầu> <CĐ kết thúc> (hhmm)\nVD: /set_tou 0000 0600 1730 2230"));
        } else if (text.startsWith("/set_wifi")) {
            // [WIFI-MULTI] Giống hệt cú pháp bên Master: SSID và mật khẩu
            // cách nhau bằng dấu ";". THÊM mạng mới vào danh sách — KHÔNG
            // xoá các mạng đã lưu trước đó (khác /wifi_set cũ, vốn ghi đè
            // và làm mất WiFi cũ).
            String args = text.substring(9); args.trim();
            String newSsid, newPass; int semiIdx = args.indexOf(';');
            if (semiIdx != -1) { newSsid = args.substring(0, semiIdx); newPass = args.substring(semiIdx + 1); }
            newSsid.trim(); newPass.trim();
            if (newSsid.length() > 0 && newPass.length() > 0) {
                int res = addOrUpdateWifiNetwork(newSsid, newPass);
                if (res != 0) {
                    sendTelegramDirect(alertBlock(ICON_SUCCESS, "Lưu WiFi mới", "SSID: " + newSsid + "\nÁp dụng sau khi /reset."));
                } else {
                    sendTelegramDirect(alertBlock(ICON_WARN, "Không lưu được", "Đã đủ tối đa " + String(MAX_WIFI_NETWORKS) + " mạng. Xoá bớt bằng /del_wifi trước."));
                }
            } else {
                sendTelegramDirect(alertBlock(ICON_WARN, "Sai cú pháp", "/set_wifi <SSID>;<PASS>"));
            }
        } else if (text == "/list_wifi") {
            String wifiListMsg = "📋 *DANH SÁCH WIFI ĐÃ LƯU*\n" + divider();
            for (int j = 0; j < savedWifiCount; j++) {
                wifiListMsg += "├─ " + String(j + 1) + ". SSID: `" + savedWifiList[j].ssid + "` | PASS: `" + savedWifiList[j].pass + "`\n";
            }
            if (savedWifiCount == 0) wifiListMsg += "❌ Chưa lưu WiFi nào.\n";
            sendTelegramDirect(wifiListMsg);
        } else if (text.startsWith("/del_wifi")) {
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
            sendTelegramDirect(replyMsg);
        } else if (text.startsWith("/adduser ")) {
            String args = text.substring(9); args.trim();
            int sp = args.indexOf(' ');
            String targetId = (sp > 0) ? args.substring(0, sp) : args;
            bool asAdmin = (sp > 0) && (args.substring(sp + 1) == "admin");
            if (addUserToList(targetId, asAdmin ? ROLE_ADMIN : ROLE_USER))
                sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã thêm người dùng", escapeMarkdown(targetId) + (asAdmin ? " (ADMIN)" : " (USER)")));
            else sendTelegramDirect(alertBlock(ICON_WARN, "Không thêm được", "Đã tồn tại hoặc danh sách đầy."));
        } else if (text.startsWith("/removeuser ")) {
            String targetId = text.substring(12); targetId.trim();
            if (removeUserFromList(targetId)) sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đã xoá người dùng", escapeMarkdown(targetId)));
            else sendTelegramDirect(alertBlock(ICON_WARN, "Không tìm thấy", targetId));
        } else if (text == "/reset") {
            sendTelegramDirect(alertBlock(ICON_WARN, "Đang khởi động lại..."));
            // [FIX v1.3.2] Task WDT đã bị esp_task_wdt_delete(NULL) ở loop()
            // trước khi vào đây (xem nơi gọi handleNewMessages), nên trước đây
            // vòng lặp esp_task_wdt_reset() 60 lần ở đây không hề có tác dụng
            // (task chưa được add lại) — chỉ là code thừa. Bỏ hẳn, chỉ cần chờ
            // ngắn để tin nhắn kịp gửi trước khi lấy nốt update rồi khởi động lại.
            delay(600);
            esp_task_wdt_add(NULL);
            tgBot.getUpdates(tgBot.last_message_received + 1);
            esp_task_wdt_delete(NULL);
            ESP.restart();
        } else if (text == "/version") {
            sendTelegramDirect(alertBlock(ICON_INFO, "Phiên bản firmware", escapeMarkdown(String(FW_VERSION))));
        } else if (text == "/update") { checkOTAUpdate(); }
        else if (text == "/update_list") { listOtaVersions(); }
        else if (text.startsWith("/update_to ")) { selectOtaVersion(text.substring(11)); }
        else if (text == "/update_confirm") {
            if (otaPendingVersion.length() == 0 || (long)(millis() - otaPendingExpireAt) > 0)
                sendTelegramDirect(alertBlock(ICON_WARN, "Không có yêu cầu OTA nào đang chờ", "/update trước."));
            else performOTAUpdate();
        } else if (text == "/update_cancel") {
            otaPendingVersion = ""; otaPendingFile = "";
            sendTelegramDirect(alertBlock(ICON_INFO, "Đã huỷ yêu cầu cập nhật."));
        }
    }
}

//======================================================
// [REBOOT] TỰ ĐỘNG KHỞI ĐỘNG LẠI HẰNG NGÀY
// Khởi động lại định kỳ 1 lần/ngày (mặc định 00h00, chỉnh được bằng
// /set_reboot_hour, bật/tắt bằng /reboot_on /reboot_off) để dọn bộ nhớ
// (chống phân mảnh heap do dùng nhiều String khi chạy liên tục nhiều
// tháng) — đây là bảo trì định kỳ, KHÔNG phải sự cố. Số liệu điện năng
// được lưu vào NVS trước khi reboot nên không mất dữ liệu.
//======================================================
void performDailyReboot() {
    if (m1.online) saveEnergyStat("m1_", statM1);
    if (m2.online) saveEnergyStat("m2_", statM2);
    sendTelegramDirect(alertBlock(ICON_INFO, "Khởi động lại định kỳ hằng ngày",
        "Giờ: " + String(dailyRebootHour) + "h00 — bảo trì bộ nhớ định kỳ, không phải sự cố."
        "\nTắt bằng /reboot_off nếu không muốn dùng."));
    esp_task_wdt_reset();
    delay(500);   // đợi tin nhắn Telegram gửi xong trước khi mất kết nối
    ESP.restart();
}

//======================================================
// SETUP & LOOP
//======================================================
//======================================================
// [MQTT-DISPLAY v1.9.3] ĐẨY DỮ LIỆU LÊN MQTT CHO MÀN HÌNH LVGL Ở NHÀ
// Màn hình ở mạng WiFi khác nên không gọi thẳng vào trạm được. Trạm chủ động
// PUBLISH (retained) 1 gói JSON mỗi 10 giây lên broker; màn hình SUBSCRIBE.
// Toàn bộ chạy trong 1 FreeRTOS task riêng trên core 0: bắt tay TLS/mất mạng
// có chậm tới vài chục giây cũng KHÔNG chặn loop() — nên /phase_status cho
// Master (an toàn bơm), đọc công tơ và Telegram không bị ảnh hưởng.
// Trạm chỉ GỬI RA, không nhận lệnh nào từ MQTT (không subscribe topic nào).
//======================================================
#define MQTT_ENABLED       1
#define MQTT_USE_TLS       1        // 1 = TLS cổng 8883 (HiveMQ Cloud...) ; 0 = không mã hoá, cổng 1883
#define MQTT_HOST          "xxxxxxxxxxxxxxxx.s1.eu.hivemq.cloud"
#define MQTT_PORT          8883
#define MQTT_USER          "tramdien_tram"          // tài khoản được PUBLISH
#define MQTT_PASS          "DOI_MAT_KHAU_NAY"
#define MQTT_TOPIC_BASE    "tramdien/DOI_CHUOI_NGAU_NHIEN"   // phải TRÙNG phía màn hình
#define MQTT_PUBLISH_MS    10000UL
#define MQTT_RETRY_MS      20000UL
#define MQTT_MIN_HEAP      60000UL  // RAM trống thấp hơn mức này thì không mở kết nối MQTT (TLS tốn ~40KB)

#if MQTT_ENABLED
#if MQTT_USE_TLS
static WiFiClientSecure mqttNet;
#else
static WiFiClient       mqttNet;
#endif
static PubSubClient     mqttClient(mqttNet);
static const char      *MQTT_T_STATE  = MQTT_TOPIC_BASE "/state";
static const char      *MQTT_T_STATUS = MQTT_TOPIC_BASE "/status";
static unsigned long    mqttLastTry = 0, mqttLastPub = 0;
static char             mqttBuf[1800];

static float fin(float v) { return isfinite(v) ? v : 0.0f; }

// Ghi JSON của 1 công tơ vào b (tối đa cap byte). Trả số byte đã ghi.
static int mqttMeterJson(char *b, size_t cap, MeterData &m, EnergyStat &s, bool phaseLoss, float iMax) {
    bool uv = m.online && !phaseLoss && (m.Ua < alertUMin || m.Ua > alertUMax || m.Ub < alertUMin || m.Ub > alertUMax || m.Uc < alertUMin || m.Uc > alertUMax);
    bool ic = m.online && (m.Ia > iMax || m.Ib > iMax || m.Ic > iMax);
    return snprintf(b, cap,
        "{\"on\":%d,\"pl\":%d,\"uv\":%d,\"ic\":%d,"
        "\"Ua\":%.1f,\"Ub\":%.1f,\"Uc\":%.1f,\"Ia\":%.2f,\"Ib\":%.2f,\"Ic\":%.2f,"
        "\"P\":%.2f,\"Q\":%.2f,\"S\":%.2f,\"PF\":%.3f,\"F\":%.2f,\"E\":%.1f,"
        "\"d\":%.2f,\"y\":%.2f,\"m\":%.1f,\"pm\":%.1f,\"yr\":%.1f,\"pyr\":%.1f,\"avg\":%.2f,"
        "\"cm\":%.0f,\"cy\":%.0f,\"cpm\":%.0f,\"bt\":%.1f,\"td\":%.1f,\"cd\":%.1f}",
        m.online ? 1 : 0, phaseLoss ? 1 : 0, uv ? 1 : 0, ic ? 1 : 0,
        fin(m.Ua), fin(m.Ub), fin(m.Uc), fin(m.Ia), fin(m.Ib), fin(m.Ic),
        fin(m.Pt / 1000.0f), fin(m.Qt / 1000.0f), fin(m.St / 1000.0f), fin(m.PFt), fin(m.Freq), fin(m.EnergyTotal),
        fin(kwhToday(s, m.EnergyTotal)), fin(s.yesterdayKwh), fin(kwhMonth(s, m.EnergyTotal)), fin(touTotal(s.lastMonthTou)),
        fin(kwhYear(s, m.EnergyTotal)), fin(s.lastYearKwh), fin(s.dailyAvgKwh < 0 ? 0 : s.dailyAvgKwh),
        fin(computeTouCost(s.monthTou)), fin(computeTouCost(s.yestTou)), fin(computeTouCost(s.lastMonthTou)),
        fin(s.monthTou.bt), fin(s.monthTou.td), fin(s.monthTou.cd));
}

// Dựng + gửi gói trạng thái (retained: màn hình mới bật là có số liệu ngay).
static void mqttPublishState() {
    char tbuf[24] = "--:--";
    struct tm tn;
    if (ntpSynced && getNow(tn)) snprintf(tbuf, sizeof(tbuf), "%02d:%02d %02d/%02d/%04d", tn.tm_hour, tn.tm_min, tn.tm_mday, tn.tm_mon + 1, tn.tm_year + 1900);

    int n = snprintf(mqttBuf, sizeof(mqttBuf),
        "{\"v\":1,\"fw\":\"%s\",\"t\":\"%s\",\"ntp\":%d,\"up\":%lu,\"rssi\":%d,\"alarm\":%d,"
        "\"uMin\":%.0f,\"uMax\":%.0f,\"i1\":%.0f,\"i2\":%.0f,\"m1\":",
        FW_VERSION, tbuf, ntpSynced ? 1 : 0, (millis() - bootTime) / 1000UL, (int)WiFi.RSSI(), physicalAlarmActive ? 1 : 0,
        alertUMin, alertUMax, alertIMaxM1, alertIMaxM2);
    if (n <= 0 || n >= (int)sizeof(mqttBuf)) return;
    n += mqttMeterJson(mqttBuf + n, sizeof(mqttBuf) - n, m1, statM1, m1PhaseLossGlobal, alertIMaxM1);
    if (n >= (int)sizeof(mqttBuf) - 20) return;
    n += snprintf(mqttBuf + n, sizeof(mqttBuf) - n, ",\"m2\":");
    n += mqttMeterJson(mqttBuf + n, sizeof(mqttBuf) - n, m2, statM2, m2PhaseLossGlobal, alertIMaxM2);
    if (n >= (int)sizeof(mqttBuf) - 4) return;     // tràn bộ đệm -> bỏ gói này, không gửi JSON cụt
    n += snprintf(mqttBuf + n, sizeof(mqttBuf) - n, "}");
    mqttClient.publish(MQTT_T_STATE, (const uint8_t *)mqttBuf, n, true);
}

static void mqttTask(void *) {
    for (;;) {
        bool wifiUp = (currentNet == NET_WIFI && WiFi.status() == WL_CONNECTED && !otaRunning);
        if (wifiUp) {
            if (mqttClient.connected()) {
                mqttClient.loop();
                if (millis() - mqttLastPub >= MQTT_PUBLISH_MS) {
                    mqttLastPub = millis();
                    mqttPublishState();
                }
            } else if ((mqttLastTry == 0 || millis() - mqttLastTry >= MQTT_RETRY_MS) && ESP.getFreeHeap() >= MQTT_MIN_HEAP) {
                mqttLastTry = millis();
                String cid = "tramdien-" + String((uint32_t)ESP.getEfuseMac(), HEX);
                // LWT: nếu trạm đứt kết nối đột ngột broker tự phát "offline" cho màn hình biết
                if (mqttClient.connect(cid.c_str(), MQTT_USER, MQTT_PASS, MQTT_T_STATUS, 1, true, "offline")) {
                    mqttClient.publish(MQTT_T_STATUS, "online", true);
                    mqttPublishState();
                    mqttLastPub = millis();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void mqttSetup() {
#if MQTT_USE_TLS
    mqttNet.setInsecure();            // có mã hoá nhưng không kiểm tra chứng chỉ máy chủ
    mqttNet.setHandshakeTimeout(8);
#endif
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setBufferSize(2048);
    mqttClient.setSocketTimeout(5);
    mqttClient.setKeepAlive(30);
    xTaskCreatePinnedToCore(mqttTask, "mqtt", 10240, NULL, 1, NULL, 0);
}
#else
static void mqttSetup() {}
#endif

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    delay(100);
    randomSeed(esp_random());

    pinMode(RS485_DE_RE_PIN, OUTPUT); digitalWrite(RS485_DE_RE_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT);      digitalWrite(BUZZER_PIN, LOW);   // [ALARM]
    rs485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    loadWifiCredentials();
    netPrefs.begin("net_cfg", true);   // [SELF-HEAL] nạp lại số lần đã tự restart do mất mạng (lũy kế qua các lần restart)
    netRestartCount = netPrefs.getInt("netRstCnt", 0);
    netPrefs.end();
    loadTelegramCredentials();
    loadAlertConfig();
    loadTariffFromNVS();      // [PRICE]
    loadUsersFromNVS();       // [MULTIUSER]
    loadEnergyStat("m1_", statM1);
    loadEnergyStat("m2_", statM2);

    bot.updateToken(currentBotToken);

    setupNetwork();   // [NETMGR] chỉ còn WiFi STA — không còn phát AP
    mqttSetup();      // [MQTT-DISPLAY v1.9.3] khởi động task đẩy số liệu cho màn hình (không chặn loop)

    esp_task_wdt_config_t cfg = { .timeout_ms = WDT_TIMEOUT_SEC * 1000, .idle_core_mask = 0, .trigger_panic = true };
    esp_err_t e = esp_task_wdt_init(&cfg);
    if (e == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&cfg);
    esp_task_wdt_add(NULL);

    bot.longPoll = 0;
    bootTime = millis();
    lastMeterPoll = 0;
    lastTelegramPoll = millis();
    lastEnergyCheckpoint = millis();
}

void loop() {
    esp_task_wdt_reset();
    ensureNetwork();   // [NETMGR] duy trì/tự kết nối lại WiFi STA — không blocking lâu
    phaseServer.handleClient();   // [PHASE-LINK v1.9.2] phục vụ request /phase_status từ Master, không blocking
    checkNetworkHealth();   // [SELF-HEAL] tự restart ESP32 nếu mất mạng quá lâu
    checkHeapHealth();      // [SELF-HEAL] restart phòng ngừa nếu RAM tụt thấp bất thường
    checkHeapTrend();       // [HEAP-TREND] cảnh báo sớm nếu RAM giảm dần đều (nghi rò rỉ)

    // [NTP-FIX2] Kiểm tra đồng bộ giờ ĐỘC LẬP với trạng thái công tơ — trước
    // đây ntpSynced chỉ được set bên trong updateEnergyRollover(), mà hàm đó
    // chỉ chạy khi công tơ online, nên mất kết nối công tơ thì /status luôn
    // báo "chưa đồng bộ" dù giờ hệ thống thực ra đã có.
    if (!ntpSynced) {
        struct tm tCheck;
        if (getNow(tCheck)) ntpSynced = true;
    }

    if (millis() - lastMeterPoll >= METER_POLL_MS) {
        lastMeterPoll = millis();
        pollMeters();
        bool changed1 = false, changed2 = false;
        if (m1.online) accumulateTou(statM1, m1.EnergyTotal, lastPollEnergyM1); else lastPollEnergyM1 = -1;
        if (m2.online) accumulateTou(statM2, m2.EnergyTotal, lastPollEnergyM2); else lastPollEnergyM2 = -1;
        if (m1.online) updateEnergyRollover(statM1, m1.EnergyTotal, "m1_", "Moong 1", changed1);
        if (m2.online) updateEnergyRollover(statM2, m2.EnergyTotal, "m2_", "Moong 2", changed2);
        checkAlerts();
    }

    if (millis() - lastEnergyCheckpoint >= ENERGY_CHECKPOINT_MS) {
        lastEnergyCheckpoint = millis();
        if (m1.online) saveEnergyStat("m1_", statM1);
        if (m2.online) saveEnergyStat("m2_", statM2);
    }

    // [ALARM] Cập nhật còi vật lý mỗi vòng lặp.
    digitalWrite(BUZZER_PIN, physicalAlarmActive ? HIGH : LOW);

    // [REPORT] Gửi báo cáo tự động đúng giờ cấu hình, 1 lần/ngày.
    if (ntpSynced) {
        struct tm t;
        if (getNow(t) && t.tm_hour == dailyReportHour && t.tm_min < DAILY_REPORT_WINDOW_MIN && lastReportSentDay != t.tm_yday) {
            sendTelegramDirect(buildDailyReport());
            lastReportSentDay = t.tm_yday;
        }
    }

    // [REBOOT] Tự khởi động lại đúng giờ cấu hình, 1 lần/ngày — không chạy
    // khi đang OTA để tránh làm gián đoạn quá trình cập nhật firmware.
    if (ntpSynced && dailyRebootEnabled && !otaRunning) {
        struct tm tr;
        if (getNow(tr) && tr.tm_hour == dailyRebootHour && tr.tm_min < DAILY_REPORT_WINDOW_MIN && lastRebootDay != tr.tm_yday) {
            lastRebootDay = tr.tm_yday;   // đặt trước khi gọi, vì hàm dưới sẽ restart nên không quay lại đây nữa
            performDailyReboot();
        }
    }

    if (!otaRunning && currentNet != NET_NONE && millis() - lastTelegramPoll >= TELEGRAM_POLL_MS) {
        lastTelegramPoll = millis();
        esp_task_wdt_delete(NULL);
        int n = bot.getUpdates(bot.last_message_received + 1);
        while (n) { handleNewMessages(n, bot); n = bot.getUpdates(bot.last_message_received + 1); }
        esp_task_wdt_add(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}
