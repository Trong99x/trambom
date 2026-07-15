/*****************************************************************
 * SLAVE TRAM HO NODE — ESP32 + SX1278
 * VERSION 1.2 — BẢN HOÀN THIỆN AN TOÀN CHỐNG TỰ BẬT BƠM
 *
 * CHUC NANG:
 * - Nhan goi MSG_COMMAND tu Master (payload = 0-100 = % toc do)
 * - Tu dong luu cong suat dat tu Telegram vao Flash (Preferences)
 * - Tu dong khoi phuc lai toc do truoc do neu mat dien
 * - Chuyen doi % toc do thanh dien ap 0-10V qua DAC noi (GPIO25)
 * - Gui ACK xac nhan va dập tắt nhiễu, bảo vệ biến tần an toàn.
 *****************************************************************/

#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>     
#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Khởi tạo đối tượng lưu trữ bộ nhớ
Preferences prefs;

//======================================================
// DINH DANH NODE
//======================================================
#define MASTER_ID    0
#define SLAVE_M1_ID  1
#define SLAVE_M2_ID  2
#define SLAVE_H_ID   3

//======================================================
// CHAN GPIO
//======================================================
#define DAC_PIN         25      
#define LED_PIN          2      
#define RELAY_ENABLE_PIN 22     
// [FIX] Chân đọc tiếp điểm phụ (NO) của relay — xác nhận VẬT LÝ relay
// đã thực sự đóng, không chỉ là lệnh MCU đã ra. Đấu: COM->GND, NO->chân
// này, dùng INPUT_PULLUP (relay đóng = LOW, relay mở = HIGH thả nổi).
// Nếu tủ điện đấu theo NC hoặc COM nối dương thay vì GND, đảo logic tại
// hàm readRelayFeedbackRaw().
#define RELAY_FEEDBACK_PIN 27

//======================================================
// CHAN SPI / LORA
//======================================================
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
#define LORA_SS      5
#define LORA_RST    14
#define LORA_DIO0    4      

//======================================================
// THONG SO LORA
//======================================================
#define LORA_FREQ   433E6
#define LORA_BW     125E3
#define LORA_SF     12
#define LORA_CR      5
#define TX_POWER    20

//======================================================
// TIMING
//======================================================
#define HEARTBEAT_MS              60000UL
#define STAGGER_PER_ID_MS         15000UL
#define HEARTBEAT_JITTER_MS        3000UL
#define ACK_WAIT_MS                8000UL
#define RETRY_INTERVAL_MS          3000UL
#define MAX_RETRIES                     3
#define WDT_TIMEOUT_SEC                30
#define SIGNAL_LOST_TIMEOUT_MS    180000UL   
// [FIX] Thời gian tín hiệu phải ổn định liên tục trước khi được tin —
// chống nhiễu điện từ VFD gây đọc sai tiếp điểm relay.
#define RELAY_FEEDBACK_DEBOUNCE_MS    50UL

//======================================================
// THONG SO DAC & RAMP
//======================================================
#define DAC_FULL_SCALE      255     
#define DAC_ZERO            0       
#define RAMP_STEP_MS        50UL    
#define RAMP_STEP_SIZE       2      

//======================================================
// KIEU GOI TIN
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
// BIEN TOAN CUC
//======================================================
uint16_t txSeq = 0;
uint8_t  targetSpeed        = 0;    
uint8_t  currentSpeed       = 0;    
uint8_t  telegramSavedSpeed = 50;   

// ACK tracking
bool          waitingAck    = false;
uint16_t      pendingSeq    = 0;
uint8_t       retryCount    = 0;
unsigned long lastSentTime  = 0;
Packet        pendingPkt;

// Timing
unsigned long lastHeartbeat     = 0;
unsigned long heartbeatInterval = HEARTBEAT_MS;   
unsigned long lastCmdReceived   = 0;   
unsigned long lastRampStep      = 0;

bool vfdEnabled = false;    

// [FIX] Trạng thái relay VẬT LÝ đọc từ tiếp điểm phụ (đã debounce),
// dùng để gửi lên Master thay cho việc chỉ echo lại biến phần mềm
// vfdEnabled như trước.
bool          relayFeedbackRaw        = false;
bool          relayFeedbackDebounced  = false;
unsigned long relayFeedbackChangeTime = 0;

// [FIX] Bit cao nhất của payload ACK/HEARTBEAT mang trạng thái relay
// VẬT LÝ (đọc từ tiếp điểm phụ NO, đã debounce) — cho phép Master xác
// nhận relay thật sự đã đóng/mở, không chỉ là MCU đã ra lệnh.
#define RELAY_STATE_BIT 0x80

//======================================================
// DOC TIEP DIEM PHU (FEEDBACK) - XAC NHAN RELAY THAT
//======================================================
// Đọc điện áp thô trên chân feedback và quy đổi theo đấu dây giả định
// (COM->GND, NO->GPIO, INPUT_PULLUP): LOW = relay đóng (ON).
// Đảo dấu "==" thành "!=" tại đây nếu tủ điện đấu ngược cực/dùng NC.
bool readRelayFeedbackRaw() {
    return (digitalRead(RELAY_FEEDBACK_PIN) == LOW);
}

// Chỉ chấp nhận giá trị mới sau khi tín hiệu đã ổn định liên tục đủ
// RELAY_FEEDBACK_DEBOUNCE_MS — lọc nhiễu xung từ trường VFD/khởi động
// động cơ vốn đã biết là nguồn nhiễu chính trong tủ này.
void updateRelayFeedback() {
    bool raw = readRelayFeedbackRaw();
    unsigned long now = millis();
    if (raw != relayFeedbackRaw) {
        relayFeedbackRaw        = raw;
        relayFeedbackChangeTime = now;
    }
    if (now - relayFeedbackChangeTime > RELAY_FEEDBACK_DEBOUNCE_MS) {
        relayFeedbackDebounced = raw;
    }
}

//======================================================
// DIEU KHIEN DAC 0-10V
//======================================================
void setDacPercent(uint8_t pct) {
    if (pct > 100) pct = 100;
    uint8_t dacVal = (uint8_t)((uint32_t)pct * DAC_FULL_SCALE / 100);
    dacWrite(DAC_PIN, dacVal);
}

//======================================================
// RELAY ENABLE BIEN TAN
//======================================================
void setVfdRelay(bool enable) {
    if (enable == vfdEnabled) return;
    vfdEnabled = enable;
    digitalWrite(RELAY_ENABLE_PIN, enable ? HIGH : LOW);
    Serial.printf("[VFD] Relay %s\n", enable ? "ON" : "OFF");
}

//======================================================
// AP DUNG TOC DO MOI (Đã sửa lỗi loại bỏ Over-run kích nhầm)
//======================================================
void applySpeed(uint8_t pct) {
    if (pct > 100) pct = 100;
    targetSpeed = pct;

    if (pct == 0) {
        // KHÓA AN TOÀN TUYỆT ĐỐI: Đưa tất cả trạng thái về 0 ngay lập tức
        currentSpeed = 0;   
        setDacPercent(0);   
        setVfdRelay(false); 
        Serial.println("[VFD] TẮT KHẨN CẤP: Ngắt Relay kích và hạ DAC về 0V dứt điểm.");
    } else {
        if (currentSpeed == 0) {
            uint8_t firstStep = (RAMP_STEP_SIZE < pct) ? RAMP_STEP_SIZE : pct;
            currentSpeed = firstStep;
            setDacPercent(currentSpeed);
            lastRampStep = millis();
            delay(20);   
        }
        setVfdRelay(true);
        Serial.printf("[VFD] Kích Relay hoạt động -> Ramp lên %d%%\n", pct);
    }
}

//======================================================
// XU LY TANG GIAM TOC DO TU TU (RAMP)
//======================================================
void processRamp() {
    if (currentSpeed == targetSpeed) return;
    if (millis() - lastRampStep < RAMP_STEP_MS) return;

    lastRampStep = millis();

    if (currentSpeed < targetSpeed) {
        uint8_t step = RAMP_STEP_SIZE;
        if (currentSpeed + step > targetSpeed) step = targetSpeed - currentSpeed;
        currentSpeed += step;
    } else {
        uint8_t step = RAMP_STEP_SIZE;
        if (currentSpeed < step || currentSpeed - step < targetSpeed)
            step = currentSpeed - targetSpeed;
        currentSpeed -= step;
    }

    setDacPercent(currentSpeed);
}

//======================================================
// KHOI TAO LORA (Đã bật CRC chống nhiễu tủ điện)
//======================================================
bool initLoRa() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[LORA] INIT FAIL!");
        return false;
    }
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setCodingRate4(LORA_CR);
    
    // BẮT BUỘC: Phải bật CRC để chip SX1278 tự động hủy toàn bộ gói rác do nhiễu từ trường VFD sinh ra
    LoRa.enableCrc(); 
    
    LoRa.setTxPower(TX_POWER);
    Serial.println("[LORA] Khởi tạo LoRa thành công (Bảo vệ CRC ENABLED).");
    return true;
}

//======================================================
// WATCHDOG TONG CHONG TREO MẠCH
//======================================================
void initWatchdog() {
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_err_t e = esp_task_wdt_init(&cfg);
    if (e == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&cfg);
    esp_task_wdt_add(NULL);
}

void blinkLed(int times, int ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH); delay(ms / 2);
        digitalWrite(LED_PIN, LOW);  delay(ms / 2);
    }
}

bool sendPacket(Packet &pkt) {
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pkt, sizeof(Packet));
    return LoRa.endPacket();
}

// [FIX] Payload mang currentSpeed (7 bit thấp, đủ cho 0-100) và trạng
// thái relay VẬT LÝ đã debounce (bit cao nhất) — đọc từ tiếp điểm phụ
// thật, không phải suy luận/echo lại biến phần mềm vfdEnabled.
uint8_t buildStatusPayload() {
    return (uint8_t)((relayFeedbackDebounced ? RELAY_STATE_BIT : 0) | (currentSpeed & 0x7F));
}

void sendAck(uint8_t receiver, uint16_t seq) {
    Packet pkt = { (uint8_t)SLAVE_H_ID, receiver, (uint8_t)MSG_ACK, seq, buildStatusPayload(), (uint32_t)millis() };
    sendPacket(pkt);
}

void sendHeartbeat() {
    Packet pkt = { (uint8_t)SLAVE_H_ID, (uint8_t)MASTER_ID, (uint8_t)MSG_HEARTBEAT, txSeq++, buildStatusPayload(), (uint32_t)millis() };
    sendPacket(pkt);
}

//======================================================
// XU LY GOI LORA DEN
//======================================================
void processIncoming() {
    int sz = LoRa.parsePacket();
    if (sz == 0) return;

    if (sz != sizeof(Packet)) {
        while (LoRa.available()) LoRa.read();
        return;
    }

    Packet pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(Packet));

    if (pkt.receiver != SLAVE_H_ID && pkt.receiver != 255) return;

    switch (pkt.msgType) {
        case MSG_COMMAND:
            if (pkt.sender == MASTER_ID) {
                uint8_t newSpeed = pkt.payload;
                if (newSpeed > 100) newSpeed = 100;

                blinkLed(1, 100);

                // --- LOGIC KỊCH BẢN A: PHÂN TÍCH VÀ LƯU FLASH CHỐNG MẤT ĐIỆN ---
                if (newSpeed > 0 && newSpeed < 100) {
                    if (newSpeed != telegramSavedSpeed) {
                        telegramSavedSpeed = newSpeed;
                        
                        prefs.begin("vfd_settings", false);
                        prefs.putUChar("saved_speed", telegramSavedSpeed);
                        prefs.end();
                        Serial.printf("[STORAGE] Đã lưu công suất Telegram mới vào bộ nhớ: %d%%\n", telegramSavedSpeed);
                    }
                }

                // Thực thi điều khiển biến tần
                if (newSpeed != targetSpeed || (newSpeed == 0 && vfdEnabled)) {
                    Serial.printf("[LỆNH] Chuyển đổi tốc độ: %d%% -> %d%%\n", targetSpeed, newSpeed);
                    applySpeed(newSpeed);
                }

                // [FIX] Gửi ACK SAU khi relay/DAC đã được áp dụng. Payload
                // dùng relayFeedbackDebounced (đọc tại thời điểm gửi) — có
                // thể ACK NGAY sau lệnh vẫn còn phản ánh trạng thái CŨ, vì
                // relay cơ khí cần vài-vài chục ms để đóng/mở tiếp điểm và
                // debounce cần thêm RELAY_FEEDBACK_DEBOUNCE_MS ổn định.
                // Đây là độ trễ vật lý thật, không phải lỗi logic — Master
                // đã có thời gian ân hạn (RELAY_H_CONFIRM_GRACE_MS = 10s)
                // đủ lớn để bỏ qua khoảng trễ này.
                sendAck(MASTER_ID, pkt.seq);

                lastCmdReceived = millis();
            }
            break;

        case MSG_HEARTBEAT:
            if (pkt.sender == MASTER_ID) {
                sendAck(MASTER_ID, pkt.seq);
                lastCmdReceived = millis();   
            }
            break;

        case MSG_ACK:
            if (waitingAck && pkt.seq == pendingSeq) {
                waitingAck = false;
                retryCount = 0;
            }
            break;

        default:
            break;
    }
}

//======================================================
// BẢO VỆ MẤT TÍN HIỆU
//======================================================
void checkSignalLost() {
    if (lastCmdReceived == 0) return;   
    if (millis() - lastCmdReceived > SIGNAL_LOST_TIMEOUT_MS) {
        if (targetSpeed != 0 || vfdEnabled) {
            Serial.println("[BẢO VỆ] MẤT SÓNG LORA QUÁ 3 PHÚT -> TẮT KHẨN CẤP!");
            applySpeed(0);
        }
    }
}

void processRetry() {
    if (!waitingAck) return;
    if (millis() - lastSentTime < RETRY_INTERVAL_MS) return;

    if (retryCount >= MAX_RETRIES) {
        waitingAck = false;
        return;
    }

    retryCount++;
    lastSentTime     = millis();
    pendingPkt.uptime = (uint32_t)millis();
    sendPacket(pendingPkt);
}

void updateStatusLed() {
    static unsigned long lastLed = 0;
    unsigned long interval = (currentSpeed > 0) ? 200 : 1500;
    if (millis() - lastLed > interval) {
        lastLed = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

//======================================================
// SETUP SYSTEM
//======================================================
void setup() {
    // ĐÃ XÓA BỎ lệnh tắt Brown-out để đảm bảo ESP32 tự reset an toàn nếu sụt áp lưới thay vì chạy lỗi logic.
    Serial.begin(115200);
    Serial.println("\n=== SLAVE TRAM HO (H) — V1.2 FIXED ===");

    pinMode(LED_PIN, OUTPUT);
    
    // Khởi tạo kéo chân Relay về Thấp ngay lập tức tránh trạng thái thả nổi lúc boot
    pinMode(RELAY_ENABLE_PIN, OUTPUT);
    digitalWrite(RELAY_ENABLE_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    // [FIX] Chân đọc tiếp điểm phụ relay — INPUT_PULLUP vì đấu COM->GND.
    pinMode(RELAY_FEEDBACK_PIN, INPUT_PULLUP);
    // Đọc giá trị ban đầu ngay để tránh báo sai "OFF" giả trong lúc chờ
    // debounce ổn định ở vòng loop đầu tiên.
    relayFeedbackRaw = relayFeedbackDebounced = readRelayFeedbackRaw();
    relayFeedbackChangeTime = millis();

    // Xuất áp ban đầu bằng 0V
    dacWrite(DAC_PIN, DAC_ZERO);

    // --- ĐỌC LẠI DỮ LIỆU CŨ KHI KHỞI ĐỘNG (CHỐNG MẤT ĐIỆN) ---
    prefs.begin("vfd_settings", true); 
    telegramSavedSpeed = prefs.getUChar("saved_speed", 50); 
    prefs.end();
    Serial.printf("[STORAGE] Khôi phục thành công công suất Telegram cũ: %d%%\n", telegramSavedSpeed);

    if (!initLoRa()) {
        while (true) { blinkLed(3, 300); delay(1000); }
    }

    initWatchdog();
    randomSeed(esp_random());   

    unsigned long staggerOffset = (unsigned long)SLAVE_H_ID * STAGGER_PER_ID_MS;
    lastHeartbeat = millis() - staggerOffset;

    // Giữ an toàn công nghiệp: Mới cấp điện lại luôn giữ trạng thái 0% để đợi lệnh Master đồng bộ.
    lastCmdReceived = millis(); 
    Serial.println("[SETUP] Sẵn sàng nhận lệnh điều khiển phối hợp.");
}

//======================================================
// LOOP SYSTEM
//======================================================
void loop() {
    esp_task_wdt_reset();

    updateRelayFeedback();   // [FIX] Cập nhật trạng thái relay vật lý (debounced) TRƯỚC khi xử lý gói, để ACK dùng giá trị mới nhất
    processIncoming();
    processRamp();
    checkSignalLost();
    processRetry();

    if (millis() - lastHeartbeat > heartbeatInterval) {
        sendHeartbeat();
        lastHeartbeat = millis();
        long jitter = random(-(long)HEARTBEAT_JITTER_MS, (long)HEARTBEAT_JITTER_MS + 1);
        heartbeatInterval = HEARTBEAT_MS + jitter;
    }

    updateStatusLed();
    delay(10); 
}
