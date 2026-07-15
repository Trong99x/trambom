/*****************************************************************
 * SLAVE MOONG 1 NODE — ESP32 + SX1278
 * VERSION 1.2 — BAT CRC + GIAM BLOCKING + LECH PHA HEARTBEAT
 *
 * FILE NAY DA CO DINH SLAVE_ID = 1 (Moong 1) — nap thang, khong can
 * chinh gi them. File Moong 2 la mot file rieng (slave_m2_v1_2.ino).
 *
 * SUA TU V1.1:
 * - LoRa.enableCrc() bat trong initLoRa() -> tu dong loai goi tin
 *   loi/nhieu, khop voi Master V14.2.
 * - sendPacket(): doi LoRa.endPacket() (BLOCKING, cho het thoi gian
 *   phat song ~1-1.5s moi return) -> LoRa.endPacket(true) (ASYNC,
 *   giong Master), tra ve ngay lap tuc. Nho vay loop() khong bi
 *   "dieu" khi dang gui, giam kha nang bo lo goi RX toi dung luc do.
 * - processIncoming(): bo delay(200) BLOCKING truoc khi gui floatReply
 *   khi nhan Heartbeat tu Master. Thay bang co hen gio khong-blocking
 *   (pendingHeartbeatReply / heartbeatReplyAt), xu ly trong loop().
 *
 * SUA TU V1.0:
 * - readFloat(): DAO LAI chieu doc phao cho KHOP 100% voi Master.
 *   Truoc day Slave doc NGUOC chieu voi Master (LOW=FULL o Slave,
 *   nhung LOW=LOW/can o Master) -> gay bao sai trang thai muc nuoc,
 *   khien bom Moong chay SAI (bom khi day, ngung khi can).
 *   Da xac nhan phao vat ly o Moong 1/2 CUNG LOAI & CUNG KIEU LAP
 *   voi phao o be Master -> quy uoc phai giong het nhau:
 *     Chan LOW  => FLOAT_LOW  (Can)
 *     Chan HIGH => FLOAT_FULL (Day)
 *
 * CHUC NANG (giu nguyen tu V1.0):
 * - Doc cam bien phao muc nuoc (INPUT_PULLUP, debounce 5s)
 * - Gui trang thai phao (MSG_FLOAT) len Master khi co thay doi
 * - Gui heartbeat dinh ky moi 60 giay
 * - Nhan ACK tu Master, xac nhan goi da toi
 * - Tu dong retry neu khong nhan ACK trong thoi gian cho
 * - Nhan lenh Heartbeat tu Master, phan hoi biet minh con song
 * - Watchdog chong treo cung
 *
 * CAU HINH:
 * - Thay SLAVE_ID thanh SLAVE_M1_ID (1) hoac SLAVE_M2_ID (2)
 * - Thay FLOAT_PIN neu can
 *
 * PHAP GIO THONG TIN:
 * - Khi phao thay doi: gui MSG_FLOAT ngay lap tuc (sau debounce)
 * - Dinh ky HEARTBEAT_MS: gui MSG_HEARTBEAT de bao lam sang
 * - Neu khong nhan ACK sau ACK_WAIT_MS: retry toi da MAX_RETRIES lan
 *
 * GIAO THUC LOAI PACKET (phai khop 100% voi Master V14.0/V14.1):
 *   MSG_HEARTBEAT = 1
 *   MSG_FLOAT     = 2  (payload: FLOAT_FULL=1 / FLOAT_LOW=0)
 *   MSG_COMMAND   = 3
 *   MSG_ACK       = 4
 *****************************************************************/

#include <SPI.h>
#include <LoRa.h>
#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//======================================================
// *** NODE NAY CO DINH LA MOONG 1 ***
//======================================================
#define SLAVE_ID   1

//======================================================
// DINH DANH HE THONG (phai khop voi Master)
//======================================================
#define MASTER_ID    0
#define SLAVE_M1_ID  1
#define SLAVE_M2_ID  2
#define SLAVE_H_ID   3

//======================================================
// CHAN PHAO & LED TRANG THAI
//======================================================
#define FLOAT_PIN    32     // INPUT_PULLUP, phao NO: HIGH=day, LOW=can
#define LED_PIN       2     // LED onboard ESP32 (active HIGH)

//======================================================
// CHAN SPI / LORA (giong Master)
//======================================================
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
#define LORA_SS      5
#define LORA_RST    14
#define LORA_DIO0   26

//======================================================
// THONG SO LORA (phai khop 100% voi Master)
//======================================================
#define LORA_FREQ   433E6
#define LORA_BW     125E3
#define LORA_SF     12
#define LORA_CR      5
#define TX_POWER    20

//======================================================
// TIMING
//======================================================
#define HEARTBEAT_MS        60000UL   // Gui heartbeat moi 60 giay
// FIX: chia le thoi diem heartbeat theo SLAVE_ID, tranh dung do voi
// Master/cac tram khac tren cung kenh LoRa (nguyen nhan gay "mat ket noi"
// lap lai dinh ky khi cac board cung bat nguon 1 luc sau mat dien).
// 4 "nguoi phat" chia deu 60s: Master=0s, M1=15s, M2=30s, H=45s.
#define STAGGER_PER_ID_MS   15000UL
// FIX: jitter ngau nhien moi chu ky heartbeat, phong truong hop 1 node
// tu reset rieng le (brown-out/watchdog) lam mat offset da lech truoc do
// va vo tinh trung pha tro lai voi node khac.
#define HEARTBEAT_JITTER_MS  3000UL
#define FLOAT_DEBOUNCE_MS    5000UL   // Debounce phao 5 giay
#define ACK_WAIT_MS          8000UL   // Cho ACK toi da 8 giay (> SF12 ~1.2s x vai lan retry Master)
#define RETRY_INTERVAL_MS    3000UL   // Khoang cach giua cac lan retry
#define MAX_RETRIES              5    // So lan thu lai toi da
#define WDT_TIMEOUT_SEC         30    // Watchdog 30 giay
#define LED_BLINK_OK_MS        200UL  // Nhay LED khi gui thanh cong
#define LED_BLINK_ERR_MS      1000UL  // Nhay LED khi loi

//======================================================
// GIA TRI PHAO
//======================================================
#define FLOAT_FULL  1
#define FLOAT_LOW   0

//======================================================
// KIEU GOI TIN (phai khop voi Master V14.0/V14.1)
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

// Trang thai phao
uint8_t       currentFloat    = FLOAT_FULL;
uint8_t       rawFloat        = FLOAT_FULL;
unsigned long floatChangeTime = 0;
bool          floatPending    = false;   // Co goi FLOAT chua gui / chua duoc ACK

// Trang thai ACK
bool          waitingAck      = false;
uint16_t      pendingSeq      = 0;
uint8_t       retryCount      = 0;
unsigned long lastSentTime    = 0;
Packet        pendingPkt;               // Goi dang cho ACK

// Timing
unsigned long lastHeartbeat      = 0;
unsigned long heartbeatInterval  = HEARTBEAT_MS;   // FIX: co the doi moi chu ky (jitter)

// Hen gio gui floatReply sau khi nhan Heartbeat tu Master (khong-blocking)
bool          pendingHeartbeatReply = false;
unsigned long heartbeatReplyAt      = 0;

//======================================================
// KHOI TAO LORA
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
    LoRa.setTxPower(TX_POWER);
    LoRa.enableCrc();   // MOI: tu dong loai bo goi tin loi (bit-flip do nhieu)
    Serial.printf("[LORA] READY — SF%d BW%.0fkHz Node ID=%d CRC:ON\n",
                  LORA_SF, LORA_BW/1000.0, SLAVE_ID);
    return true;
}

//======================================================
// KHOI TAO WATCHDOG
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

//======================================================
// LED HELPER
//======================================================
void blinkLed(int times, unsigned long ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(ms / 2);
        digitalWrite(LED_PIN, LOW);
        delay(ms / 2);
    }
}

//======================================================
// GUI GOI LORA (blocking — Slave don luong, khong can mutex)
//======================================================
bool sendPacket(Packet &pkt) {
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pkt, sizeof(Packet));
    bool ok = LoRa.endPacket(true);   // ASYNC (giong Master) — khong block cho xong TX,
                                       // giam kha nang bo lo goi RX toi dung luc dang gui
    if (ok) {
        Serial.printf("[LORA] TX: type=%d seq=%d payload=%d\n",
                      pkt.msgType, pkt.seq, pkt.payload);
    } else {
        Serial.println("[LORA] TX FAIL");
    }
    return ok;
}

//======================================================
// GUI GOI FLOAT — dat vao hang cho ACK
//======================================================
void sendFloat(uint8_t floatState) {
    Packet pkt = {
        (uint8_t)SLAVE_ID,
        (uint8_t)MASTER_ID,
        (uint8_t)MSG_FLOAT,
        txSeq++,
        floatState,
        (uint32_t)millis()
    };

    if (sendPacket(pkt)) {
        pendingPkt    = pkt;
        pendingSeq    = pkt.seq;
        waitingAck    = true;
        retryCount    = 0;
        lastSentTime  = millis();
        blinkLed(1, LED_BLINK_OK_MS);
    }
    Serial.printf("[FLOAT] GUI: %s (seq=%d)\n",
                  floatState == FLOAT_LOW ? "CAN" : "DAY", pkt.seq);
}

//======================================================
// GUI HEARTBEAT
//======================================================
void sendHeartbeat() {
    // Heartbeat KHONG cho ACK (fire-and-forget), Master tu cap nhat lastSeen
    Packet pkt = {
        (uint8_t)SLAVE_ID,
        (uint8_t)MASTER_ID,
        (uint8_t)MSG_HEARTBEAT,
        txSeq++,
        currentFloat,          // payload = trang thai phao hien tai (tham khao)
        (uint32_t)millis()
    };
    sendPacket(pkt);
    Serial.printf("[HB] Gui heartbeat, float=%s\n",
                  currentFloat == FLOAT_LOW ? "CAN" : "DAY");
}

//======================================================
// GUI ACK VE MASTER (khi nhan duoc lenh/heartbeat tu Master)
//======================================================
void sendAck(uint8_t receiver, uint16_t seq) {
    Packet pkt = {
        (uint8_t)SLAVE_ID,
        receiver,
        (uint8_t)MSG_ACK,
        seq,
        0,
        (uint32_t)millis()
    };
    sendPacket(pkt);
}

//======================================================
// XU LY GOI NHAN TU MASTER
//======================================================
void processIncoming() {
    int sz = LoRa.parsePacket();
    if (sz == 0) return;

    if (sz != sizeof(Packet)) {
        while (LoRa.available()) LoRa.read();
        Serial.printf("[LORA] Goi rac %d bytes\n", sz);
        return;
    }

    Packet pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(Packet));

    int rssi = LoRa.packetRssi();

    // Chi xu ly goi gui cho minh hoac broadcast
    if (pkt.receiver != SLAVE_ID && pkt.receiver != 255) return;

    Serial.printf("[LORA] RX: sender=%d type=%d seq=%d RSSI=%ddBm\n",
                  pkt.sender, pkt.msgType, pkt.seq, rssi);

    switch (pkt.msgType) {
        case MSG_ACK:
            // Master xac nhan da nhan goi cua minh
            if (waitingAck && pkt.seq == pendingSeq && pkt.receiver == SLAVE_ID) {
                waitingAck   = false;
                floatPending = false;
                retryCount   = 0;
                blinkLed(2, LED_BLINK_OK_MS);
                Serial.printf("[ACK] Nhan ACK cho seq=%d sau %d lan gui\n",
                              pendingSeq, retryCount + 1);
            }
            break;

        case MSG_HEARTBEAT:
            // Master broadcast heartbeat — phan hoi bang 1 goi FLOAT
            // de Master biet minh con song va biet trang thai phao moi nhat
            if (pkt.sender == MASTER_ID) {
                sendAck(MASTER_ID, pkt.seq);
                // FIX V1.2: khong con delay(200) BLOCKING o day nua.
                // Danh dau can gui floatReply sau 200ms, xu ly trong loop()
                // de nhuong cho ACK gui truoc ma khong "dieu" toan bo may.
                pendingHeartbeatReply = true;
                heartbeatReplyAt      = millis() + 200;
            }
            break;

        case MSG_COMMAND:
            // Slave Moong khong co lenh dieu khien tu Master,
            // nhung van gui ACK de Master khong bi treo pending
            sendAck(MASTER_ID, pkt.seq);
            Serial.printf("[CMD] Nhan lenh khong ap dung (payload=%d), da gui ACK\n",
                          pkt.payload);
            break;

        default:
            break;
    }
}

//======================================================
// DOC VA DEBOUNCE PHAO
// FIX V1.1: DAO CHIEU cho khop 100% voi Master
//   Chan LOW  => FLOAT_LOW  (Can)
//   Chan HIGH => FLOAT_FULL (Day)
// (Truoc day V1.0 doc NGUOC: LOW=>FULL, HIGH=>LOW — SAI so voi
//  Master, gay dieu khien bom nguoc thuc te.)
//======================================================
void readFloat() {
    // Chân FLOAT_PIN noi voi phao (NO) xuong GND, dung INPUT_PULLUP
    // Quy uoc THONG NHAT voi Master:
    //   Pin = LOW  -> FLOAT_LOW  (Can)
    //   Pin = HIGH -> FLOAT_FULL (Day)
    uint8_t raw = (digitalRead(FLOAT_PIN) == LOW) ? FLOAT_LOW : FLOAT_FULL;

    if (raw != rawFloat) {
        rawFloat        = raw;
        floatChangeTime = millis();
    }

    if ((millis() - floatChangeTime >= FLOAT_DEBOUNCE_MS) && (raw != currentFloat)) {
        currentFloat = raw;
        floatPending = true;   // Danh dau can gui len Master
        Serial.printf("[FLOAT] Thay doi -> %s\n",
                      currentFloat == FLOAT_LOW ? "CAN" : "DAY");
    }
}

//======================================================
// XU LY RETRY ACK
//======================================================
void processRetry() {
    if (!waitingAck) return;

    unsigned long now = millis();

    // Kiem tra qua han
    if (now - lastSentTime < RETRY_INTERVAL_MS) return;

    if (retryCount >= MAX_RETRIES) {
        // Het retry — canh bao qua Serial, tiep tuc song
        // (khong co Telegram o Slave — Master se phat hien qua timeout)
        Serial.printf("[RETRY] THAT BAI sau %d lan. seq=%d. Master se tu timeout.\n",
                      MAX_RETRIES, pendingSeq);
        waitingAck   = false;
        floatPending = false;
        blinkLed(5, LED_BLINK_ERR_MS);
        return;
    }

    // Gui lai
    retryCount++;
    lastSentTime = now;
    pendingPkt.uptime = (uint32_t)now;  // Cap nhat uptime
    sendPacket(pendingPkt);
    Serial.printf("[RETRY] Lan %d/%d, seq=%d\n", retryCount, MAX_RETRIES, pendingSeq);
}

//======================================================
// SETUP
//======================================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    Serial.printf("\n=== SLAVE MOONG %d — V1.2 (CRC + GIAM BLOCKING) ===\n", SLAVE_ID);

    randomSeed(esp_random());   // FIX: seed cho jitter heartbeat ben duoi

    pinMode(FLOAT_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Doc trang thai phao ban dau (khong qua debounce)
    // Cung dung quy uoc moi: LOW => FLOAT_LOW (Can), HIGH => FLOAT_FULL (Day)
    rawFloat     = (digitalRead(FLOAT_PIN) == LOW) ? FLOAT_LOW : FLOAT_FULL;
    currentFloat = rawFloat;
    floatChangeTime = millis();

    if (!initLoRa()) {
        // Neu LoRa loi: nhay LED lien tuc, khong treo
        while (true) { blinkLed(3, 300); delay(1000); }
    }

    initWatchdog();

    // Bao Master biet Slave vua boot
    delay(500);
    sendFloat(currentFloat);

    // FIX: lech pha heartbeat theo SLAVE_ID so voi Master/cac tram khac
    // (xem giai thich o STAGGER_PER_ID_MS phia tren)
    unsigned long staggerOffset = (unsigned long)SLAVE_ID * STAGGER_PER_ID_MS;
    lastHeartbeat = millis() - staggerOffset;
    Serial.printf("[SETUP] Lech pha heartbeat: +%lus (ID=%d)\n",
                  staggerOffset / 1000UL, SLAVE_ID);
    Serial.printf("[SETUP] Hoan tat. Float=%s\n",
                  currentFloat == FLOAT_LOW ? "CAN" : "DAY");
}

//======================================================
// LOOP
//======================================================
void loop() {
    esp_task_wdt_reset();

    // 1. Doc phao + debounce
    readFloat();

    // 2. Neu phao moi thay doi va chua dang cho ACK → gui ngay
    if (floatPending && !waitingAck) {
        sendFloat(currentFloat);
    }

    // 3. Xu ly retry neu dang cho ACK qua han
    processRetry();

    // 4. Doc va xu ly goi tu Master
    processIncoming();

    // 4b. Gui floatReply da hen gio (thay cho delay(200) blocking cu, V1.2)
    if (pendingHeartbeatReply && millis() >= heartbeatReplyAt) {
        pendingHeartbeatReply = false;
        Packet floatReply = {
            (uint8_t)SLAVE_ID,
            (uint8_t)MASTER_ID,
            (uint8_t)MSG_FLOAT,
            txSeq++,
            currentFloat,
            (uint32_t)millis()
        };
        sendPacket(floatReply);
    }

    // 5. Heartbeat dinh ky (chi gui khi khong dang cho ACK)
    // FIX: dung heartbeatInterval (co jitter) thay vi HEARTBEAT_MS co dinh,
    // tranh dong pha lai voi node khac ve lau dai
    if (!waitingAck && millis() - lastHeartbeat > heartbeatInterval) {
        sendHeartbeat();
        lastHeartbeat = millis();
        long jitter = random(-(long)HEARTBEAT_JITTER_MS, (long)HEARTBEAT_JITTER_MS + 1);
        heartbeatInterval = HEARTBEAT_MS + jitter;
    }

    // Delay ngan, giam busy-loop, tiep ket nap lai WDT
    delay(50);
}
