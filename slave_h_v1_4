/*****************************************************************
 * SLAVE TRAM HO NODE — ESP32 + E220-400T30D (LLCC68)
 * VERSION 1.4 — POWER-SAVE: HA/TANG XUNG CPU THEO TRANG THAI BOM
 *
 * SUA TU V1.3:
 * - [POWER-SAVE] Slave nay chay pin du phong, KHONG giong slave M1/M2
 *   (chi doc phao, khong dieu khien gi) — tram H nay DIEU KHIEN BOM
 *   THAT qua bien tan (DAC 0-10V + relay kich), co logic ramp toc do
 *   tung buoc moi RAMP_STEP_MS=50ms -> CAN CPU du nhanh luc dang ramp,
 *   KHONG duoc ha xung co dinh nhu M1/M2.
 *   Ap dung kieu tuong tu ben Master: ha CPU xuong 80MHz CHI khi bom
 *   ho hoan toan dung yen (vfdEnabled==false && currentSpeed==0 &&
 *   targetSpeed==0 — ca 3 dieu kien de chac chan khong con ramp do
 *   dang), tang NGAY ve 240MHz khi co lenh chay/dang ramp. Xem
 *   updatePowerSaving(), goi trong loop() ngay sau processRamp() de
 *   luon doc dung trang thai ramp moi nhat cua chu ky do.
 *   Khong doi RAMP_STEP_MS/RAMP_STEP_SIZE hay bat ky logic dieu khien
 *   DAC/relay nao khac.
 *
 * VERSION 1.3 — FIX RACE CONDITION KHI KHOI TAO LORA (AUX TIMING)
 *
 * SUA TU V1.2:
 * - [FIX-LORA-AUX] Dong bo voi Master: enterConfigMode()/
 *   enterNormalMode() truoc day goi waitAuxHigh() NGAY sau khi doi
 *   chan M0/M1, co the doc AUX truoc khi module kip keo xuong LOW bao
 *   "dang ban" -> gui lenh cau hinh (0xC0...) khi module chua san sang
 *   nhan UART -> pushConfig() that bai gia, initLoRa() bao loi phan
 *   cung du day/nguon deu chuan. Them delay(20) settle truoc khi kiem
 *   tra AUX. Dong thoi pushConfig() chi in log hex phan hoi khi THAT
 *   BAI de de chan doan neu tai dien, khong lam ron Serial luc chay
 *   binh thuong.
 *
 * VERSION 1.2 — BẢN HOÀN THIỆN AN TOÀN CHỐNG TỰ BẬT BƠM
 *
 * CHUC NANG:
 * - Nhan goi MSG_COMMAND tu Master (payload = 0-100 = % toc do)
 * - Tu dong luu cong suat dat tu Telegram vao Flash (Preferences)
 * - Tu dong khoi phuc lai toc do truoc do neu mat dien
 * - Chuyen doi % toc do thanh dien ap 0-10V qua DAC noi (GPIO25)
 * - Gui ACK xac nhan va dập tắt nhiễu, bảo vệ biến tần an toàn.
 *****************************************************************/

//======================================================
// [E220-UPGRADE] DRIVER E220-400T30D (LLCC68) QUA UART
// Thay thế hoàn toàn lớp SPI/SX1278 cũ. Giữ nguyên tên đối tượng
// "LoRa" và các hàm begin/beginPacket/write/endPacket/parsePacket/
// available/read/readBytes/packetRssi để KHÔNG phải sửa lại toàn bộ
// sendPacket()/processLoRa()/initLoRa() phía dưới — chỉ đổi phần
// cấu hình chân + khối này.
//
// Giao thức cấu hình module (chế độ M0=1,M1=1 — xem datasheet EBYTE
// E220-400T30D, chip LLCC68, thanh ghi 0x00-0x07):
//   C0 <addr> <len> <data...>  = ghi thanh ghi (lưu Flash), phản hồi C1...
// Cấu hình áp dụng: ADDH/ADDL=0xFFFF (broadcast — định danh node vẫn
// nằm trong Packet.sender/receiver như code cũ, KHÔNG lọc địa chỉ ở
// lớp radio), UART 9600 8N1, air rate 2.4kbps (xa nhất), bật byte
// RSSI sau mỗi gói nhận, chế độ transparent, LBT tắt.
//
// Khung dữ liệu tự đóng gói: [SYNC 0xAA][LEN][DATA...][CRC8]
// (không dùng thanh ghi length của module) để tự bảo vệ khỏi lệch
// khung trên luồng UART; SX1278 cũ có CRC phần cứng theo từng gói SPI
// nên không cần lớp này — E220 chạy UART trong suốt (transparent)
// nên ta tự thêm CRC8 + đồng bộ khung ở đây.
//======================================================
#define LORA_UART_BAUD   9600
#define LORA_FRAME_SYNC  0xAA
#define LORA_MAX_FRAME   96   // du cho OtaPacket (79 byte) + SYNC/LEN/CRC8

class LoRaE220Class {
public:
    bool begin(uint32_t freqHz = 433125000UL) {
        pinMode(_m0, OUTPUT);
        pinMode(_m1, OUTPUT);
        pinMode(_aux, INPUT);
        _serial->begin(LORA_UART_BAUD, SERIAL_8N1, _rxPin, _txPin);
        delay(50);
        waitAuxHigh(2000);

        uint8_t channel = (uint8_t)(((freqHz / 1000000.0) - 410.125) + 0.5);

        enterConfigMode();
        bool ok = pushConfig(channel);
        enterNormalMode();
        return ok;
    }

    // Gán chân điều khiển M0/M1/AUX + cổng UART (thay cho SPI.begin +
    // LoRa.setPins(ss,rst,dio0) kiểu cũ).
    void setPins(uint8_t m0Pin, uint8_t m1Pin, uint8_t auxPin,
                 HardwareSerial &serialPort, uint8_t rxPin, uint8_t txPin) {
        _m0 = m0Pin; _m1 = m1Pin; _aux = auxPin;
        _serial = &serialPort; _rxPin = rxPin; _txPin = txPin;
    }

    // E220 dùng 1 tham số "air data rate" cố định (đã chọn 2.4kbps —
    // xa nhất — trong pushConfig()), không có BW/SF/CR riêng như
    // SX1278. Giữ hàm rỗng để các dòng gọi cũ vẫn biên dịch được mà
    // KHÔNG cần sửa từng chỗ gọi trong initLoRa()/tryInitLoRaOnce().
    void setSignalBandwidth(long) {}
    void setSpreadingFactor(int) {}
    void setCodingRate4(int) {}
    void enableCrc() {}  // module luôn CRC ở lớp vật lý; khung ta tự thêm CRC8 riêng

    void setTxPower(int dBm) {
        // E220-400T30D chỉ có 4 mức: 30/27/24/21 dBm — chọn mức gần
        // nhất không vượt quá giá trị yêu cầu.
        if (dBm >= 30)      _txPowerBits = 0b00;
        else if (dBm >= 27) _txPowerBits = 0b01;
        else if (dBm >= 24) _txPowerBits = 0b10;
        else                _txPowerBits = 0b11;
        if (_ready) { enterConfigMode(); pushConfig(_channel); enterNormalMode(); }
    }

    bool beginPacket() { _txLen = 0; return true; }

    size_t write(const uint8_t *buf, size_t len) {
        if (_txLen + len > sizeof(_txBuf)) return 0;
        memcpy(_txBuf + _txLen, buf, len);
        _txLen += len;
        return len;
    }

    bool endPacket(bool /*async*/ = false) {
        if (_txLen == 0) return false;
        uint8_t frame[LORA_MAX_FRAME];
        uint8_t fi = 0;
        frame[fi++] = LORA_FRAME_SYNC;
        frame[fi++] = _txLen;
        memcpy(frame + fi, _txBuf, _txLen); fi += _txLen;
        frame[fi++] = crc8(_txBuf, _txLen);

        // Chỉ đợi module RẢNH trước khi ghi (không block chờ phát
        // xong wireless) — giữ đúng ngữ nghĩa "bất đồng bộ" của
        // endPacket(true) trong code gốc.
        waitAuxHigh(50);
        _serial->write(frame, fi);
        return true;
    }

    // Gọi định kỳ trong task LoRa — hút hết byte UART đang chờ, tách
    // khung hợp lệ. Trả về kích thước payload nếu có gói mới, ngược
    // lại trả 0 (giữ đúng ngữ nghĩa parsePacket() cũ của thư viện SPI).
    int parsePacket() {
        while (_serial->available()) feed((uint8_t)_serial->read());
        if (_rxReady) { _rxReady = false; return _rxLen; }
        return 0;
    }

    int available() { return (int)_rxLen - (int)_rxReadPos; }

    int read() {
        if (_rxReadPos >= _rxLen) return -1;
        return _rxBuf[_rxReadPos++];
    }

    void readBytes(uint8_t *buf, size_t len) {
        size_t n = ((size_t)available() < len) ? (size_t)available() : len;
        memcpy(buf, _rxBuf + _rxReadPos, n);
        _rxReadPos += n;
    }

    int packetRssi() { return _lastRssi; }

private:
    uint8_t _m0 = 0, _m1 = 0, _aux = 0, _rxPin = 0, _txPin = 0;
    HardwareSerial *_serial = nullptr;
    bool    _ready = false;
    uint8_t _channel = 23;
    uint8_t _txPowerBits = 0; // 00 = 30dBm (mặc định)

    uint8_t _txBuf[LORA_MAX_FRAME]; uint8_t _txLen = 0;
    uint8_t _rxBuf[LORA_MAX_FRAME]; uint8_t _rxLen = 0; uint8_t _rxReadPos = 0;
    bool    _rxReady = false;
    int     _lastRssi = 0;

    enum RxState { WAIT_SYNC, WAIT_LEN, WAIT_DATA, WAIT_CRC, WAIT_RSSI };
    RxState _state = WAIT_SYNC;
    uint8_t _frameLen = 0, _frameIdx = 0;
    uint8_t _frameBuf[LORA_MAX_FRAME];

    void feed(uint8_t b) {
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
                    _state = WAIT_RSSI; // khung hợp lệ — module tự thêm 1 byte RSSI sau đó
                } else {
                    _state = WAIT_SYNC; // CRC sai — bỏ, tìm SYNC tiếp theo
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

    void waitAuxHigh(unsigned long timeoutMs) {
        unsigned long t0 = millis();
        while (digitalRead(_aux) == LOW) {
            if (millis() - t0 > timeoutMs) return;
            delay(2);
        }
        delay(2); // theo khuyến cáo datasheet: chờ thêm ~2ms sau cạnh lên AUX
    }

    // [FIX-LORA-AUX] Them delay(20) settle ngay sau khi doi chan M0/M1,
    // truoc khi kiem tra AUX - tranh race condition: module can vai ms
    // de THUC SU bat dau chuyen mode va keo AUX xuong LOW bao "dang
    // ban"; neu doc AUX qua nhanh se tuong nham la da san sang va gui
    // lenh cau hinh khi module chua kip nhan UART (dong bo voi Master,
    // xem changelog FIX-LORA-AUX).
    void enterConfigMode() {
        digitalWrite(_m0, HIGH); digitalWrite(_m1, HIGH);
        delay(20);
        waitAuxHigh(1000);
    }

    void enterNormalMode() {
        digitalWrite(_m0, LOW); digitalWrite(_m1, LOW);
        delay(20);
        waitAuxHigh(1000);
        _ready = true;
    }

    bool pushConfig(uint8_t channel) {
        _channel = channel;
        uint8_t cmd[11] = {
            0xC0, 0x00, 0x08,
            0xFF, 0xFF,                       // ADDH/ADDL = broadcast (0xFFFF)
            0x62,                             // REG0: UART 9600 8N1, air rate 2.4kbps
            (uint8_t)(_txPowerBits & 0x03),   // REG1: gói 200B, tắt ambient RSSI, công suất TX
            channel,                          // REG2: kênh tần số
            0x80,                             // REG3: bật byte RSSI, transparent, LBT tắt
            0x00, 0x00                        // CRYPT_H/L = không mã hoá
        };
        while (_serial->available()) _serial->read();
        _serial->write(cmd, sizeof(cmd));
        unsigned long t0 = millis();
        uint8_t resp[11]; uint8_t ri = 0;
        while (millis() - t0 < 500 && ri < sizeof(resp)) {
            if (_serial->available()) resp[ri++] = _serial->read();
        }
        bool ok = (ri > 0 && resp[0] == 0xC1);
        // [FIX-LORA-AUX] Chi in log hex khi THAT BAI (dong bo voi
        // Master) - de chan doan nhanh neu gap loi tuong tu: ri=0 ->
        // kiem tra day GND chung/tiep xuc UART; co byte nhung khong
        // bat dau 0xC1 hoac khong on dinh giua cac lan thu -> nghi ngo
        // tiep xuc long/nhieu GND.
        if (!ok) {
            Serial.printf("[LORA-CFG] THAT BAI - resp count=%d, hex: ", ri);
            for (uint8_t i = 0; i < ri; i++) Serial.printf("%02X ", resp[i]);
            Serial.println();
        }
        return ok;
    }
};

LoRaE220Class LoRa;

#include <Preferences.h>     
#include <esp_task_wdt.h>
#include <Update.h>       // [LORA-OTA] flash firmware nhan qua LoRa
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

//======================================================
// CHAN UART / DIEU KHIEN MODULE E220-400T30D
// (thay cho khoi CHAN SPI / LORA cu - module cu da thao ra)
//======================================================
#define LORA_M0        5
#define LORA_M1       18
#define LORA_AUX      19
#define LORA_UART_RX  23
#define LORA_UART_TX  14
//======================================================
// THONG SO LORA
//======================================================
#define LORA_FREQ   433E6
#define LORA_BW     125E3
#define LORA_SF     12
#define LORA_CR      5
#define TX_POWER    30

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
    MSG_ACK       = 4,
    // [LORA-OTA] Cap nhat firmware qua LoRa cho slave - Master la nguon
    // phat (START/DATA), slave phan hoi ACK, Master ket thuc bang END.
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

// [LORA-OTA] Goi tin rieng cho MSG_OTA_START/MSG_OTA_DATA - mang du
// lieu firmware (payload lon hon nhieu so voi Packet 10 byte o tren).
// MSG_OTA_ACK/MSG_OTA_END/MSG_OTA_ABORT van dung Packet thuong (nho,
// nhanh) - chi START/DATA moi can OtaPacket vi phai cho du lieu file.
#define OTA_CHUNK_SIZE 64
typedef struct __attribute__((packed)) {
    uint8_t  sender;
    uint8_t  receiver;
    uint8_t  msgType;     // MSG_OTA_START hoac MSG_OTA_DATA
    uint16_t chunkIndex;  // START: tong so chunk se gui; DATA: chi so chunk (0-based)
    uint16_t chunkLen;    // START: kich thuoc 1 chunk chuan (OTA_CHUNK_SIZE); DATA: so byte hop le trong data[]
    uint32_t totalSize;   // START: tong dung luong firmware (byte); DATA: khong dung (0)
    uint32_t crc32;       // START: CRC32 toan bo firmware; DATA: CRC32 rieng cua chunk nay
    uint8_t  data[OTA_CHUNK_SIZE];
} OtaPacket;

// [LORA-OTA] CRC32 chuan (giong zlib/Python zlib.crc32) - dung ca cho
// Master (tinh CRC32 toan bo file khi tai ve) lan Slave (kiem tra tung
// chunk). Goi crc32Compute(0xFFFFFFFF, buf, len) lan dau, truyen lai
// gia tri tra ve cho lan ke tiep neu tinh gop nhieu doan; XOR voi
// 0xFFFFFFFF o buoc cuoi cung de ra ket qua CRC32 chuan.
uint32_t crc32Compute(uint32_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (~(crc & 1) + 1));
    }
    return crc;
}

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

// [POWER-SAVE] Trạng thái hạ xung CPU hiện tại (true = đang ở 80MHz vì
// bơm hồ hoàn toàn đứng yên). Chỉ gọi setCpuFrequencyMhz() khi trạng
// thái THAY ĐỔI, tránh gọi lặp lại vô ích mỗi vòng loop().
bool cpuFreqLow = false;

//======================================================
// [LORA-OTA] TRANG THAI NHAN FIRMWARE MOI QUA LORA
//======================================================
#define OTA_SLAVE_TIMEOUT_MS 120000UL   // qua 2 phut khong nhan chunk moi -> huy phien
bool     otaActive        = false;
uint16_t otaExpectedChunk = 0;
uint16_t otaTotalChunks   = 0;
uint32_t otaTotalSize     = 0;
uint32_t otaExpectedCrc32 = 0;
unsigned long otaLastPacketAt = 0;

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
// [POWER-SAVE] HẠ/TĂNG XUNG CPU THEO TRẠNG THÁI BƠM HỒ
//======================================================
// CHỈ hạ xung khi bơm hồ ĐỨNG YÊN HOÀN TOÀN (relay tắt, tốc độ hiện
// tại VÀ tốc độ mục tiêu đều = 0 — đủ cả 3 điều kiện để chắc chắn
// không còn dở dang ramp lên/xuống). Ngay khi có lệnh chạy (targetSpeed
// > 0) hoặc relay bật, tăng về 240MHz TRƯỚC KHI processRamp() của chu
// kỳ tiếp theo chạy, đảm bảo ramp luôn chính xác đúng nhịp 50ms.
void updatePowerSaving() {
    bool pumpIdle = (!vfdEnabled) && (currentSpeed == 0) && (targetSpeed == 0);

    if (!pumpIdle) {
        if (cpuFreqLow) {
            setCpuFrequencyMhz(240);
            cpuFreqLow = false;
            Serial.println("[POWER-SAVE] Có lệnh chạy bơm -> tăng CPU về 240MHz.");
        }
    } else {
        if (!cpuFreqLow) {
            setCpuFrequencyMhz(80);
            cpuFreqLow = true;
            Serial.println("[POWER-SAVE] Bơm hồ đứng yên -> hạ CPU xuống 80MHz.");
        }
    }
}

//======================================================
// KHOI TAO LORA (Đã bật CRC chống nhiễu tủ điện)
//======================================================
bool initLoRa() {
    LoRa.setPins(LORA_M0, LORA_M1, LORA_AUX, Serial2, LORA_UART_RX, LORA_UART_TX);

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

void sendAck(uint8_t receiver, uint16_t seq) {
    Packet pkt = { (uint8_t)SLAVE_H_ID, receiver, (uint8_t)MSG_ACK, seq, currentSpeed, (uint32_t)millis() };
    sendPacket(pkt);
}

void sendHeartbeat() {
    Packet pkt = { (uint8_t)SLAVE_H_ID, (uint8_t)MASTER_ID, (uint8_t)MSG_HEARTBEAT, txSeq++, currentSpeed, (uint32_t)millis() };
    sendPacket(pkt);
}

//======================================================
// [LORA-OTA] GUI ACK RIENG CHO OTA (dung Packet nho co san)
//======================================================
void sendOtaAck(uint16_t seq, uint8_t status) {
    Packet pkt = { (uint8_t)SLAVE_H_ID, (uint8_t)MASTER_ID, (uint8_t)MSG_OTA_ACK, seq, status, (uint32_t)millis() };
    sendPacket(pkt);
}

//======================================================
// [LORA-OTA] MASTER BAO BAT DAU GUI FIRMWARE MOI
//======================================================
void handleOtaStart(OtaPacket &opkt) {
    Serial.printf("[OTA] START: size=%lu chunks=%u crc=%08lX\n",
                  (unsigned long)opkt.totalSize, opkt.chunkIndex, (unsigned long)opkt.crc32);

    if (otaActive) Update.abort(); // dang co phien cu do dang -> huy, bat dau lai tu dau

    if (!Update.begin(opkt.totalSize)) {
        Serial.printf("[OTA] Update.begin() THAT BAI: %s\n", Update.errorString());
        otaActive = false;
        sendOtaAck(0, 0); // 0 = tu choi (khong du bo nho flash / loi)
        return;
    }

    otaActive        = true;
    otaExpectedChunk  = 0;
    otaTotalChunks    = opkt.chunkIndex;
    otaTotalSize      = opkt.totalSize;
    otaExpectedCrc32  = opkt.crc32;
    otaLastPacketAt   = millis();

    sendOtaAck(0, 1); // 1 = san sang nhan chunk 0
}

//======================================================
// [LORA-OTA] NHAN 1 CHUNK DU LIEU FIRMWARE
//======================================================
void handleOtaData(OtaPacket &opkt) {
    if (!otaActive) return; // chua START ma da nhan DATA -> goi lac, bo qua

    otaLastPacketAt = millis();

    if (opkt.chunkIndex < otaExpectedChunk) {
        // Chunk nay da ghi flash roi (ACK truoc bi that lac nen Master gui
        // lai) -> ACK lai NGAY, tuyet doi khong ghi chong len flash lan 2.
        sendOtaAck(opkt.chunkIndex, 1);
        return;
    }
    if (opkt.chunkIndex > otaExpectedChunk) {
        // Nhay chunk — khong nen xay ra vi Master gui tuan tu tung chunk
        // mot va cho ACK. Bo qua, khong ACK, de Master tu timeout & gui lai.
        return;
    }

    uint32_t chunkCrc = crc32Compute(0xFFFFFFFF, opkt.data, opkt.chunkLen) ^ 0xFFFFFFFF;
    if (chunkCrc != opkt.crc32) {
        Serial.printf("[OTA] Chunk %u loi CRC -> yeu cau gui lai\n", opkt.chunkIndex);
        sendOtaAck(opkt.chunkIndex, 0); // 0 = loi, yeu cau gui lai dung chunk nay
        return;
    }

    if (Update.write((uint8_t*)opkt.data, opkt.chunkLen) != opkt.chunkLen) {
        Serial.printf("[OTA] Ghi flash THAT BAI o chunk %u: %s\n", opkt.chunkIndex, Update.errorString());
        Update.abort();
        otaActive = false;
        sendOtaAck(opkt.chunkIndex, 0);
        return;
    }

    otaExpectedChunk++;
    sendOtaAck(opkt.chunkIndex, 1);

    if (otaExpectedChunk % 50 == 0 || otaExpectedChunk == otaTotalChunks) {
        Serial.printf("[OTA] Da nhan %u/%u chunk\n", otaExpectedChunk, otaTotalChunks);
    }
}

//======================================================
// [LORA-OTA] MASTER BAO DA GUI XONG TOAN BO FIRMWARE
// An toan rieng cho tram H: HA AP VE 0 + NGAT RELAY BIEN TAN truoc khi
// khoi dong lai, tranh de may bom "treo" o toc do cu trong luc reboot.
//======================================================
void handleOtaEnd(Packet &pkt) {
    if (!otaActive) { sendOtaAck(0xFFFF, 0); return; }

    uint16_t declaredChunks = pkt.seq; // Master dat tong so chunk vao seq cua goi END

    if (otaExpectedChunk != declaredChunks || otaExpectedChunk != otaTotalChunks) {
        Serial.printf("[OTA] END nhung con thieu chunk (%u/%u) -> huy\n", otaExpectedChunk, otaTotalChunks);
        Update.abort();
        otaActive = false;
        sendOtaAck(0xFFFF, 0);
        return;
    }

    if (!Update.end()) {
        Serial.printf("[OTA] Update.end() THAT BAI: %s -> GIU NGUYEN firmware cu\n", Update.errorString());
        otaActive = false;
        sendOtaAck(0xFFFF, 0);
        return;
    }

    Serial.println("[OTA] THANH CONG — ha ap ve 0, ngat relay, khoi dong lai...");
    applySpeed(0); // AN TOAN: tat VFD/DAC truoc khi reboot sang firmware moi
    sendOtaAck(0xFFFF, 1);
    otaActive = false;
    for (int i = 0; i < 30; i++) delay(20); // cho goi ACK cuoi kip bay di truoc khi reboot
    ESP.restart();
}

//======================================================
// [LORA-OTA] HUY PHIEN OTA (Master chu dong huy)
//======================================================
void handleOtaAbort() {
    if (otaActive) {
        Update.abort();
        otaActive = false;
        Serial.println("[OTA] Master da huy phien cap nhat.");
    }
    sendOtaAck(0xFFFE, 1); // xac nhan rieng cho ABORT (sentinel khac END=0xFFFF)
}

//======================================================
// [LORA-OTA] TU HUY NEU QUA LAU KHONG NHAN CHUNK MOI (mat song/Master loi)
//======================================================
void checkOtaTimeout() {
    if (!otaActive) return;
    if (millis() - otaLastPacketAt > OTA_SLAVE_TIMEOUT_MS) {
        Serial.println("[OTA] TIMEOUT — khong nhan chunk moi, huy phien cap nhat.");
        Update.abort();
        otaActive = false;
    }
}

//======================================================
// XU LY GOI LORA DEN
//======================================================
void processIncoming() {
    int sz = LoRa.parsePacket();
    if (sz == 0) return;

    // [LORA-OTA] Goi lon (OtaPacket) mang du lieu firmware — xu ly rieng,
    // tach khoi nhanh Packet nho ben duoi.
    if (sz == sizeof(OtaPacket)) {
        OtaPacket opkt;
        LoRa.readBytes((uint8_t*)&opkt, sizeof(OtaPacket));
        if (opkt.receiver != SLAVE_H_ID && opkt.receiver != 255) return;
        if      (opkt.msgType == MSG_OTA_START) handleOtaStart(opkt);
        else if (opkt.msgType == MSG_OTA_DATA)  handleOtaData(opkt);
        return;
    }

    if (sz != sizeof(Packet)) {
        while (LoRa.available()) LoRa.read();
        return;
    }

    Packet pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(Packet));

    if (pkt.receiver != SLAVE_H_ID && pkt.receiver != 255) return;

    // [LORA-OTA] Coi bat ky goi hop le nao tu Master cung la "con song",
    // tranh checkSignalLost() ngat bom oan trong luc dang OTA lau.
    lastCmdReceived = millis();

    switch (pkt.msgType) {
        case MSG_COMMAND:
            if (pkt.sender == MASTER_ID) {
                uint8_t newSpeed = pkt.payload;
                if (newSpeed > 100) newSpeed = 100;

                sendAck(MASTER_ID, pkt.seq);
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

        case MSG_OTA_END:
            handleOtaEnd(pkt);
            break;

        case MSG_OTA_ABORT:
            handleOtaAbort();
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

    processIncoming();
    checkOtaTimeout();   // [LORA-OTA] tu huy phien cap nhat neu qua lau khong nhan chunk moi
    processRamp();
    updatePowerSaving();   // [POWER-SAVE] đọc trạng thái ramp MỚI NHẤT của chính chu kỳ này
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
