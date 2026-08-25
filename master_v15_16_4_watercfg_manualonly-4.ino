/*****************************************************************
 * MONOLITHIC MASTER NODE - ESP32 + E220-400T30D (LLCC68) + TELEGRAM
 * VERSION 15.16.1 - WATER-CFG: LỆNH CHỈNH CHIỀU CAO TÉC/OFFSET + BẬT-TẮT CẢM BIẾN
 *
 * THAY ĐỔI SO VỚI v15.16.0:
 *  [WATER-CFG] Thêm 3 lệnh Telegram (đều dành riêng cho ADMIN):
 *        `/water_on` `/water_off` — bật/tắt cảm biến siêu âm (mặc định
 *        TẮT lúc mới flash, an toàn khi phần cứng chưa đấu xong; lúc
 *        tắt readWaterLevelSensor() không phát xung Trig nữa).
 *        `/set_tank_height <cm>` — chỉnh chiều cao téc (trước đây cố
 *        định cứng trong code qua WATER_TANK_HEIGHT_CM=200).
 *        `/set_water_offset <cm>` — chỉnh khoảng cách đo lúc téc ĐẦY,
 *        bù trừ vùng chết cảm biến (trước đây cố định WATER_FULL_OFFSET_CM=0).
 *        Cả 3 giá trị (chiều cao/offset/bật-tắt) lưu vào NVS namespace
 *        "water_cfg", nạp lại trong setup() nên không mất khi mất điện.
 *        2 macro cũ đổi tên thành WATER_TANK_HEIGHT_CM_DEFAULT/
 *        WATER_FULL_OFFSET_CM_DEFAULT — chỉ còn dùng làm giá trị mặc
 *        định lần đầu flash, giá trị thực tế nằm ở biến runtime
 *        waterTankHeightCm/waterFullOffsetCm.
 *        [FIX-COMPILE] Nhân tiện vá luôn lỗi thiếu forward declare cho
 *        waterMeasureOnceCm()/waterSortSamples()/readWaterLevelSensor()/
 *        waterLevelText() — bản v15.16.0 gọi các hàm này sớm trong file
 *        (buildStatus()/handleNewMessages()) nhưng chỉ định nghĩa thật
 *        ở phía sau, gần processControl() -> KHÔNG COMPILE ĐƯỢC nếu
 *        không có forward declare -> firmware không nạp được -> toàn
 *        bộ lệnh Telegram im lặng không phản hồi (cùng dạng lỗi từng
 *        gặp và đã ghi nhận ở nhánh master_v15_16_3_fix_watchdog).
 *
 * VERSION 15.16.0 - WATER-LEVEL: THÊM CẢM BIẾN SIÊU ÂM AJ-SR04M ĐO % MỰC NƯỚC TÉC
 *
 * THAY ĐỔI SO VỚI v15.15.5:
 *  [WATER-LEVEL] Thêm đo mực nước téc (cao 2m) bằng cảm biến siêu âm
 *        AJ-SR04M (chế độ 2 dây Trig/Echo). TRIG = GPIO33 (output),
 *        ECHO = GPIO34 (input-only, an toàn vì chỉ đọc). CHÚ Ý PHẦN
 *        CỨNG: ECHO của AJ-SR04M ra mức 5V, chân ESP32 chỉ chịu 3.3V
 *        → bắt buộc cầu phân áp/hạ áp trước khi nối vào GPIO34, không
 *        nối thẳng kẻo cháy chân.
 *        Đo định kỳ mỗi 3s (WATER_READ_INTERVAL_MS), mỗi lần lấy
 *        trung vị 3 mẫu để chống nhiễu, quy đổi khoảng cách -> % theo
 *        chiều cao téc WATER_TANK_HEIGHT_CM=200 (cm). Hiển thị % trong
 *        `/status`, `/status_full`, và lệnh mới `/mucnuoc` (đo ngay
 *        lập tức khi gõ lệnh). Tự gửi cảnh báo Telegram khi mực nước
 *        xuống dưới 15% hoặc lên trên 95% (có debounce 60s chống spam
 *        khi mực nước dao động quanh ngưỡng).
 *
 * VERSION 15.15.5 - PHASE-LINK: SỬA URL GỌI TRẠM ĐIỆN (mDNS thay vì gatewayIP)
 *
 * THAY ĐỔI SO VỚI v15.15.4:
 *  [PHASE-LINK-FIX] pollTramDienPhaseStatus() trước đây gọi
 *        "http://" + WiFi.gatewayIP() + "/phase_status", dựa trên giả
 *        định trạm điện tự phát WiFi AP và Master là client của AP đó.
 *        Giả định này đã SAI kể từ khi firmware trạm điện xoá tính năng
 *        AP (bản v1.8.0 của trạm điện) — cả 2 thiết bị giờ đều là WiFi
 *        STA nối vào CÙNG 1 router nhà, nên gatewayIP() trả về IP của
 *        router chứ không phải trạm điện -> mọi request /phase_status
 *        đều gọi sai địa chỉ và thất bại một cách ÂM THẦM (không log lỗi
 *        rõ ràng, remotePhaseLossActive chỉ đơn giản giữ nguyên giá trị
 *        mặc định false mãi mãi) — tính năng tự ngắt bơm khi mất pha
 *        THỰC TẾ KHÔNG HOẠT ĐỘNG dù code trông như đã hoàn chỉnh.
 *        Sửa: đổi sang gọi cố định "http://tramdien.local/phase_status"
 *        qua mDNS — trạm điện từ firmware v1.9.2 trở lên tự đăng ký
 *        hostname "tramdien" nên địa chỉ này ổn định dù DHCP đổi IP.
 *        YÊU CẦU: trạm điện phải chạy firmware >= v1.9.2 (có mDNS +
 *        server /phase_status) thì tính năng này mới hoạt động được.
 *
 * VERSION 15.15.3 - XAC NHAN FIX LORA THANH CONG, DON LOG DEBUG
 *
 * THAY ĐỔI SO VỚI v15.15.2:
 *  [FIX-LORA-AUX-3] Đã xác nhận LoRa khởi tạo thành công sau khi khắc
 *           phục phần cứng thực tế (nối lại đúng chiều RX/TX + đảm bảo
 *           tiếp xúc/GND chung ổn định — xem log chẩn đoán: phản hồi
 *           ổn định tuyệt đối `C1 00 08 FF FF 62 00 17 80 00 00` lặp
 *           lại y hệt qua nhiều lần thử, đúng định dạng phản hồi ghi
 *           thanh ghi của EBYTE). Nguyên nhân gốc là dây nối vật lý
 *           (RX/TX sai chiều, tiếp xúc/GND chưa chắc chắn gây nhiễu
 *           tín hiệu ngẫu nhiên), KHÔNG phải lỗi logic trong code.
 *           delay(20) "settle" thêm ở FIX-LORA-AUX vẫn giữ lại vì là
 *           cải thiện đúng, chỉ dọn bớt log debug: chỉ in hex phản hồi
 *           khi THẤT BẠI (không in mỗi lần thử khi chạy bình thường)
 *           để không làm rối Serial Monitor lúc vận hành thật, nhưng
 *           vẫn giữ đủ thông tin để chẩn đoán nhanh nếu tái diễn.
 *
 * VERSION 15.15.2 - DEBUG: IN TOAN BO HEX PHAN HOI CAU HINH LORA
 *
 * THAY ĐỔI SO VỚI v15.15.1:
 *  [FIX-LORA-AUX-2] Sau khi đảo dây RX/TX, module đã BẮT ĐẦU phản hồi
 *           (resp count=10, ổn định qua nhiều lần thử) — xác nhận phần
 *           cứng UART/AUX/M0/M1 đã thông đúng, không còn là lỗi dây
 *           nữa. Vấn đề còn lại: byte đầu tiên nhận được là 0xFF thay
 *           vì 0xC1 như code đang kiểm tra — có thể do định dạng phản
 *           hồi thực tế của module khác với giả định ban đầu (lệch
 *           offset, thiếu/dư byte header...). Để xác định chính xác
 *           thay vì đoán mò, đổi log debug từ chỉ in byte đầu sang in
 *           TOÀN BỘ chuỗi hex nhận được — cần log này để sửa đúng điều
 *           kiện kiểm tra trong pushConfig().
 *
 * VERSION 15.15.1 - FIX RACE CONDITION KHI KHOI TAO LORA (AUX TIMING)
 *
 * THAY ĐỔI SO VỚI v15.15.0:
 *  [FIX-LORA-AUX] Đã ghi nhận thực tế: dây/nguồn LoRa đều đã kiểm tra
 *           chuẩn nhưng initLoRa() vẫn báo lỗi phần cứng sau đủ
 *           LORA_INIT_MAX_RETRY lần thử. Nguyên nhân: enterConfigMode()/
 *           enterNormalMode() đổi chân M0/M1 xong gọi waitAuxHigh()
 *           NGAY LẬP TỨC. Module EBYTE cần vài ms để THỰC SỰ bắt đầu
 *           chuyển mode và kéo AUX xuống LOW báo "đang bận" — nếu CPU
 *           đọc AUX nhanh hơn thời gian đó, waitAuxHigh() thấy AUX vẫn
 *           đang HIGH từ trạng thái trước, tưởng module đã sẵn sàng và
 *           trả về ngay. Kết quả: pushConfig() gửi lệnh cấu hình
 *           (0xC0...) trong lúc module CHƯA sẵn sàng nhận UART → không
 *           phản hồi hoặc phản hồi rác → pushConfig() trả về false →
 *           initLoRa() báo "LỖI PHẦN CỨNG LORA" dù dây/nguồn đều đúng.
 *           Đây là lỗi timing phần mềm, không phải phần cứng.
 *
 *           Sửa: thêm delay(20) "settle" ngay sau khi đổi M0/M1, trước
 *           khi bắt đầu theo dõi AUX, để loại bỏ race condition này.
 *           Đồng thời thêm log Serial.printf trong pushConfig() in ra
 *           số byte phản hồi + byte đầu tiên nhận được, giúp chẩn đoán
 *           nhanh nếu sau này lại gặp lỗi tương tự (ri=0 → module
 *           không phản hồi gì, kiểm tra lại dây UART/AUX; byte0 khác
 *           0xC1 → có phản hồi nhưng sai định dạng, thường do module
 *           đang ở baud rate khác 9600 từ trước).
 *
 * VERSION 15.13.3 - THÊM TÍNH NĂNG CẬP NHẬT FIRMWARE QUA OTA (GITHUB)
 *
 * THAY ĐỔI SO VỚI v15.13.2:
 *  [PORT-OTA] Mang tính năng OTA đa phiên bản từ nhánh
 *           master_v15_12_6_multiversion.ino sang bản này (bản này
 *           trước đó hoàn toàn CHƯA có OTA). Lấy manifest
 *           "versions.json" từ repo GitHub public (Trong99x/trambom,
 *           nhánh main) — mỗi bản có tên file .bin RIÊNG, không còn
 *           bắt buộc đặt tên cố định "firmware.bin":
 *             /version        — xem phiên bản đang chạy.
 *             /update         — kiểm tra nhanh bản MỚI NHẤT (mục đầu
 *                               tiên trong versions.json).
 *             /update_list    — liệt kê TOÀN BỘ các bản có trong repo.
 *             /update_to <số thứ tự|version> — chọn đúng bản đó,
 *                               kể cả chọn ngược về bản cũ hơn (rollback).
 *             /update_confirm — bắt đầu tải & flash (bắt buộc gõ để
 *                               xác nhận, không tự động).
 *             /update_cancel  — huỷ yêu cầu đang chờ.
 *           Yêu cầu chọn bản tự hết hạn sau 5 phút nếu không xác nhận
 *           (OTA_CONFIRM_TIMEOUT_MS), tránh xác nhận nhầm yêu cầu cũ.
 *           Trong lúc tải/flash, vTelegramTask tạm dừng poll tin nhắn
 *           (cờ otaRunning) và stack của task này được tăng lên 16384
 *           byte để đủ chỗ cho WiFiClientSecure + HTTPClient +
 *           HTTPUpdate chạy lồng bên trong.
 *           Khác biệt so với bản gốc v15.12.6: sau khi flash xong,
 *           dùng ĐÚNG cơ chế chống lặp riêng của bản 15.13.x này
 *           (gọi bot.getUpdates(bot.last_message_received + 1) để xác
 *           nhận offset với Telegram trước ESP.restart(), giống hệt
 *           cách /reset đã làm ở FIX-20) thay vì cơ chế lưu
 *           last_update_id vào NVS của bản 15.12.6 (bản này không có
 *           sẵn hàm đó) — mục đích như nhau: tránh Master đọc lại và
 *           xử lý lặp lại lệnh /update_confirm cũ ngay sau khi khởi
 *           động lại.
 *
 * VERSION 15.13.2 - CHỐNG VÒNG LẶP RESET KHI DÙNG LỆNH /reset
 *
 * THAY ĐỔI SO VỚI v15.13.1:
 *  [FIX-20] Đã ghi nhận thực tế: gõ lệnh /reset khiến hệ thống rơi
 *           vào vòng lặp khởi động lại vô hạn, không lên bình thường
 *           được. Nguyên nhân: bot.getUpdates() cập nhật
 *           bot.last_message_received trong RAM ngay khi lấy được
 *           lệnh /reset, nhưng Telegram server chỉ coi update đó là
 *           "đã xác nhận" khi client gọi getUpdates() LẦN TIẾP THEO
 *           với offset mới hơn. Vì ESP.restart() xảy ra ngay sau khi
 *           xử lý /reset (không kịp gọi getUpdates lần 2), Telegram
 *           vẫn giữ update đó ở trạng thái chưa xác nhận. Sau khi
 *           reboot, last_message_received về lại 0 -> gọi
 *           getUpdates(1) -> Telegram trả lại ĐÚNG lệnh /reset cũ ->
 *           xử lý lại -> restart lại -> lặp vô hạn.
 *
 *           Sửa: gọi thêm 1 lần bot.getUpdates(bot.last_message_received
 *           + 1) để xác nhận offset với Telegram NGAY TRƯỚC khi
 *           ESP.restart(), chặn đứng việc lệnh /reset bị gửi lại sau
 *           mỗi lần reboot.
 *
 * THAY ĐỔI SO VỚI v15.13.0:
 *  [FIX-19] Đã ghi nhận thực tế reset "WATCHDOG TASK - Có task bị
 *           treo" (ESP_RST_TASK_WDT). Nguyên nhân khả dĩ nhất:
 *           secured_client.setTimeout(TELEGRAM_HTTP_TIMEOUT_SEC) chỉ
 *           giới hạn thời gian ĐỌC dữ liệu, KHÔNG giới hạn DNS
 *           resolve + TCP connect + TLS handshake bên trong
 *           bot.sendMessage()/bot.getUpdates(). Khi WiFi chập chờn,
 *           bước connect() có thể treo lâu hơn WDT_TIMEOUT_SEC (60s)
 *           mà không có esp_task_wdt_reset() nào can thiệp được vì
 *           đó là 1 lệnh gọi chặn (blocking) duy nhất. Nghi phạm phụ:
 *           WiFi.scanNetworks() trong scanAndPickBestWifi() (3 chỗ
 *           gọi trong vTelegramTask) cũng không có bảo vệ tương tự.
 *
 *           Sửa: (1) thêm secured_client.setHandshakeTimeout(10) để
 *           giảm nguy cơ ngay từ gốc; (2) quan trọng hơn — tạm gỡ
 *           task khỏi watchdog bằng esp_task_wdt_delete(NULL) ngay
 *           trước mỗi lệnh mạng có thể chậm tự nhiên (sendMessage,
 *           getUpdates, scanAndPickBestWifi), rồi esp_task_wdt_add(NULL)
 *           đăng ký lại ngay sau. Nhờ vậy các lệnh mạng hợp lệ nhưng
 *           chậm không còn bị hiểu nhầm là "treo" và gây reset oan —
 *           trong khi các task khác (LoRa, loop chính) vẫn được WDT
 *           giám sát bình thường suốt thời gian đó.
 *
 * THAY ĐỔI SO VỚI v15.12.0:
 *  [FIX-18] Thêm theo dõi + hiển thị thời gian BẬT của 3 bơm (Moong 1,
 *           Moong 2, Trạm Hồ) TRONG NGÀY HÔM NAY, hiện trong /status
 *           và /status_full.
 *           - Cần đồng bộ giờ thực qua NTP (configTime() gọi 1 lần
 *             ngay sau khi WiFi kết nối lần đầu, timezone VN = UTC+7,
 *             không DST) để biết chính xác lúc nào là "sang ngày
 *             mới" — nếu chưa đồng bộ được giờ, hệ thống hiển thị rõ
 *             "⚠️ Chưa đồng bộ được giờ hệ thống" thay vì im lặng
 *             hiện 00h00m gây hiểu lầm là bơm chưa chạy gì.
 *           - Thời gian tích luỹ được LƯU NVS mỗi 5 phút
 *             (RUNTIME_CHECKPOINT_MS) và mỗi khi bơm tắt, để nếu
 *             Master bị treo/reset giữa ngày (watchdog, LoRa lỗi...)
 *             thì không mất trắng số liệu — nạp lại đúng số đã tích
 *             luỹ nếu vẫn cùng ngày.
 *           - Khi sang ngày mới (theo giờ NTP), tự "chốt sổ" (checkpoint)
 *             số liệu hôm qua rồi reset về 0 cho ngày mới.
 *           - Giới hạn: bơm Trạm Hồ (Slave H) không gửi trạng thái
 *             vật lý thật về Master, nên thời gian ghi nhận là thời
 *             gian Master RA LỆNH bật/tắt (targetPumpH), không phải
 *             xác nhận từ cảm biến thật trên Slave H.
 *           - Nếu Master reboot xuyên qua nửa đêm đúng lúc, hoặc bơm
 *             đang chạy xuyên qua thời điểm 00:00, hệ thống KHÔNG chia
 *             tách chính xác phần trước/sau nửa đêm — coi như phiên
 *             chạy "bắt đầu lại" tại mốc sang ngày (đơn giản hoá có
 *             chủ đích, sai số tối đa không quá 1 chu kỳ đang chạy).
 *
 * THAY ĐỔI SO VỚI v15.11.0:
 *  [FIX-17] Trước đây setup() gọi initLoRa() TRƯỚC setupWiFiSingle().
 *           Nếu phần cứng LoRa hỏng vĩnh viễn (vd cấp nhầm 5V), 
 *           initLoRa() gọi ESP.restart() sau 5 lần thử thất bại —
 *           nhưng vì WiFi/Telegram CHƯA từng được khởi tạo ở thời
 *           điểm đó, hệ thống rơi vào VÒNG LẶP REBOOT VÔ HẠN mà
 *           KHÔNG BAO GIỜ gửi được cảnh báo Telegram nào — admin chỉ
 *           phát hiện được bằng cách cắm dây Serial xem log trực
 *           tiếp. Đây là lỗ hổng quan sát nghiêm trọng cho vận hành
 *           24/7 không người trông.
 *
 *           Sửa: (1) đảo thứ tự — setupWiFiSingle() chạy TRƯỚC
 *           initLoRa(); (2) initLoRa() không còn ESP.restart() khi
 *           thất bại — thay vào đó đặt cờ loraHardwareOk=false, gửi
 *           NGAY 1 cảnh báo Telegram (giờ đã gửi được vì WiFi đã
 *           lên) rồi để hệ thống tiếp tục chạy bình thường (WiFi,
 *           Telegram, lệnh thủ công /reset, /status_full... vẫn hoạt
 *           động), chỉ riêng mảng LoRa/bơm tự động bị treo cho tới
 *           khi phần cứng được khắc phục; (3) vLoRaRealtimeTask tự
 *           thử phục hồi LoRa ngầm mỗi LORA_HW_RETRY_MS (5 phút) mà
 *           KHÔNG cần reboot toàn bộ — nếu thành công, tự gửi cảnh
 *           báo "LoRa đã phục hồi" và tiếp tục hoạt động bình
 *           thường ngay, không cần can thiệp tay.
 *
 * THAY ĐỔI SO VỚI v15.10.0:
 *  [FIX-16] Thêm lệnh Telegram `/reset`: khởi động lại toàn bộ ESP32
 *           bằng ESP.restart() — về mặt hiệu quả TƯƠNG ĐƯƠNG nhấn nút
 *           RESET vật lý (chip chạy lại từ setup(), toàn bộ biến RAM
 *           (volatile hay không) reset về giá trị mặc định khai báo
 *           — pumpM1/M2/H, controlMode, manualPump*, thống kê LoRa,
 *           danh sách pending/hàng đợi lệnh... tất cả về trạng thái
 *           "mới cứng" y hệt lúc chip vừa cấp nguồn).
 *           LƯU Ý QUAN TRỌNG: giống hệt nút reset vật lý thật, lệnh
 *           này KHÔNG xoá dữ liệu đã lưu trong NVS/flash (WiFi đã
 *           lưu, Token Telegram, tốc độ bơm Hồ, trạng thái phao
 *           M1/M2 theo FIX-13) — vì bộ nhớ flash không bị ảnh hưởng
 *           bởi bất kỳ loại reset nào (kể cả nhấn nút EN vật lý thật
 *           sự). Nếu cần xoá sạch cấu hình ("factory reset") thì đây
 *           là một tính năng KHÁC, cần lệnh riêng — hiện chưa có.
 *
 *  GHI CHÚ VỀ TREO/WATCHDOG: hệ thống ĐÃ TỰ ĐỘNG khởi động lại y hệt
 *  reset vật lý mỗi khi Task Watchdog phát hiện task bị treo, nhờ cấu
 *  hình `trigger_panic = true` trong initWatchdog() (đã có từ trước,
 *  không đổi ở bản này) — khi WDT hết hạn, ESP-IDF panic handler in
 *  log rồi tự reboot toàn chip, ghi nhận lý do ESP_RST_TASK_WDT /
 *  ESP_RST_INT_WDT / ESP_RST_PANIC trong getResetReasonText(). Đây
 *  chính là cơ chế đã quan sát thấy hoạt động đúng trong log thực tế
 *  trước đó ("Lý do khởi động: WATCHDOG TASK - Có task bị treo").
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
#include <ESPmDNS.h>   // [PHASE-LINK-FIX v15.15.5] để chắc chắn resolver hỗ trợ "tramdien.local"
                        // (ESP32 core tự hỗ trợ .local qua mdns_query khi include thư viện này,
                        // không cần gọi MDNS.begin() ở phía Master — chỉ trạm điện mới cần begin()
                        // vì nó là bên "quảng bá" hostname, Master chỉ là bên "hỏi").
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <LittleFS.h>      // [LORA-OTA] luu tam file .bin slave truoc khi relay qua LoRa
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

    // [FIX-LORA-AUX] Truoc day goi waitAuxHigh() NGAY sau khi doi M0/M1.
    // Loi: module can vai ms de THUC SU bat dau chuyen mode va keo AUX
    // xuong LOW bao "dang ban". Neu CPU doc AUX qua nhanh (truoc khi
    // module kip keo xuong), waitAuxHigh() thay AUX van dang HIGH tu
    // trang thai truoc do -> tuong lam xong ngay -> gui lenh cau hinh
    // (0xC0...) trong luc module CHUA san sang nhan UART -> module
    // khong phan hoi hoac phan hoi rac -> pushConfig() tra ve false ->
    // initLoRa() bao "loi phan cung" du day/nguon deu chuan. Them
    // delay(20) "settle" ngay sau khi doi chan, truoc khi kiem tra AUX,
    // de loai bo race condition nay.
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
        // [FIX-LORA-AUX] Chi in log hex khi THAT BAI - luc chay binh
        // thuong khong can lam ron Serial Monitor. Neu sau nay lai gap
        // loi khoi tao LoRa, log nay se cho biet ngay module co phan
        // hoi gi khong (ri=0 -> kiem tra day GND chung/tiep xuc UART;
        // co byte nhung khong bat dau 0xC1 hoac khong on dinh giua cac
        // lan thu -> nghi ngo tiep xuc long/nhieu GND, xem changelog
        // FIX-LORA-AUX va FIX-LORA-AUX-2 o dau file).
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
#include <time.h>   // [FIX-18] time(), localtime_r(), struct tm — cần cho NTP/theo dõi ngày

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//======================================================
// CẤU HÌNH MẠNG & TELEGRAM MẶC ĐỊNH
//======================================================
#define WIFI_SSID_DEFAULT "VuThiMai"
#define WIFI_PASS_DEFAULT "09121958"

#define BOT_TOKEN_DEFAULT "8777904636:AAGMBfWJDtN8WvsUtXhYT9SEXBfAnKL9ySE"
#define CHAT_ID_DEFAULT   "8752050398"

//======================================================
// OTA UPDATE QUA GITHUB (ĐA PHIÊN BẢN) [PORT từ v15.12.6]
// Repo public: https://github.com/Trong99x/trambom
// Quy trình cập nhật: build .bin mới (tên tuỳ ý, không cần đè file
// cũ) -> upload file .bin đó lên nhánh main -> thêm 1 mục mới vào ĐẦU
// danh sách "versions" trong versions.json (gồm version + tên file +
// ghi chú) -> gửi lệnh /update (hoặc /update_list) qua Telegram để
// xem & chọn bản cần cài. Master chỉ kiểm tra & cập nhật khi NHẬN
// LỆNH, không tự động định kỳ.
//
// versions.json (đặt ở gốc nhánh main) có dạng:
// {
//   "versions": [
//     { "version": "15.13.2", "file": "master_v15_13_2.bin", "note": "..." },
//     { "version": "15.12.6", "file": "master_v15_12_6.bin", "note": "..." }
//   ]
// }
// Mục ĐẦU TIÊN trong mảng luôn được coi là bản MỚI NHẤT (dùng cho
// /update kiểm tra nhanh). Các mục còn lại vẫn có thể cài lại bất cứ
// lúc nào bằng /update_to <số thứ tự|version> — kể cả để ROLLBACK về
// bản cũ hơn bản đang chạy.
//======================================================
#define FW_VERSION        "15.16.3"
#define OTA_GITHUB_OWNER  "Trong99x"
#define OTA_GITHUB_REPO   "trambom"
#define OTA_GITHUB_BRANCH "main"
#define OTA_MANIFEST_URL  "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/versions.json"
#define OTA_RAW_BASE_URL  "https://raw.githubusercontent.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/" OTA_GITHUB_BRANCH "/"
#define OTA_HTTP_TIMEOUT_MS 30000UL
#define OTA_MAX_VERSIONS  15   // số mục tối đa đọc từ versions.json (mảng tĩnh, không cấp phát động)

//======================================================
// ĐỊNH DANH NODE
//======================================================
#define MASTER_ID    0
#define SLAVE_M1_ID  1
#define SLAVE_M2_ID  2
#define SLAVE_H_ID   3

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
// THÔNG SỐ LORA
//======================================================
#define LORA_FREQ  433E6
#define LORA_BW    125E3
#define LORA_SF    12
#define LORA_CR    5
#define TX_POWER   30
#define LORA_INIT_MAX_RETRY  5
// [FIX-17] Khi phần cứng LoRa hỏng, thử phục hồi ngầm mỗi 5 phút thay
// vì reboot toàn bộ liên tục.
#define LORA_HW_RETRY_MS   300000UL
// [FIX-18] Chốt sổ + lưu NVS thời gian chạy bơm trong ngày mỗi 5 phút.
#define RUNTIME_CHECKPOINT_MS 300000UL

//======================================================
// CHÂN GPIO
//======================================================
#define FLOAT_MASTER_PIN 32
#define RELAY_M1_PIN     27
#define RELAY_M2_PIN     25

// [WATER-LEVEL] Cảm biến siêu âm AJ-SR04M đo mực nước téc (chế độ 2 dây
// Trig/Echo, KHÔNG dùng chế độ 1 dây). Lưu ý phần cứng: AJ-SR04M chạy
// 5V và chân ECHO xuất mức HIGH 5V — chân ESP32 chỉ chịu 3.3V, BẮT BUỘC
// qua cầu phân áp (VD: điện trở 1K nối tiếp 2K xuống GND, lấy giữa) hoặc
// module hạ áp trước khi nối vào WATER_ECHO_PIN để tránh cháy chân GPIO.
#define WATER_TRIG_PIN   33
#define WATER_ECHO_PIN   34   // chân input-only trên ESP32, phù hợp vì chỉ cần đọc

// Chiều cao téc nước tính bằng cm, tính từ mặt nước thấp nhất (0%) đến
// vị trí lắp cảm biến ở đỉnh téc (100%). Téc cao 2m → 200cm.
// [WATER-CFG] Đây chỉ là giá trị MẶC ĐỊNH dùng lần đầu flash (chưa có
// gì lưu trong NVS). Sau đó chỉnh qua lệnh Telegram /set_tank_height,
// giá trị thực tế đang dùng nằm trong biến waterTankHeightCm (nạp từ
// NVS lúc setup()), không phải macro này nữa.
#define WATER_TANK_HEIGHT_CM_DEFAULT   200
// Khoảng cách đo được lúc téc ĐẦY (nước dâng gần sát cảm biến). Nếu
// cảm biến lắp sát mép trong đỉnh téc, để 0. AJ-SR04M có vùng chết đo
// gần ~20-25cm nên khuyến nghị lắp cách mặt nước tối đa ít nhất chừng
// đó; nếu vậy chỉnh giá trị này = khoảng cách thực đo lúc téc đầy.
// [WATER-CFG] Cũng chỉ là giá trị mặc định — dùng /set_water_offset để
// chỉnh, giá trị thực tế nằm trong waterFullOffsetCm.
#define WATER_FULL_OFFSET_CM_DEFAULT   0
#define WATER_READ_INTERVAL_MS 3000UL
#define WATER_PULSE_TIMEOUT_US 30000UL   // ứng với ~5m, dư cho téc 2m
#define WATER_SAMPLE_COUNT     3         // lấy trung vị 3 lần đo để chống nhiễu
// [WATER-CFG] Mặc định TẮT cảm biến lúc mới flash — an toàn khi phần
// cứng chưa đấu xong (TRIG/ECHO chưa nối, hoặc chưa lắp cầu phân áp).
// Bật lên bằng lệnh Telegram /water_on khi đã lắp xong.
#define WATER_SENSOR_ENABLED_DEFAULT false
#define WATER_LOW_PERCENT      15        // % cảnh báo thấp
#define WATER_HIGH_PERCENT     95        // % cảnh báo gần đầy
#define WATER_ALERT_DEBOUNCE_MS 60000UL  // chống báo động rung ở ngưỡng

// [WATER-INTERLOCK] Khoá chéo bơm Moong (M1/M2) <-> bơm Trạm Hồ theo %
// mực nước téc, ÁP DỤNG BẤT KỂ chế độ AUTO hay THỦ CÔNG:
//  - Mực nước < ngưỡng THẤP  -> khoá cứng M1+M2 (không cho bơm ra khỏi
//    téc nữa), chỉ còn bơm Hồ được phép chạy (bơm Hồ vẫn theo logic
//    thường, không bị ép bật).
//  - Mực nước >= ngưỡng CAO  -> khoá cứng bơm Hồ (không bơm thêm vào
//    téc nữa), chỉ còn M1/M2 được phép chạy.
//  - Sau khi bị khoá do ĐẦY, chỉ mở khoá lại bơm Hồ khi mực nước rút
//    xuống <= ngưỡng HỒI PHỤC (chống đóng/cắt rung sát ngưỡng CAO).
// Cả 3 ngưỡng đều chỉnh được qua Telegram (/set_water_pump_low,
// /set_water_pump_high, /set_water_pump_recover), lưu NVS. Các macro
// *_DEFAULT chỉ dùng lần đầu flash, giá trị thực tế nằm trong biến
// waterPumpBlockLowPercent/waterPumpBlockHighPercent/waterPumpHighRecoverPercent.
#define WATER_PUMP_BLOCK_LOW_PERCENT_DEFAULT     20
#define WATER_PUMP_BLOCK_HIGH_PERCENT_DEFAULT    100
#define WATER_PUMP_HIGH_RECOVER_PERCENT_DEFAULT  90

//======================================================
// TIMING HỆ THỐNG
//======================================================
#define HEARTBEAT_MS              60000UL
#define HEARTBEAT_JITTER_MS         3000UL
#define SLAVE_TIMEOUT_MS         600000UL
#define FLOAT_DEBOUNCE_MS          5000UL
#define ACK_DELAY_MS               2500UL
#define CMD_DELAY_MS              3000UL
#define PUMP_H_RESYNC_MS         150000UL
#define TELEGRAM_POLL_MS           7000UL
#define WDT_TIMEOUT_SEC               60
#define LORA_MUTEX_TIMEOUT_MS      300UL
#define WIFI_RECONNECT_MS         50000UL
#define WIFI_OFFLINE_FAILSAFE_MS 300000UL
#define WIFI_WAKE_INTERVAL_MS   3600000UL
#define WIFI_WAKE_TRY_MS          30000UL
#define TELEGRAM_HTTP_TIMEOUT_SEC    10
#define STATUS_CACHE_TTL_MS       20000UL
#define CMD_MAX_RETRIES               5
#define WIFI_SWITCH_DELAY_MS       3000UL
#define BOOT_GRACE_MS            120000UL

#define SLAVE_FLOAT_DEBOUNCE_MS    5000UL

#define RESOURCE_MONITOR_MS      620000UL
#define HEAP_WARN_THRESHOLD_BYTES  30000UL

#define AUTO_REBOOT_INTERVAL_MS 86400000UL
#define AUTO_REBOOT_CHECK_MS      660000UL

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

// [FIX-17] Trạng thái phần cứng LoRa — false nghĩa là chip không phản
// hồi (hỏng/mất dây), hệ thống vẫn chạy WiFi/Telegram bình thường,
// chỉ tạm ngưng chức năng LoRa/bơm tự động cho tới khi tự phục hồi
// hoặc được khắc phục vật lý.
volatile bool loraHardwareOk       = false;
unsigned long lastLoraRetryAttempt = 0;

// [FIX-18] Thời gian BẬT của từng bơm trong ngày hôm nay (giây), cộng
// dồn mỗi khi bơm chuyển từ BẬT sang TẮT. Đồng bộ ngày qua giờ NTP.
volatile unsigned long pumpM1RuntimeTodaySec = 0;
volatile unsigned long pumpM2RuntimeTodaySec = 0;
volatile unsigned long pumpHRuntimeTodaySec  = 0;
// Mốc millis() lúc bơm vừa BẬT — 0 nghĩa là hiện đang TẮT.
volatile unsigned long pumpM1OnSinceMs       = 0;
volatile unsigned long pumpM2OnSinceMs       = 0;
volatile unsigned long pumpHOnSinceMs        = 0;
// -1 = chưa xác định được ngày hiện tại (chưa đồng bộ NTP xong).
int           trackedDayMarker      = -1;
unsigned long lastRuntimeCheckpoint = 0;

Preferences runtimePrefs;   // [FIX-18] namespace riêng lưu thời gian chạy bơm/ngày qua reboot

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

// [WATER-LEVEL] Trạng thái mực nước téc đo bằng AJ-SR04M.
int           waterLevelPercent   = -1;     // -1 = chưa có số liệu hợp lệ
float         waterLevelDistanceCm = -1;
bool          waterSensorOk       = false;  // false nếu lần đo gần nhất timeout/vô lý
unsigned long lastWaterReadMs     = 0;
unsigned long lastWaterAlertMs    = 0;
bool          waterLowAlertSent   = false;
bool          waterHighAlertSent  = false;

// [WATER-CFG] Cấu hình téc nước chỉnh được qua Telegram, lưu NVS để
// không mất khi mất điện/reboot. Nạp giá trị thật trong setup() từ
// namespace "water_cfg"; các macro *_DEFAULT ở trên chỉ dùng lần đầu.
Preferences   waterCfgPrefs;
int           waterTankHeightCm  = WATER_TANK_HEIGHT_CM_DEFAULT;
int           waterFullOffsetCm  = WATER_FULL_OFFSET_CM_DEFAULT;
bool          waterSensorEnabled = WATER_SENSOR_ENABLED_DEFAULT;

// [WATER-INTERLOCK] Ngưỡng % khoá chéo bơm Moong <-> bơm Hồ, chỉnh được
// qua Telegram, lưu NVS namespace "water_cfg" (giống các cấu hình téc
// nước khác ở trên).
int           waterPumpBlockLowPercent    = WATER_PUMP_BLOCK_LOW_PERCENT_DEFAULT;
int           waterPumpBlockHighPercent   = WATER_PUMP_BLOCK_HIGH_PERCENT_DEFAULT;
int           waterPumpHighRecoverPercent = WATER_PUMP_HIGH_RECOVER_PERCENT_DEFAULT;
// Latch (chốt trạng thái) khoá bơm Hồ do téc ĐẦY — có độ trễ (hysteresis):
// bật lên true khi mực nước chạm ngưỡng CAO, chỉ về false khi mực nước
// rút xuống <= ngưỡng HỒI PHỤC. Không latch cho phía M1/M2 vì yêu cầu
// không cần độ trễ ở ngưỡng thấp.
bool          waterPumpHBlockedByFull     = false;

// [PHASE-LINK] Trạng thái mất pha nhận được từ trạm điện. Trạm điện KHÔNG
// còn tự phát WiFi AP riêng nữa (đã bỏ từ bản v1.8.0 phía firmware trạm
// điện) — cả Master lẫn trạm điện giờ đều là WiFi STA nối chung 1 router
// nhà, nên KHÔNG thể dùng WiFi.gatewayIP() (trỏ về router, không phải
// trạm điện). Trạm điện đăng ký sẵn mDNS "tramdien.local" (từ firmware
// v1.9.2 trở lên) để gọi ổn định dù DHCP đổi IP.
#define PHASE_LINK_POLL_MS      3000UL   // hỏi trạm điện mỗi 3s
#define PHASE_LINK_TIMEOUT_MS   15000UL  // quá lâu không hỏi được -> coi là mất liên lạc (KHÔNG tự suy ra mất pha)
#define TRAM_DIEN_PHASE_LINK_URL "http://tramdien.local/phase_status"
volatile bool  remotePhaseLossActive   = false;   // true = trạm điện đang báo mất pha -> khoá bơm
bool           phaseLinkReachable      = false;   // có gọi được trạm điện lần gần nhất không
unsigned long  lastPhaseLinkPollMs     = 0;
unsigned long  lastPhaseLinkOkMs       = 0;
bool           lastNotifiedPhaseLoss   = false;   // để chỉ gửi Telegram khi trạng thái đổi

String alertBlock(const char *icon, const String &title, const String &detail = "");   // [PHASE-LINK] forward declare, định nghĩa thật ở dưới
void   sendTelegramAlert(const String &msg);                                            // [PHASE-LINK] forward declare, định nghĩa thật ở dưới

// [WATER-LEVEL] forward declare — được gọi trong buildStatus()/handleNewMessages()
// (nằm sớm trong file) nhưng định nghĩa thật nằm ở gần processControl()
// (nằm sau, gần cuối file). KHÔNG được bỏ các dòng forward declare này,
// nếu không sẽ lỗi compile "was not declared in this scope" và firmware
// không build/nạp được -> toàn bộ lệnh Telegram im lặng không phản hồi
// (đã từng gặp đúng lỗi này, xem lịch sử fix ở bản v15.16.2).
float  waterMeasureOnceCm();
void   waterSortSamples(float *a, int n);
void   readWaterLevelSensor();
String waterLevelText();

// [PHASE-LINK] Gọi HTTP GET tới trạm điện qua mDNS (tramdien.local) để lấy
// trạng thái mất pha mới nhất — KHÔNG dùng gatewayIP() vì trạm điện không
// còn là AP (xem giải thích ở khai báo TRAM_DIEN_PHASE_LINK_URL phía trên).
void pollTramDienPhaseStatus() {
    if (WiFi.status() != WL_CONNECTED) return;   // đang không có WiFi thì bỏ qua, giữ nguyên trạng thái cũ

    HTTPClient http;
    http.setTimeout(2000);
    if (!http.begin(TRAM_DIEN_PHASE_LINK_URL)) { phaseLinkReachable = false; return; }

    int code = http.GET();
    if (code == HTTP_CODE_OK) {
        String body = http.getString();
        StaticJsonDocument<128> doc;
        DeserializationError err = deserializeJson(doc, body);
        if (!err) {
            bool any = doc["anyPhaseLoss"] | false;
            remotePhaseLossActive = any;
            phaseLinkReachable    = true;
            lastPhaseLinkOkMs     = millis();

            if (any != lastNotifiedPhaseLoss) {
                lastNotifiedPhaseLoss = any;
                if (any) sendTelegramAlert(alertBlock(ICON_CRIT, "🚨 TRẠM ĐIỆN BÁO MẤT PHA",
                              "Đã tự động TẮT toàn bộ bơm (M1, M2, Trạm Hồ) để bảo vệ động cơ.\nChờ trạm điện báo hết mất pha để hoạt động trở lại."));
                else      sendTelegramAlert(alertBlock(ICON_SUCCESS, "Trạm điện: đã hết mất pha",
                              "Hệ thống bơm được phép hoạt động trở lại theo chế độ hiện tại."));
            }
        }
    } else {
        phaseLinkReachable = false;
    }
    http.end();

    // [PHASE-LINK] Mất liên lạc quá lâu với trạm điện: KHÔNG tự coi là mất
    // pha (tránh dừng bơm oan khi chỉ là lỗi WiFi/HTTP tạm thời), chỉ giữ
    // nguyên remotePhaseLossActive ở giá trị đã biết gần nhất. An toàn hơn
    // là để người vận hành nhận cảnh báo mất liên lạc qua /status_full.
}
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

// [OTA] Bật lên trong lúc đang tải + flash firmware để vTelegramTask
// tạm dừng poll tin nhắn (getUpdates dùng chung TLS client với OTA).
volatile bool otaRunning = false;

//======================================================
// [LORA-OTA] TRẠNG THÁI PHIÊN CẬP NHẬT FIRMWARE SLAVE QUA LORA
// (chạy trên vLoraOtaRelayTask riêng — KHÔNG chặn Telegram/điều khiển
// thủ công trong lúc truyền, vì có thể mất rất lâu qua LoRa).
//======================================================
TaskHandle_t      otaLoraTaskHandle       = NULL;
volatile bool     otaLoraActive           = false;
volatile bool     otaLoraCancelRequested  = false;
volatile uint16_t otaLoraChunkIndex       = 0;
volatile uint16_t otaLoraTotalChunks      = 0;
uint8_t            otaLoraTargetId        = 0;
String             otaLoraTargetKey       = "";   // "m1" | "m2" | "h"
String             otaLoraVersion         = "";

// [LORA-OTA] Nơi processLoRa() ghi lại ACK/NACK mới nhất nhận được từ
// slave (msgType==MSG_OTA_ACK) để vLoraOtaRelayTask theo dõi/chờ, mà
// KHÔNG tự gọi LoRa.parsePacket() (tránh tranh chấp với vLoRaRealtimeTask
// đang là nơi DUY NHẤT đọc UART của module).
volatile bool     otaAckReceived = false;
volatile uint16_t otaAckSeq      = 0;
volatile uint8_t  otaAckStatus   = 0;

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

String alertBlock(const char *icon, const String &title, const String &detail) {
    String m = String(icon) + " *" + title + "*";
    if (detail.length() > 0) m += "\n" + detail;
    return m;
}

// [FIX-OTA-MD] Chuỗi ĐỘNG lấy từ nguồn ngoài (versions.json, thông báo lỗi
// HTTP/JSON, tên file .bin...) có thể chứa ký tự đặc biệt của Markdown
// (_ * ` [ ). Vì sendTelegramDirect() LUÔN bật parse_mode="Markdown" (do
// alertBlock() luôn bọc title trong dấu *), một ký tự không đi theo cặp
// (vd: "_" lẻ trong "firmware_v15_14_1.bin") khiến Telegram trả lỗi
// "can't parse entities" và bot.sendMessage() ÂM THẦM THẤT BẠI — tin nhắn
// không tới máy người dùng nào cả, dù lệnh vẫn chạy đúng phía thiết bị.
// Escape các ký tự này (\_ \* \` \[) trước khi ghép vào tin nhắn để
// Telegram luôn parse được, thay vì im lặng huỷ gửi.
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

//======================================================
// QUẢN LÝ NHIỀU NGƯỜI DÙNG (ADMIN / USER)
// Lưu bằng Preferences (namespace "users") -> KHÔNG mất
// danh sách sau khi mất điện / khởi động lại, khác với
// mảng String thuần chỉ sống trong RAM.
//======================================================
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

// Gọi 1 lần trong setup(). Nếu chưa từng có ai trong NVS (lần nạp đầu
// tiên), tự động thêm CHAT_ID_DEFAULT làm ADMIN đầu tiên, để tránh bị
// khoá ngoài hệ thống của chính mình sau khi cập nhật firmware.
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

// true nếu thêm thành công; false nếu đã tồn tại hoặc danh sách đầy.
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

// [MULTI-USER] Gửi NGAY (đồng bộ) tới TOÀN BỘ người dùng đã đăng ký —
// dùng cho các cảnh báo/thông báo hệ thống quan trọng (OTA, reset...).
// Trước đây chỉ gửi tới currentChatId (1 người duy nhất).
void sendTelegramDirect(const String &msg) {
    logAlert(msg);
    bool md = (msg.indexOf('*') != -1);
    if (userCount == 0) {
        // Chưa nạp danh sách người dùng (hoặc rỗng) -> fallback về
        // currentChatId để không bao giờ "câm" hoàn toàn.
        bot.sendMessage(currentChatId, msg, md ? "Markdown" : "");
        return;
    }
    for (int i = 0; i < userCount; i++) {
        bot.sendMessage(userIds[i], msg, md ? "Markdown" : "");
    }
}

// [MULTI-USER] Gửi NGAY (đồng bộ) chỉ tới 1 chat_id cụ thể — dùng để
// trả lời riêng cho người vừa gõ lệnh (vd: /myid, /users, /adduser...)
// thay vì phát cho tất cả mọi người.
void sendTelegramReplyTo(const String &chatId, const String &msg) {
    logAlert(msg);
    bool md = (msg.indexOf('*') != -1);
    bot.sendMessage(chatId, msg, md ? "Markdown" : "");
}

//======================================================
// OTA: TRẠNG THÁI CHỜ XÁC NHẬN  [PORT từ v15.12.6]
// Sau khi /update, /update_list hoặc /update_to chọn ra 1 bản cụ thể,
// KHÔNG tải ngay — lưu tạm version + tên file vào otaPendingVersion /
// otaPendingFile và chờ user gõ /update_confirm.
// Hết OTA_CONFIRM_TIMEOUT_MS mà không xác nhận thì coi như huỷ, phải
// chọn lại từ đầu (tránh xác nhận nhầm 1 yêu cầu đã quá cũ).
//
// [LORA-OTA] otaPendingTarget = "master" (mặc định, giữ nguyên hành vi
// cũ) hoặc "m1"/"m2"/"h" — khi khác "master", /update_confirm sẽ chạy
// performLoraOtaUpdate() thay vì httpUpdate tự cập nhật chính Master.
//======================================================
#define OTA_CONFIRM_TIMEOUT_MS 300000UL // 5 phút
String        otaPendingVersion   = "";
String        otaPendingFile      = "";
String        otaPendingTarget    = "master";
unsigned long otaPendingExpireAt  = 0;

// Danh sách các bản đọc được từ versions.json lần gần nhất (dùng để
// /update_to <số thứ tự> ánh xạ ra đúng version/file mà không phải
// tải lại manifest). Mảng tĩnh, không cấp phát động.
struct OtaEntry {
    String version;
    String file;
    String note;
    String target; // "master" (mặc định nếu versions.json không ghi) | "m1" | "m2" | "h"
};
OtaEntry      otaList[OTA_MAX_VERSIONS];
int           otaListCount = 0;

//======================================================
// OTA: TẢI + PHÂN TÍCH versions.json TỪ GITHUB (chưa tải .bin nào cả)
// Trả về false nếu lỗi mạng/HTTP/JSON (đã tự gửi Telegram báo lỗi).
//
// [LORA-OTA] versions.json giờ có thể trộn lẫn bản của Master và của
// các slave, phân biệt bằng field "target" tuỳ chọn:
//   { "version": "1.3", "file": "slave_h_v1_3.bin", "target": "h" }
// Mục nào KHÔNG có "target" mặc định hiểu là "master" (tương thích
// ngược 100% với versions.json cũ đang dùng).
//======================================================
bool fetchOtaManifest() {
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
        // (mã lỗi HTTP là số, không cần escape)
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
        if (strlen(v) == 0 || strlen(f) == 0) continue; // bỏ qua mục thiếu dữ liệu bắt buộc
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

//======================================================
// [LORA-OTA] Tim entry ĐẦU TIÊN trong otaList[] khớp 1 target cho truoc
// ("master"/"m1"/"m2"/"h"). Tra ve -1 neu khong co ban nao cho target do.
//======================================================
int findLatestOtaIndexForTarget(const String& target) {
    for (int i = 0; i < otaListCount; i++) {
        if (otaList[i].target == target) return i;
    }
    return -1;
}

//======================================================
// [LORA-OTA] Cac ham OTA dung CHUNG cho ca Master (target="master") lan
// slave (target="m1"/"m2"/"h") - loc otaList[] theo target truoc khi
// hien thi/chon, tranh nham firmware giua cac node du chung 1
// versions.json. /update, /update_list, /update_to (cu, khong doi cach
// dung) chi la wrapper goi cac ham nay voi target="master".
//======================================================
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

//======================================================
// Đặt yêu cầu OTA đang chờ xác nhận (dùng chung cho /update, /update_to,
// /lora_update, /lora_update_to), rồi gửi tin nhắn hỏi xác nhận.
//======================================================
void setOtaPending(const String& version, const String& file, const String& target) {
    otaPendingVersion  = version;
    otaPendingFile     = file;
    otaPendingTarget   = target;
    otaPendingExpireAt = millis() + OTA_CONFIRM_TIMEOUT_MS;

    String curInfo = (target == "master") ? ("Hiện tại: " + escapeMarkdown(String(FW_VERSION)) + " → ") : "";
    String extraNote = (target == "master")
        ? "\n\nGõ /update_confirm để bắt đầu tải & cập nhật (thiết bị sẽ khởi động lại sau khi xong)."
        : "\n\n⚠️ Cập nhật QUA LORA cho slave có thể mất TỪ VÀI CHỤC PHÚT ĐẾN VÀI GIỜ tuỳ khoảng cách/dung lượng firmware — đây là kênh dự phòng khi không thể cắm dây trực tiếp, không phải thao tác nên làm thường xuyên.\nGõ /update_confirm để bắt đầu.";

    sendTelegramDirect(alertBlock(ICON_WARN, "Đã chọn bản để cập nhật [" + targetDisplayName(target) + "] — cần xác nhận",
        curInfo + "Chọn: " + escapeMarkdown(version) + " (" + escapeMarkdown(file) + ")" +
        extraNote +
        "\nGõ /update_cancel để huỷ."
        "\n⏳ Yêu cầu này hết hạn sau 5 phút nếu không xác nhận."));
}

//======================================================
// Kiểm tra nhanh bản MỚI NHẤT cho 1 target — dùng cho /update (target=
// "master") và /lora_update <slave>.
//======================================================
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
        otaPendingVersion = ""; otaPendingFile = ""; otaPendingTarget = "master"; // dọn sạch yêu cầu chờ xác nhận cũ (nếu có)
        sendTelegramDirect(alertBlock(ICON_OK, "Đã là bản mới nhất", "Bản hiện tại: " + escapeMarkdown(String(FW_VERSION))));
        return;
    }

    setOtaPending(otaList[latest].version, otaList[latest].file, target);
}

//======================================================
// Liệt kê TOÀN BỘ các bản có trong versions.json khớp 1 target — dùng
// cho /update_list (target="master") và /lora_update_list <slave>.
//======================================================
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
    msg += "\n\nGõ " + cmdHint + " <số thứ tự> hoặc <version> để chọn bản cần cài (có thể chọn cả bản CŨ hơn để rollback).";

    sendTelegramDirect(alertBlock(ICON_INFO, "Danh sách firmware [" + targetDisplayName(target) + "] trên GitHub", msg));
}

//======================================================
// Chọn 1 bản cụ thể theo số thứ tự (như hiển thị ở danh sách CÙNG
// target) hoặc theo đúng chuỗi version, rồi chuyển sang chờ xác nhận.
// Dùng cho /update_to (target="master") và /lora_update_to <slave>.
//======================================================
void selectOtaVersionForTarget(const String& target, String arg) {
    arg.trim();
    if (arg.length() == 0) {
        sendTelegramDirect(alertBlock(ICON_WARN, "Thiếu tham số",
            "Dùng: <lệnh> <số thứ tự>  hoặc  <lệnh> <version>\nGõ lệnh liệt kê danh sách trước để xem."));
        return;
    }

    if (!fetchOtaManifest()) return;

    int idxArr[OTA_MAX_VERSIONS];
    int n = filterOtaByTarget(target, idxArr, OTA_MAX_VERSIONS);
    if (n == 0) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Không tìm thấy bản nào cho " + targetDisplayName(target)));
        return;
    }

    // Ưu tiên hiểu là số thứ tự (1-based, THEO DANH SÁCH ĐÃ LỌC target này)
    // nếu toàn bộ ký tự là số.
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
            "\"" + escapeMarkdown(arg) + "\" không khớp số thứ tự hay version nào cho " + targetDisplayName(target) + ".\nGõ lệnh liệt kê danh sách để xem lại."));
        return;
    }

    setOtaPending(otaList[foundIdx].version, otaList[foundIdx].file, target);
}

//======================================================
// /update, /update_list, /update_to — GIỮ NGUYÊN cách dùng cũ, chỉ là
// wrapper gọi các hàm target-aware ở trên với target="master".
//======================================================
void checkOTAUpdate()             { checkOtaUpdateForTarget("master"); }
void listOtaVersions()            { listOtaVersionsForTarget("master"); }
void selectOtaVersion(String arg) { selectOtaVersionForTarget("master", arg); }

//======================================================
// OTA BƯỚC CUỐI: TẢI + FLASH FIRMWARE (chỉ chạy sau khi user gõ
// /update_confirm và yêu cầu chưa hết hạn). URL file .bin được ghép
// động từ otaPendingFile — không còn cố định tên "firmware.bin".
//======================================================
void performOTAUpdate() {
    String remoteVersion = otaPendingVersion;
    String remoteFile    = otaPendingFile;
    otaPendingVersion = ""; otaPendingFile = ""; // dùng 1 lần, xoá ngay để tránh xác nhận lặp lại

    otaRunning = true; // Chặn luồng check tin nhắn Telegram

    String firmwareUrl = String(OTA_RAW_BASE_URL) + remoteFile;

    sendTelegramDirect(alertBlock(ICON_WARN, "⬇️ Đang cập nhật firmware mới...",
        "Hiện tại: " + escapeMarkdown(String(FW_VERSION)) + " → Mới: " + escapeMarkdown(remoteVersion) + "\nFile: " + escapeMarkdown(remoteFile)));

    // Vòng lặp delay an toàn (1 giây) thay cho delay(1000) để gói tin Telegram kịp gửi đi
    for (int i = 0; i < 100; i++) {
        esp_task_wdt_reset();
        delay(10);
    }

    // Khai báo kết nối TLS riêng biệt hoàn toàn cho việc tải file BIN
    WiFiClientSecure fwClient;
    fwClient.setInsecure();
    fwClient.setTimeout(15000);        // Giới hạn 15s cho mỗi block dữ liệu tải về
    fwClient.setHandshakeTimeout(10);   // Handshake tối đa 10s

    httpUpdate.rebootOnUpdate(false);   // Tắt tự động reboot để ta chủ động xử lý xoá/thêm WDT

    // Xoá Task hiện tại khỏi danh sách giám sát của Watchdog trước khi ghi Flash
    esp_task_wdt_delete(NULL);

    // Bắt đầu quá trình tải và nạp code
    t_httpUpdate_return ret = httpUpdate.update(fwClient, firmwareUrl);

    if (ret == HTTP_UPDATE_OK) {
        // [FIX-OTA-NOTIFY] Trước đây thành công là restart NGAY, không hề
        // báo cho ai biết -> gửi thông báo thành công trước, rồi mới xác
        // nhận offset getUpdates + restart như cũ.
        sendTelegramDirect(alertBlock(ICON_SUCCESS, "✅ OTA thành công",
            "Đã cập nhật lên bản: " + escapeMarkdown(remoteVersion) + "\nThiết bị sẽ khởi động lại ngay bây giờ..."));
        for (int i = 0; i < 100; i++) { esp_task_wdt_reset(); delay(10); } // để kịp gửi trước khi restart

        // [FIX-20] Giống hệt cơ chế chống lặp của /reset: xác nhận offset
        // getUpdates với Telegram TRƯỚC khi restart, để sau khi OTA xong
        // và khởi động lại, bot KHÔNG đọc lại lệnh /update_confirm cũ
        // (tránh kích hoạt lại một vòng OTA/redirect không mong muốn).
        esp_task_wdt_add(NULL);
        bot.getUpdates(bot.last_message_received + 1);
        esp_task_wdt_delete(NULL);
        // Cập nhật thành công, thực hiện restart hệ thống ngay lập tức
        ESP.restart();
    } else {
        // Nếu thất bại: Khôi phục lại trạng thái hệ thống ban đầu
        esp_task_wdt_add(NULL); // Đưa Task trở lại sự giám sát của Watchdog
        otaRunning = false;     // Mở khoá luồng để Telegram tiếp tục hoạt động

        sendTelegramDirect(alertBlock(ICON_BAD, "OTA thất bại",
            "Lỗi #" + String(httpUpdate.getLastError()) + ": " + escapeMarkdown(httpUpdate.getLastErrorString())));
    }
}

//======================================================
// [LORA-OTA] TIỆN ÍCH TARGET SLAVE
//======================================================
bool isValidLoraTarget(const String& t) { return (t == "m1" || t == "m2" || t == "h"); }

uint8_t loraTargetToId(const String& t) {
    if (t == "m1") return SLAVE_M1_ID;
    if (t == "m2") return SLAVE_M2_ID;
    if (t == "h")  return SLAVE_H_ID;
    return 0xFF;
}

//======================================================
// [LORA-OTA] TẢI 1 FILE .bin TỪ GITHUB VỀ LittleFS, GHI TRỰC TIẾP TỪNG
// KHỐI NHỎ (không giữ cả file trong RAM — firmware ~1MB, RAM ESP32 có
// hạn). Trả về false + tự gửi Telegram báo lỗi nếu thất bại ở bất kỳ
// bước nào.
//======================================================
bool downloadToLittleFS(const String& url, const char* path, uint32_t& outSize) {
    if (WiFi.status() != WL_CONNECTED) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Chưa kết nối WiFi."));
        return false;
    }

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
        esp_task_wdt_reset();
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

//======================================================
// [LORA-OTA] GỬI 1 OtaPacket (MSG_OTA_START / MSG_OTA_DATA) VÀ CHỜ ACK
// TỪ SLAVE, CÓ RETRY. Trả về true nếu nhận ACK với payload=1 (OK).
// expectedAckSeq: giá trị "seq" mà slave sẽ đặt trong gói ACK phản hồi
// (START luôn ACK với seq=0; DATA ACK với seq=chunkIndex — xem
// sendOtaAck() phía slave).
//======================================================
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
        if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(LORA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            LoRa.beginPacket();
            LoRa.write((uint8_t*)&opkt, sizeof(OtaPacket));
            LoRa.endPacket(true);
            xSemaphoreGive(loraMutex);
        }

        portENTER_CRITICAL(&stateMux);
        otaAckReceived = false;
        portEXIT_CRITICAL(&stateMux);

        unsigned long waitStart = millis();
        while (millis() - waitStart < ackTimeoutMs) {
            esp_task_wdt_reset();
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

//======================================================
// [LORA-OTA] GỬI 1 Packet nhỏ (MSG_OTA_END / MSG_OTA_ABORT) VÀ CHỜ ACK,
// CÓ RETRY. seqVal là "seq" đặt trong gói GỬI ĐI (Master dùng để mang
// tổng số chunk cho END); expectedAckSeq là "seq" mong đợi trong gói
// ACK PHẢN HỒI của slave (END luôn ACK sentinel 0xFFFF, ABORT là 0xFFFE
// — xem handleOtaEnd()/handleOtaAbort() phía slave).
//======================================================
bool sendOtaSmallAndWaitAck(uint8_t targetId, uint8_t msgType, uint16_t seqVal,
                             uint16_t expectedAckSeq, uint8_t maxRetries, unsigned long ackTimeoutMs) {
    Packet pkt = { MASTER_ID, targetId, msgType, seqVal, 0, (uint32_t)millis() };

    for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
        if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(LORA_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            LoRa.beginPacket();
            LoRa.write((uint8_t*)&pkt, sizeof(Packet));
            LoRa.endPacket(true);
            xSemaphoreGive(loraMutex);
        }

        portENTER_CRITICAL(&stateMux);
        otaAckReceived = false;
        portEXIT_CRITICAL(&stateMux);

        unsigned long waitStart = millis();
        while (millis() - waitStart < ackTimeoutMs) {
            esp_task_wdt_reset();
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

//======================================================
// [LORA-OTA] TASK RIÊNG: ĐỌC FILE FIRMWARE ĐÃ TẢI SẴN TRONG LittleFS,
// TRUYỀN TỪNG CHUNK QUA LORA CHO SLAVE (START -> N x DATA -> END), CÓ
// ACK + RETRY MỖI CHUNK. Chạy độc lập với vTelegramTask/vLoRaRealtimeTask
// nên KHÔNG chặn Telegram hay điều khiển thủ công trong lúc truyền —
// việc này có thể mất từ vài chục phút đến vài giờ.
//======================================================
void vLoraOtaRelayTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    File f = LittleFS.open("/lora_ota.bin", "r");
    if (!f) {
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Không đọc lại được file firmware tạm."));
    } else {
        uint32_t totalSize   = f.size();
        uint16_t totalChunks = (uint16_t)((totalSize + OTA_CHUNK_SIZE - 1) / OTA_CHUNK_SIZE);

        // Tính CRC32 toàn bộ file (đọc tuần tự 1 lượt trước khi gửi)
        uint32_t crc = 0xFFFFFFFF;
        {
            uint8_t cbuf[256];
            while (f.available()) {
                int n = f.read(cbuf, sizeof(cbuf));
                if (n <= 0) break;
                crc = crc32Compute(crc, cbuf, n);
                esp_task_wdt_reset();
            }
            crc ^= 0xFFFFFFFF;
            f.seek(0);
        }

        otaLoraTotalChunks = totalChunks;
        otaLoraChunkIndex  = 0;

        sendTelegramDirect(alertBlock(ICON_WARN, "Bắt đầu cập nhật LoRa: " + targetDisplayName(otaLoraTargetKey),
            "Bản: " + escapeMarkdown(otaLoraVersion) + "\nDung lượng: " + String(totalSize) + " byte\nSố chunk: " + String(totalChunks) +
            "\n\n⚠️ Quá trình này có thể mất TỪ VÀI CHỤC PHÚT ĐẾN VÀI GIỜ tuỳ khoảng cách/nhiễu sóng."
            "\nGõ /lora_update_status để xem tiến độ, /lora_update_abort để huỷ giữa chừng."));

        bool started = sendOtaBigAndWaitAck(otaLoraTargetId, MSG_OTA_START, totalChunks, OTA_CHUNK_SIZE,
                                             totalSize, crc, NULL, /*expectedAckSeq=*/0, 5, 3000);
        if (!started) {
            sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại",
                targetDisplayName(otaLoraTargetKey) + " không phản hồi START (có thể mất sóng/tắt nguồn)."));
        } else {
            bool allOk = true;
            for (uint16_t i = 0; i < totalChunks; i++) {
                if (otaLoraCancelRequested) {
                    sendOtaSmallAndWaitAck(otaLoraTargetId, MSG_OTA_ABORT, 0, /*expectedAckSeq=*/0xFFFE, 2, 1000);
                    sendTelegramDirect(alertBlock(ICON_WARN, "Đã huỷ cập nhật LoRa", targetDisplayName(otaLoraTargetKey)));
                    allOk = false;
                    break;
                }

                uint8_t buf[OTA_CHUNK_SIZE];
                int n = f.read(buf, OTA_CHUNK_SIZE);
                if (n <= 0) { allOk = false; break; }

                uint32_t chunkCrc = crc32Compute(0xFFFFFFFF, buf, n) ^ 0xFFFFFFFF;
                bool chunkOk = sendOtaBigAndWaitAck(otaLoraTargetId, MSG_OTA_DATA, i, (uint16_t)n,
                                                     0, chunkCrc, buf, /*expectedAckSeq=*/i, 5, 4000);
                if (!chunkOk) {
                    sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại",
                        targetDisplayName(otaLoraTargetKey) + " không xác nhận chunk " + String(i) + " sau nhiều lần gửi lại."));
                    allOk = false;
                    break;
                }

                otaLoraChunkIndex = i + 1;
                if (totalChunks > 0 && ((i % 100 == 0) || (i == (uint16_t)(totalChunks - 1)))) {
                    int pct = (int)((long)(i + 1) * 100L / totalChunks);
                    sendTelegramDirect(alertBlock(ICON_INFO, "Tiến độ " + targetDisplayName(otaLoraTargetKey),
                        String(i + 1) + "/" + String(totalChunks) + " chunk (" + String(pct) + "%)"));
                }
                esp_task_wdt_reset();
            }

            if (allOk) {
                bool ended = sendOtaSmallAndWaitAck(otaLoraTargetId, MSG_OTA_END, totalChunks, /*expectedAckSeq=*/0xFFFF, 5, 5000);
                if (ended) {
                    sendTelegramDirect(alertBlock(ICON_SUCCESS, "Cập nhật LoRa THÀNH CÔNG",
                        targetDisplayName(otaLoraTargetKey) + " đã nhận đủ firmware và đang tự khởi động lại."));
                } else {
                    sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại ở bước cuối",
                        targetDisplayName(otaLoraTargetKey) + " không xác nhận kết thúc — kiểm tra lại thiết bị."));
                }
            }
        }
        f.close();
    }

    LittleFS.remove("/lora_ota.bin");
    otaLoraActive          = false;
    otaLoraCancelRequested = false;
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

//======================================================
// [LORA-OTA] BƯỚC CUỐI (target != "master"): TẢI FIRMWARE SLAVE VỀ
// LittleFS RỒI TẠO TASK RELAY QUA LORA. Chỉ block vTelegramTask trong
// lúc TẢI (vài giây, qua WiFi) — phần truyền qua LoRa (có thể mất rất
// lâu) chạy ở vLoraOtaRelayTask riêng.
//======================================================
void performLoraOtaUpdate() {
    String remoteVersion = otaPendingVersion;
    String remoteFile    = otaPendingFile;
    String target         = otaPendingTarget;
    otaPendingVersion = ""; otaPendingFile = ""; otaPendingTarget = "master";

    if (otaLoraActive) {
        sendTelegramDirect(alertBlock(ICON_WARN, "Đang có 1 phiên cập nhật LoRa khác chạy",
            "Gõ /lora_update_status để xem, hoặc /lora_update_abort để huỷ trước khi bắt đầu cái mới."));
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
        sendTelegramDirect(alertBlock(ICON_BAD, "Cập nhật LoRa thất bại", "Không mở được bộ nhớ tạm (LittleFS) trên Master."));
        return;
    }
    LittleFS.remove("/lora_ota.bin");

    uint32_t downloadedSize = 0;
    if (!downloadToLittleFS(firmwareUrl, "/lora_ota.bin", downloadedSize)) {
        return; // downloadToLittleFS đã tự gửi thông báo lỗi
    }

    otaLoraTargetKey       = target;
    otaLoraTargetId        = loraTargetToId(target);
    otaLoraVersion         = remoteVersion;
    otaLoraChunkIndex      = 0;
    otaLoraTotalChunks     = 0;
    otaLoraCancelRequested = false;
    otaLoraActive          = true;

    xTaskCreatePinnedToCore(vLoraOtaRelayTask, "LoraOtaRelay", 8192, NULL, 1, &otaLoraTaskHandle, 1);
}

//======================================================
// /lora_update_status — xem tiến độ phiên cập nhật LoRa đang chạy.
//======================================================
void showLoraOtaStatus() {
    if (!otaLoraActive) {
        sendTelegramDirect(alertBlock(ICON_INFO, "Không có phiên cập nhật LoRa nào đang chạy."));
        return;
    }
    uint16_t idx = otaLoraChunkIndex, total = otaLoraTotalChunks;
    int pct = (total > 0) ? (int)((long)idx * 100L / total) : 0;
    sendTelegramDirect(alertBlock(ICON_INFO, "Tiến độ cập nhật LoRa: " + targetDisplayName(otaLoraTargetKey),
        "Bản: " + escapeMarkdown(otaLoraVersion) + "\n" + String(idx) + "/" + String(total) + " chunk (" + String(pct) + "%)" +
        "\n\nGõ /lora_update_abort để huỷ giữa chừng."));
}

//======================================================
// /lora_update_abort — huỷ phiên cập nhật LoRa ĐANG CHẠY (khác với
// /update_cancel — lệnh đó chỉ huỷ yêu cầu CHƯA xác nhận).
//======================================================
void abortLoraOtaTransfer() {
    if (!otaLoraActive) {
        sendTelegramDirect(alertBlock(ICON_INFO, "Không có phiên cập nhật LoRa nào đang chạy để huỷ."));
        return;
    }
    otaLoraCancelRequested = true;
    sendTelegramDirect(alertBlock(ICON_WARN, "Đã gửi yêu cầu huỷ — đang chờ tác vụ dừng lại..."));
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

// [MULTI-USER] isAdmin quyết định có hiện các lệnh quản trị (cấu hình,
// OTA, quản lý người dùng...) hay chỉ hiện các lệnh giám sát/vận hành
// cơ bản dành cho USER thường.
String buildHelpMessage(bool isAdmin) {
    String h = "🇻🇳 *TRẠM BƠM MỎ ĐÁ — HƯỚNG DẪN*\n" + divider();
    h += "📊 *GIÁM SÁT*\n├─ `/status` — Tổng quan hệ thống\n├─ `/status_full` — Chi tiết cấu hình + Log lỗi\n├─ `/mucnuoc` — Mực nước téc (siêu âm)\n├─ `/pending` — Lệnh LoRa chưa nhận ACK\n├─ `/version` — Xem phiên bản hiện tại\n└─ `/myid` — Xem Chat ID của bạn\n\n";
    h += "⚙️ *VẬN HÀNH*\n├─ `/auto` — Chế độ TỰ ĐỘNG\n└─ `/manual` — Chế độ THỦ CÔNG\n\n";
    h += "🛠️ *THỦ CÔNG* (Gõ /manual trước)\n├─ `/bat_m1` `/tat_m1` — Bơm Moong 1\n├─ `/bat_m2` `/tat_m2` — Bơm Moong 2\n└─ `/bat_h` `/tat_h` — Bơm Trạm Hồ\n\n";
    if (!isAdmin) {
        h += "ℹ️ Một số lệnh cấu hình/OTA/quản lý người dùng chỉ dành cho ADMIN.";
        return h;
    }
    h += "🔑 *QUẢN LÝ NGƯỜI DÙNG (ADMIN)*\n├─ `/adduser <chat_id>` — Thêm người dùng\n├─ `/deluser <chat_id>` — Xóa người dùng\n├─ `/users` — Danh sách người dùng\n├─ `/admin` — Xem quyền của bạn\n└─ `/testall` — Gửi tin nhắn thử tới tất cả\n\n";
    h += "🚨 *KHẨN CẤP (ADMIN)*\n└─ `/reset` — Khởi động lại toàn bộ (giống nút RESET vật lý)\n\n";
    h += "📐 *CẤU HÌNH TỐC ĐỘ (ADMIN)*\n└─ `/seth [0-100]` — Đặt tốc độ nền khi chạy 1 bơm (%)\n";
    h += "🚰 *CẢM BIẾN MỰC NƯỚC TÉC (ADMIN)*\n├─ `/water_on` `/water_off` — Bật/tắt cảm biến siêu âm\n├─ `/set_tank_height <cm>` — Chiều cao téc (từ mặt nước cạn đến vị trí lắp cảm biến)\n└─ `/set_water_offset <cm>` — Khoảng cách đo lúc téc ĐẦY (bù trừ vùng chết cảm biến)\n\n";
    h += "🔒 *KHOÁ CHÉO BƠM THEO MỰC NƯỚC (ADMIN)*\n├─ `/set_water_pump_low <%>` — Dưới ngưỡng này: khoá M1+M2, chỉ bơm Hồ chạy (mặc định 20%)\n├─ `/set_water_pump_high <%>` — Từ ngưỡng này: khoá bơm Hồ, chỉ M1/M2 chạy (mặc định 100%)\n└─ `/set_water_pump_recover <%>` — Mực nước rút xuống ngưỡng này thì mở khoá bơm Hồ lại (mặc định 90%)\n\n";
    h += "📶 *MẠNG & BOT (ADMIN)*\n├─ `/list_wifi` — Xem DS WiFi đã lưu\n├─ `/del_wifi <số>` — Xóa WiFi theo STT\n├─ `/set_wifi <SSID>;<PASS>` — Thêm WiFi\n└─ `/set_token <TOKEN>;<ID>` — Đổi Token Bot\n";
    h += "🚀 *FIRMWARE (ADMIN)*\n├─ `/update` — Kiểm tra bản MỚI NHẤT từ GitHub\n├─ `/update_list` — Xem TẤT CẢ bản có sẵn (kể cả bản cũ để rollback)\n├─ `/update_to <số|version>` — Chọn 1 bản cụ thể từ danh sách\n├─ `/update_confirm` — Xác nhận, bắt đầu tải & cập nhật\n└─ `/update_cancel` — Huỷ yêu cầu cập nhật đang chờ\n";
    h += "📡 *FIRMWARE SLAVE QUA LORA (ADMIN)*\n├─ `/lora_update <m1|m2|h>` — Kiểm tra bản mới nhất cho slave đó\n├─ `/lora_update_list <m1|m2|h>` — Xem tất cả bản có sẵn cho slave đó\n├─ `/lora_update_to <m1|m2|h> <số|version>` — Chọn 1 bản cụ thể\n├─ `/update_confirm` / `/update_cancel` — dùng chung với Master ở trên\n├─ `/lora_update_status` — Xem tiến độ truyền đang chạy\n└─ `/lora_update_abort` — Huỷ giữa chừng phiên đang truyền\n⚠️ Cập nhật qua LoRa có thể mất từ vài chục phút đến vài giờ — chỉ dùng khi không thể cắm dây trực tiếp.\n";
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
    // [FIX-18] Snapshot thời gian chạy — cộng thêm phần đang chạy dở
    // (chưa chốt sổ) nếu bơm đang bật, để hiển thị số liệu tức thời.
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
    msg += "🚰 *MỰC NƯỚC TÉC (siêu âm)*\n└─ " + waterLevelText() + "\n\n";
    msg += "🔌 *TRẠNG THÁI BƠM*\n├─ Bơm Moong 1: " + pumpBadge(pumpM1) + "\n";
    msg += "├─ Bơm Moong 2: " + pumpBadge(pumpM2) + "\n";
    msg += "└─ BƠM HỒ     :      " + pumpBadge(pumpH) + (pumpH ? " (" + String(speedH) + "%)" : "") + "\n\n";

    // [FIX-18] Thời gian chạy bơm trong ngày hôm nay.
    if (trackedDayMarker == -1) {
        msg += "⏱️ *THỜI GIAN CHẠY BƠM HÔM NAY*\n" + String(ICON_WARN) + " Chưa đồng bộ được giờ hệ thống (NTP)\n\n";
    } else {
        msg += "⏱️ *THỜI GIAN CHẠY BƠM HÔM NAY*\n├─ Moong 1: " + formatTimeSpan(m1RunSec * 1000UL) +
               "\n├─ Moong 2: " + formatTimeSpan(m2RunSec * 1000UL) +
               "\n└─ Trạm Hồ: " + formatTimeSpan(hRunSec * 1000UL) + "\n\n";
    }

    msg += "📐 Cài đặt công suất bơm Hồ: *" + String(telegramSpeed) + "%*\n\n";
    if (!loraHardwareOk) {
        msg += "📡 *TÍN HIỆU LORA*\n" + String(ICON_CRIT) + " *Phần cứng LoRa lỗi — đang tự thử phục hồi nền mỗi 5 phút*\n\n";
    } else {
        msg += "📡 *TÍN HIỆU LORA*\n├─ Moong 1: " + statusBadge(sM1On) + (sM1On ? " " + signalBadge(rM1) : "") + "\n├─ Moong 2: " + statusBadge(sM2On) + (sM2On ? " " + signalBadge(rM2) : "") + "\n└─ Trạm Hồ: " + statusBadge(sHOn) + (sHOn ? " " + signalBadge(rH) : "") + "\n\n";
    }
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
        // [FIX-19] Giới hạn thời gian bắt tay TLS (mặc định có thể rất lâu
        // nếu mạng chập chờn) — góp phần giảm nguy cơ vTelegramTask bị treo
        // quá WDT_TIMEOUT_SEC. Nếu bản arduino-esp32 core đang dùng chưa có
        // hàm này (core cũ < 2.0.6), có thể xoá dòng dưới — cơ chế gỡ/thêm
        // watchdog quanh bot.sendMessage()/getUpdates() vẫn bảo vệ được.
        secured_client.setHandshakeTimeout(10);
        // [FIX-18] Đồng bộ giờ thực qua NTP (UTC+7, không DST) để biết
        // chính xác thời điểm sang ngày mới cho thống kê chạy bơm.
        configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
        sendTelegramAlert(alertBlock(ICON_SUCCESS, "Hệ thống đã kết nối", "Lý do khởi động: " + bootResetReason));
    }
}

// [FIX-17] Tách riêng 1 lần thử khởi tạo LoRa (không delay dài, không
// restart) để dùng chung cho cả initLoRa() lúc boot lẫn retry ngầm
// trong vLoRaRealtimeTask.
bool tryInitLoRaOnce() {
    if (LoRa.begin(LORA_FREQ)) {
        LoRa.setSignalBandwidth(LORA_BW);
        LoRa.setSpreadingFactor(LORA_SF);
        LoRa.setCodingRate4(LORA_CR);
        LoRa.setTxPower(TX_POWER);
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
        Serial.printf("[LORA] Khởi tạo thất bại (lần %d/%d)...\n", retry, LORA_INIT_MAX_RETRY);
        delay(1000);
    }

    if (ok) {
        loraHardwareOk = true;
        Serial.println("[LORA] Khởi tạo thành công.");
    } else {
        // [FIX-17] KHÔNG còn ESP.restart() ở đây. Vì setupWiFiSingle()
        // giờ chạy TRƯỚC hàm này trong setup(), WiFi/Telegram đã sẵn
        // sàng nên có thể cảnh báo NGAY thay vì lặp reboot trong im
        // lặng như trước. Hệ thống tiếp tục chạy WiFi/Telegram/lệnh
        // thủ công bình thường; vLoRaRealtimeTask sẽ tự thử phục hồi
        // ngầm mỗi LORA_HW_RETRY_MS mà không cần reboot toàn bộ.
        loraHardwareOk = false;
        Serial.println("[LORA] Lỗi phần cứng không phục hồi được sau nhiều lần thử — chuyển sang chế độ chờ, thử lại nền.");
        sendTelegramDirect(alertBlock(ICON_CRIT, "LỖI PHẦN CỨNG LORA",
                            "Không khởi tạo được module LoRa sau " + String(LORA_INIT_MAX_RETRY) + " lần thử.\n"
                            "Hệ thống VẪN chạy WiFi/Telegram để bạn theo dõi/điều khiển thủ công, "
                            "nhưng KHÔNG nhận được dữ liệu phao/bơm tự động cho tới khi khắc phục phần cứng "
                            "(kiểm tra nguồn 3.3V-5V, dây UART/M0/M1/AUX, module LoRa).\n"
                            "Hệ thống sẽ tự thử kết nối lại mỗi 5 phút."));
    }
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

    // [WATER-LEVEL] AJ-SR04M chế độ Trig/Echo.
    pinMode(WATER_TRIG_PIN, OUTPUT);
    digitalWrite(WATER_TRIG_PIN, LOW);
    pinMode(WATER_ECHO_PIN, INPUT);
}

void handleNewMessages(int n) {
    for (int i = 0; i < n; i++) {
        // [MULTI-USER] Trước đây chỉ nhận tin từ đúng 1 currentChatId.
        // Giờ nhận tin từ BẤT KỲ chat nào, rồi tự kiểm tra người đó đã
        // được Admin cấp quyền (/adduser) hay chưa.
        String senderChatId = bot.messages[i].chat_id;
        String text = bot.messages[i].text; String user = bot.messages[i].from_name;
        text.trim();

        // [MULTI-USER] /start và /myid luôn được phép, kể cả người lạ
        // chưa đăng ký — để họ lấy Chat ID gửi cho Admin xin cấp quyền.
        if (text.startsWith("/start") || text == "/myid") {
            String outMsg;
            if (isRegisteredUser(senderChatId)) {
                outMsg = buildHelpMessage(isAdminUser(senderChatId));
            } else {
                outMsg = "👋 *CHÀO MỪNG*\n" + divider();
                outMsg += "ID của bạn:\n`" + senderChatId + "`\n\n";
                outMsg += "Gửi ID này cho quản trị viên để được cấp quyền sử dụng bot.";
            }
            sendTelegramReplyTo(senderChatId, outMsg);
            bot.messages[i].text = ""; bot.messages[i].from_name = "";
            continue;
        }

        // [MULTI-USER] Chặn người lạ chưa được Admin /adduser với mọi
        // lệnh khác ngoài /start, /myid ở trên.
        if (!isRegisteredUser(senderChatId)) {
            String denyMsg = "❌ *Bạn chưa được cấp quyền sử dụng bot.*\n" + divider();
            denyMsg += "Chat ID của bạn:\n`" + senderChatId + "`\n\n";
            denyMsg += "Hãy gửi ID này cho quản trị viên.";
            sendTelegramReplyTo(senderChatId, denyMsg);
            bot.messages[i].text = ""; bot.messages[i].from_name = "";
            continue;
        }

        bool senderIsAdmin = isAdminUser(senderChatId);

        // [MULTI-USER] Các lệnh chỉ dành cho ADMIN — thay đổi cấu hình
        // hoặc ảnh hưởng tới toàn hệ thống/mọi người dùng.
        bool isAdminOnlyCmd =
            text == "/reset" || text.startsWith("/seth ") ||
            text.startsWith("/set_tank_height ") || text.startsWith("/set_water_offset ") ||
            text.startsWith("/set_water_pump_low ") || text.startsWith("/set_water_pump_high ") ||
            text.startsWith("/set_water_pump_recover ") ||
            text == "/water_on" || text == "/water_off" ||
            text.startsWith("/set_wifi") || text.startsWith("/del_wifi") ||
            text.startsWith("/set_token") ||
            text == "/update" || text == "/update_list" ||
            text.startsWith("/update_to") || text == "/update_confirm" || text == "/update_cancel" ||
            text.startsWith("/lora_update") ||
            text.startsWith("/adduser") || text.startsWith("/deluser") ||
            text == "/users" || text == "/admin" || text == "/testall";

        if (isAdminOnlyCmd && !senderIsAdmin) {
            sendTelegramReplyTo(senderChatId, alertBlock(ICON_WARN, "Không đủ quyền", "Lệnh này chỉ dành cho ADMIN."));
            bot.messages[i].text = ""; bot.messages[i].from_name = "";
            continue;
        }

        if (text == "/help") {
            String outMsg = buildHelpMessage(senderIsAdmin);
            sendTelegramReplyTo(senderChatId, outMsg);
        }
        else if (text.startsWith("/adduser")) {
            // Cú pháp: /adduser <chat_id>            -> thêm quyền USER
            //          /adduser <chat_id> admin       -> thêm quyền ADMIN
            String args = text.substring(8); args.trim();
            String targetId = args; uint8_t role = ROLE_USER;
            int sp = args.indexOf(' ');
            if (sp != -1) {
                targetId = args.substring(0, sp); targetId.trim();
                String roleArg = args.substring(sp + 1); roleArg.trim();
                if (roleArg == "admin") role = ROLE_ADMIN;
            }
            String replyMsg;
            if (targetId.length() == 0) {
                replyMsg = "⚠️ *SAI CÚ PHÁP*\nDùng: `/adduser <chat_id>` hoặc `/adduser <chat_id> admin`";
            } else if (findUserIndex(targetId) >= 0) {
                replyMsg = alertBlock(ICON_WARN, "Người dùng đã tồn tại", "ID: `" + targetId + "`");
            } else if (addUserToList(targetId, role)) {
                replyMsg = "✅ *Đã thêm người dùng*\n" + divider();
                replyMsg += "ID: `" + targetId + "`\nQuyền: " + String(role == ROLE_ADMIN ? "ADMIN" : "USER") +
                            "\nTổng số người dùng: " + String(userCount);
            } else {
                replyMsg = alertBlock(ICON_BAD, "Không thể thêm", "Danh sách đã đầy (" + String(MAX_USERS) + ")");
            }
            sendTelegramReplyTo(senderChatId, replyMsg);
        }
        else if (text.startsWith("/deluser")) {
            String args = text.substring(8); args.trim();
            String replyMsg;
            if (args.length() == 0) {
                replyMsg = "⚠️ *SAI CÚ PHÁP*\nDùng: `/deluser <chat_id>`";
            } else if (args == senderChatId) {
                replyMsg = alertBlock(ICON_WARN, "Không thể tự xóa chính mình");
            } else if (removeUserFromList(args)) {
                replyMsg = "✅ *Đã xóa người dùng*\n" + divider();
                replyMsg += "ID: `" + args + "`\nTổng số người dùng: " + String(userCount);
            } else {
                replyMsg = alertBlock(ICON_BAD, "Không tìm thấy người dùng này", "ID: `" + args + "`");
            }
            sendTelegramReplyTo(senderChatId, replyMsg);
        }
        else if (text == "/users") {
            sendTelegramReplyTo(senderChatId, buildUsersListMessage());
        }
        else if (text == "/admin") {
            String m = "🔑 *THÔNG TIN QUYỀN ADMIN*\n" + divider();
            m += "ID của bạn: `" + senderChatId + "`\n\n";
            m += "Có quyền:\n├─ OTA (`/update`...)\n├─ Reset / Restart\n├─ Thêm / Xóa người dùng\n├─ Cấu hình WiFi / Token / Tốc độ bơm\n└─ Gửi thử toàn bộ (`/testall`)";
            sendTelegramReplyTo(senderChatId, m);
        }
        else if (text == "/testall") {
            sendTelegramDirect(alertBlock(ICON_SUCCESS, "Đây là tin nhắn kiểm tra.", "Gửi bởi Admin: " + user));
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
        else if (text == "/mucnuoc") {
            readWaterLevelSensor();   // đo ngay lập tức thay vì chờ chu kỳ nền
            String outMsg = "🚰 *MỰC NƯỚC TÉC*\n" + divider() + waterLevelText() +
                             "\n\n📏 Téc cao: " + String(waterTankHeightCm) + "cm" +
                             "\n📐 Offset: " + String(waterFullOffsetCm) + "cm" +
                             "\n\n🔒 *Khoá chéo bơm:*" +
                             "\n├─ Ngưỡng thấp (khoá M1/M2): " + String(waterPumpBlockLowPercent) + "%" +
                             "\n├─ Ngưỡng đầy (khoá Hồ): " + String(waterPumpBlockHighPercent) + "%" +
                             "\n├─ Ngưỡng hồi phục Hồ: " + String(waterPumpHighRecoverPercent) + "%" +
                             "\n└─ Bơm Hồ hiện đang: " + (waterPumpHBlockedByFull ? "🔒 BỊ KHOÁ (téc đầy)" : "🔓 Không bị khoá");
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
        else if (text == "/reset") {
            // [FIX-16] Khởi động lại toàn bộ = tương đương nút RESET vật
            // lý. Dùng sendTelegramDirect (gửi NGAY, đồng bộ) thay vì
            // sendTelegramAlert (xếp hàng đợi) — vì hàng đợi chỉ được xử
            // lý ở vòng lặp SAU của vTelegramTask, mà ESP.restart() bên
            // dưới sẽ chặn luôn vòng lặp đó, khiến tin nhắn không kịp gửi.
            sendTelegramDirect(alertBlock(ICON_CRIT, "KHỞI ĐỘNG LẠI THỦ CÔNG (giống nút RESET vật lý)",
                                "Yêu cầu bởi: " + user +
                                "\nCấu hình đã lưu (WiFi/Token/tốc độ bơm/trạng thái phao) vẫn được giữ nguyên."));
            // [FIX-20] QUAN TRỌNG: bot.last_message_received đã được cập
            // nhật trong RAM khi getUpdates() lấy về lệnh /reset này,
            // nhưng Telegram server CHỈ coi update đó là "đã xác nhận"
            // khi client gọi getUpdates() LẦN TIẾP THEO với offset mới.
            // Nếu ESP.restart() ngay bây giờ mà chưa gọi lại, Telegram
            // vẫn giữ update /reset này ở trạng thái "chưa xác nhận" —
            // sau khi reboot, last_message_received về lại 0, thiết bị
            // gọi getUpdates(1) và bị Telegram trả lại ĐÚNG lệnh /reset
            // cũ đó lần nữa -> xử lý -> restart -> lặp vô hạn (đúng hiện
            // tượng đã gặp). Gọi thêm 1 lần getUpdates() ở đây để xác
            // nhận offset trước khi restart, chặn đứng vòng lặp.
            esp_task_wdt_delete(NULL);
            bot.getUpdates(bot.last_message_received + 1);
            esp_task_wdt_add(NULL);
            delay(500);
            ESP.restart();
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
        else if (text == "/water_on" || text == "/water_off") {
            bool en = (text == "/water_on");
            waterSensorEnabled = en;
            waterCfgPrefs.begin("water_cfg", false);
            waterCfgPrefs.putBool("enabled", en);
            waterCfgPrefs.end();
            if (!en) {
                // Tắt cảm biến -> xoá số liệu cũ, tránh /status hiển thị
                // % đo từ trước khi tắt trông như vẫn còn hiệu lực.
                waterSensorOk = false;
                waterLevelPercent = -1;
                waterLowAlertSent = false; waterHighAlertSent = false;
            }
            cachedStatusShort = ""; cachedStatusFull = "";
            sendTelegramAlert(alertBlock(en ? ICON_SUCCESS : ICON_INFO,
                               en ? "Đã BẬT cảm biến mực nước téc" : "Đã TẮT cảm biến mực nước téc",
                               "Bởi " + user));
        }
        else if (text.startsWith("/set_tank_height ")) {
            int cm = text.substring(17).toInt();
            if (cm <= 0) {
                sendTelegramReplyTo(senderChatId, "⚠️ *SAI GIÁ TRỊ*\nChiều cao téc phải là số cm > 0.\nVD: `/set_tank_height 200`");
            } else {
                waterTankHeightCm = cm;
                waterCfgPrefs.begin("water_cfg", false);
                waterCfgPrefs.putInt("height", cm);
                waterCfgPrefs.end();
                cachedStatusShort = ""; cachedStatusFull = "";
                sendTelegramAlert(alertBlock(ICON_INFO, "Cài đặt chiều cao téc", String(cm) + "cm bởi " + user));
            }
        }
        else if (text.startsWith("/set_water_offset ")) {
            int cm = text.substring(18).toInt();
            if (cm < 0) {
                sendTelegramReplyTo(senderChatId, "⚠️ *SAI GIÁ TRỊ*\nOffset phải là số cm >= 0.\nVD: `/set_water_offset 20`");
            } else {
                waterFullOffsetCm = cm;
                waterCfgPrefs.begin("water_cfg", false);
                waterCfgPrefs.putInt("offset", cm);
                waterCfgPrefs.end();
                cachedStatusShort = ""; cachedStatusFull = "";
                sendTelegramAlert(alertBlock(ICON_INFO, "Cài đặt offset mực nước téc", String(cm) + "cm bởi " + user));
            }
        }
        // [WATER-INTERLOCK] Ngưỡng % dưới đó khoá M1+M2, chỉ còn bơm Hồ
        // được phép chạy.
        else if (text.startsWith("/set_water_pump_low ")) {
            int pct = text.substring(20).toInt();
            if (pct < 0 || pct > 100) {
                sendTelegramReplyTo(senderChatId, "⚠️ *SAI GIÁ TRỊ*\nNgưỡng phải là % từ 0-100.\nVD: `/set_water_pump_low 20`");
            } else {
                waterPumpBlockLowPercent = pct;
                waterCfgPrefs.begin("water_cfg", false);
                waterCfgPrefs.putInt("pump_low", pct);
                waterCfgPrefs.end();
                cachedStatusShort = ""; cachedStatusFull = "";
                sendTelegramAlert(alertBlock(ICON_INFO, "Cài đặt ngưỡng khoá bơm M1/M2 (mực nước thấp)",
                                   String(pct) + "% bởi " + user + "\nDưới ngưỡng này: khoá M1+M2, chỉ còn bơm Hồ chạy."));
            }
        }
        // [WATER-INTERLOCK] Ngưỡng % từ đó trở lên khoá bơm Hồ, chỉ còn
        // M1/M2 được phép chạy.
        else if (text.startsWith("/set_water_pump_high ")) {
            int pct = text.substring(21).toInt();
            if (pct < 0 || pct > 100) {
                sendTelegramReplyTo(senderChatId, "⚠️ *SAI GIÁ TRỊ*\nNgưỡng phải là % từ 0-100.\nVD: `/set_water_pump_high 100`");
            } else if (pct <= waterPumpHighRecoverPercent) {
                sendTelegramReplyTo(senderChatId, "⚠️ *SAI GIÁ TRỊ*\nNgưỡng ĐẦY (" + String(pct) + "%) phải LỚN HƠN ngưỡng hồi phục hiện tại (" + String(waterPumpHighRecoverPercent) + "%).\nHãy hạ ngưỡng hồi phục trước bằng `/set_water_pump_recover`.");
            } else {
                waterPumpBlockHighPercent = pct;
                waterCfgPrefs.begin("water_cfg", false);
                waterCfgPrefs.putInt("pump_high", pct);
                waterCfgPrefs.end();
                cachedStatusShort = ""; cachedStatusFull = "";
                sendTelegramAlert(alertBlock(ICON_INFO, "Cài đặt ngưỡng khoá bơm Hồ (téc đầy)",
                                   String(pct) + "% bởi " + user + "\nTừ ngưỡng này trở lên: khoá bơm Hồ, chỉ còn M1/M2 chạy."));
            }
        }
        // [WATER-INTERLOCK] Ngưỡng % mà khi mực nước rút xuống tới đó thì
        // mở khoá lại cho bơm Hồ chạy tiếp (chống rung ở sát ngưỡng ĐẦY).
        else if (text.startsWith("/set_water_pump_recover ")) {
            int pct = text.substring(24).toInt();
            if (pct < 0 || pct > 100) {
                sendTelegramReplyTo(senderChatId, "⚠️ *SAI GIÁ TRỊ*\nNgưỡng phải là % từ 0-100.\nVD: `/set_water_pump_recover 90`");
            } else if (pct >= waterPumpBlockHighPercent) {
                sendTelegramReplyTo(senderChatId, "⚠️ *SAI GIÁ TRỊ*\nNgưỡng hồi phục (" + String(pct) + "%) phải NHỎ HƠN ngưỡng ĐẦY hiện tại (" + String(waterPumpBlockHighPercent) + "%).");
            } else {
                waterPumpHighRecoverPercent = pct;
                waterCfgPrefs.begin("water_cfg", false);
                waterCfgPrefs.putInt("pump_rec", pct);
                waterCfgPrefs.end();
                cachedStatusShort = ""; cachedStatusFull = "";
                sendTelegramAlert(alertBlock(ICON_INFO, "Cài đặt ngưỡng hồi phục bơm Hồ",
                                   String(pct) + "% bởi " + user + "\nMực nước rút xuống ngưỡng này thì bơm Hồ được mở khoá lại."));
            }
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
        else if (text == "/version") {
            sendTelegramDirect(alertBlock(ICON_INFO, "Phiên bản firmware hiện tại", String(FW_VERSION)));
        }
        else if (text == "/update") {
            sendTelegramDirect(alertBlock(ICON_INFO, "Đang kiểm tra bản mới nhất trên GitHub..."));
            checkOTAUpdate();
        }
        else if (text == "/update_list") {
            sendTelegramDirect(alertBlock(ICON_INFO, "Đang tải danh sách firmware từ GitHub..."));
            listOtaVersions();
        }
        else if (text.startsWith("/update_to")) {
            String args = text.substring(10); args.trim();
            selectOtaVersion(args);
        }
        else if (text.startsWith("/lora_update_list")) {
            int sp1 = text.indexOf(' ');
            String args = (sp1 > 0) ? text.substring(sp1 + 1) : "";
            args.trim(); args.toLowerCase();
            if (!isValidLoraTarget(args)) {
                sendTelegramDirect(alertBlock(ICON_WARN, "Thiếu / sai tham số", "Dùng: /lora_update_list <m1|m2|h>"));
            } else {
                sendTelegramDirect(alertBlock(ICON_INFO, "Đang tải danh sách firmware từ GitHub..."));
                listOtaVersionsForTarget(args);
            }
        }
        else if (text.startsWith("/lora_update_to")) {
            int sp1 = text.indexOf(' ');
            String rest = (sp1 > 0) ? text.substring(sp1 + 1) : "";
            rest.trim();
            int sp2 = rest.indexOf(' ');
            if (sp2 == -1) {
                sendTelegramDirect(alertBlock(ICON_WARN, "Thiếu tham số", "Dùng: /lora_update_to <m1|m2|h> <số thứ tự|version>"));
            } else {
                String tgt = rest.substring(0, sp2); tgt.trim(); tgt.toLowerCase();
                String arg = rest.substring(sp2 + 1); arg.trim();
                if (!isValidLoraTarget(tgt)) {
                    sendTelegramDirect(alertBlock(ICON_WARN, "Sai target", "Dùng: /lora_update_to <m1|m2|h> <số thứ tự|version>"));
                } else {
                    selectOtaVersionForTarget(tgt, arg);
                }
            }
        }
        else if (text == "/lora_update_status") {
            showLoraOtaStatus();
        }
        else if (text == "/lora_update_abort") {
            abortLoraOtaTransfer();
        }
        else if (text.startsWith("/lora_update")) {
            int sp1 = text.indexOf(' ');
            String args = (sp1 > 0) ? text.substring(sp1 + 1) : "";
            args.trim(); args.toLowerCase();
            if (!isValidLoraTarget(args)) {
                sendTelegramDirect(alertBlock(ICON_WARN, "Thiếu / sai tham số", "Dùng: /lora_update <m1|m2|h>"));
            } else {
                sendTelegramDirect(alertBlock(ICON_INFO, "Đang kiểm tra bản mới nhất cho " + targetDisplayName(args) + " trên GitHub..."));
                checkOtaUpdateForTarget(args);
            }
        }
        else if (text == "/update_confirm") {
            if (otaPendingVersion.length() == 0) {
                sendTelegramDirect(alertBlock(ICON_WARN, "Không có yêu cầu cập nhật nào đang chờ xác nhận.",
                    "Gõ /update, /update_list, /lora_update hoặc /lora_update_list để chọn bản trước."));
            } else if ((long)(millis() - otaPendingExpireAt) > 0) {
                otaPendingVersion = ""; otaPendingFile = ""; otaPendingTarget = "master";
                sendTelegramDirect(alertBlock(ICON_WARN, "Yêu cầu cập nhật đã hết hạn (quá 5 phút).",
                    "Gõ lại lệnh chọn bản từ đầu."));
            } else if (otaPendingTarget == "master") {
                performOTAUpdate();
            } else {
                performLoraOtaUpdate();
            }
        }
        else if (text == "/update_cancel") {
            if (otaPendingVersion.length() == 0) {
                sendTelegramDirect(alertBlock(ICON_INFO, "Không có yêu cầu cập nhật nào để huỷ."));
            } else {
                otaPendingVersion = ""; otaPendingFile = ""; otaPendingTarget = "master";
                sendTelegramDirect(alertBlock(ICON_OK, "Đã huỷ yêu cầu cập nhật."));
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
    // [LORA-OTA] MSG_OTA_ACK la ACK rieng cho phien cap nhat firmware qua
    // LoRa (xem vLoraOtaRelayTask) — khac voi MSG_ACK thuong, khong can
    // Master gui lai 1 ACK khac cho no.
    bool     isOtaAck    = (pkt.msgType == MSG_OTA_ACK);

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
            } else if (pkt.msgType == MSG_HEARTBEAT && (pkt.payload & FLOAT_DATA_VALID_BIT)) {
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
    lastLoraRetryAttempt            = millis();   // [FIX-17]

    for (;;) {
        esp_task_wdt_reset();

        // [FIX-17] Phần cứng LoRa đang lỗi: thử phục hồi ngầm mỗi
        // LORA_HW_RETRY_MS, KHÔNG reboot toàn bộ. Task Telegram (core
        // khác) không bị ảnh hưởng — vẫn theo dõi/điều khiển thủ công
        // được bình thường trong lúc chờ.
        if (!loraHardwareOk) {
            if (millis() - lastLoraRetryAttempt >= LORA_HW_RETRY_MS) {
                lastLoraRetryAttempt = millis();
                Serial.println("[LORA] Thử phục hồi phần cứng...");
                if (tryInitLoRaOnce()) {
                    loraHardwareOk = true;
                    sendTelegramAlert(alertBlock(ICON_SUCCESS, "LoRa đã phục hồi",
                                       "Hệ thống tiếp tục hoạt động bình thường."));
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

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
        // [OTA] Nếu đang tải/flash firmware thì bỏ qua việc kiểm tra tin
        // nhắn Telegram (tránh dùng chung TLS client với OTA cùng lúc).
        if (otaRunning) {
            for (int i = 0; i < 20; i++) {
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(10)); // Delay an toàn tổng 200ms
            }
            continue;
        }

        if (pendingWifiSwitch && millis() - wifiSwitchRequestedAt >= WIFI_SWITCH_DELAY_MS) {
            pendingWifiSwitch = false; WiFi.disconnect(true, true); vTaskDelay(pdMS_TO_TICKS(200)); WiFi.mode(WIFI_STA);
            // [FIX-19] scanNetworks() có thể mất nhiều giây bất thường trong
            // môi trường nhiễu RF -> tạm gỡ khỏi watchdog để không bị reset oan.
            esp_task_wdt_delete(NULL);
            scanAndPickBestWifi();
            esp_task_wdt_add(NULL);
            WiFi.begin(currentSsid.c_str(), currentPass.c_str());
            wifiWasDown = true; wifiDownSince = millis(); wifiDisabledPermanently = false; lastReconnectAttempt = millis();
        }

        if (wifiDisabledPermanently) {
            if (millis() - lastWifiWakeAttempt >= WIFI_WAKE_INTERVAL_MS) {
                lastWifiWakeAttempt = millis(); WiFi.mode(WIFI_STA);
                esp_task_wdt_delete(NULL);          // [FIX-19]
                scanAndPickBestWifi();
                esp_task_wdt_add(NULL);
                WiFi.begin(currentSsid.c_str(), currentPass.c_str());
                unsigned long ws = millis();
                while (WiFi.status() != WL_CONNECTED && millis() - ws < WIFI_WAKE_TRY_MS) { esp_task_wdt_reset(); vTaskDelay(pdMS_TO_TICKS(200)); }
                if (WiFi.status() == WL_CONNECTED) { wifiDisabledPermanently = false; wifiDownSince = 0; wifiWasDown = true; }
                else { WiFi.disconnect(true, true); WiFi.mode(WIFI_OFF); }
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }

        if (WiFi.status() == WL_CONNECTED) {
            if (wifiWasDown) { wifiWasDown = false; wifiDownSince = 0; sendTelegramAlert(alertBlock(ICON_SUCCESS, "Hệ thống khôi phục kết nối WiFi")); }
            String msg = "";
            if (xSemaphoreTake(telegramQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (!telegramQueue.empty()) { msg = telegramQueue.front(); telegramQueue.pop(); }
                xSemaphoreGive(telegramQueueMutex);
            }
            if (msg.length() > 0) {
                bool md = (msg.indexOf('*') != -1);
                // [FIX-19] setTimeout() của WiFiClientSecure chỉ giới hạn thời
                // gian ĐỌC dữ liệu, KHÔNG giới hạn DNS resolve + TCP connect +
                // TLS handshake. Khi mạng chập chờn, bước connect() có thể
                // treo lâu hơn WDT_TIMEOUT_SEC (60s) -> gây "WATCHDOG TASK -
                // Có task bị treo". Tạm gỡ task khỏi watchdog trong lúc gọi
                // API mạng (vốn dĩ có thể chậm tự nhiên, không phải deadlock
                // thật), rồi đăng ký lại ngay sau.
                esp_task_wdt_delete(NULL);
                // [MULTI-USER] Broadcast: mọi cảnh báo trong hàng đợi được
                // gửi tới TẤT CẢ người dùng đã đăng ký, không chỉ 1 người.
                if (userCount > 0) {
                    for (int u = 0; u < userCount; u++) {
                        bot.sendMessage(userIds[u], msg, md ? "Markdown" : "");
                    }
                } else {
                    bot.sendMessage(currentChatId, msg, md ? "Markdown" : "");
                }
                esp_task_wdt_add(NULL);
            }
            if (millis() - lastTelegramPoll > TELEGRAM_POLL_MS) {
                esp_task_wdt_delete(NULL);          // [FIX-19]
                int n = bot.getUpdates(bot.last_message_received + 1);
                esp_task_wdt_add(NULL);              // [FIX-19]
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
                lastReconnectAttempt = millis(); WiFi.disconnect();
                esp_task_wdt_delete(NULL);          // [FIX-19]
                scanAndPickBestWifi();
                esp_task_wdt_add(NULL);
                WiFi.begin(currentSsid.c_str(), currentPass.c_str());
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
    if (aM1) sendTelegramAlert(alertBlock(ICON_CRIT, "Hệ thống ngắt LoRa Moong 1 (>10p)"));
    if (aM2) sendTelegramAlert(alertBlock(ICON_CRIT, "Hệ thống ngắt LoRa Moong 2 (>10p)"));
    if (aH)  sendTelegramAlert(alertBlock(ICON_CRIT, "Hệ thống ngắt LoRa Trạm Hồ (>10p)"));
}

// [WATER-LEVEL] Đo 1 lần khoảng cách (cm) bằng xung Trig/Echo chuẩn
// HC-SR04/AJ-SR04M. Trả về -1 nếu timeout (không có phản hồi / vật quá
// xa / mất kết nối cảm biến).
float waterMeasureOnceCm() {
    digitalWrite(WATER_TRIG_PIN, LOW);
    delayMicroseconds(3);
    digitalWrite(WATER_TRIG_PIN, HIGH);
    delayMicroseconds(12);
    digitalWrite(WATER_TRIG_PIN, LOW);

    unsigned long durationUs = pulseIn(WATER_ECHO_PIN, HIGH, WATER_PULSE_TIMEOUT_US);
    if (durationUs == 0) return -1;   // timeout, không có phản hồi

    // Vận tốc âm thanh ~343 m/s ở 25°C → 0.0343 cm/us, chia 2 vì đi + về.
    float distCm = (durationUs * 0.0343f) / 2.0f;
    return distCm;
}

// Sắp xếp nhỏ->lớn mảng 3 phần tử (đủ dùng cho WATER_SAMPLE_COUNT nhỏ).
void waterSortSamples(float *a, int n) {
    for (int i = 0; i < n - 1; i++)
        for (int j = i + 1; j < n; j++)
            if (a[j] < a[i]) { float t = a[i]; a[i] = a[j]; a[j] = t; }
}

// [WATER-LEVEL] Đo nhiều lần lấy trung vị (chống nhiễu do sóng phản xạ
// lệch/nhiễu điện), cập nhật waterLevelDistanceCm/waterLevelPercent/
// waterSensorOk, và tự gửi cảnh báo Telegram khi vượt ngưỡng thấp/cao
// (có debounce để tránh spam khi mực nước dao động quanh ngưỡng).
void readWaterLevelSensor() {
    // [WATER-CFG] Cảm biến đang TẮT (mặc định lúc mới flash, hoặc admin
    // tắt tay bằng /water_off) -> không phát xung Trig (tránh đọc rác
    // nếu phần cứng chưa đấu xong), giữ nguyên trạng thái "chưa có số
    // liệu" để /status và /mucnuoc hiển thị đúng là cảm biến đang tắt.
    if (!waterSensorEnabled) {
        waterSensorOk = false;
        return;
    }

    float samples[WATER_SAMPLE_COUNT];
    int   validCount = 0;
    for (int i = 0; i < WATER_SAMPLE_COUNT; i++) {
        float d = waterMeasureOnceCm();
        if (d > 0) samples[validCount++] = d;
        delay(15);   // khoảng nghỉ ngắn giữa các lần đo để tránh dội sóng
    }

    if (validCount == 0) {
        waterSensorOk = false;
        return;   // giữ nguyên giá trị % lần đo trước, không ghi đè bằng rác
    }

    waterSortSamples(samples, validCount);
    float distCm = samples[validCount / 2];   // trung vị

    waterLevelDistanceCm = distCm;
    waterSensorOk = true;

    float usableHeight = waterTankHeightCm - waterFullOffsetCm;
    // [FIX] Offset chỉ được trừ MỘT LẦN — ở usableHeight (thu hẹp thang đo
    // về đúng dải cảm biến đo được). Không được trừ thêm lần nữa ở levelCm,
    // nếu không téc đầy thật (distCm == offset) sẽ không bao giờ ra 100%,
    // và chỉnh offset gần như không có tác dụng lên % hiển thị.
    float levelCm = waterTankHeightCm - distCm;
    if (usableHeight <= 0) usableHeight = waterTankHeightCm;   // an toàn, tránh chia 0

    int pct = (int) round((levelCm / usableHeight) * 100.0f);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    waterLevelPercent = pct;

    unsigned long now = millis();
    if (now - lastWaterAlertMs >= WATER_ALERT_DEBOUNCE_MS) {
        if (pct <= WATER_LOW_PERCENT && !waterLowAlertSent) {
            sendTelegramAlert(alertBlock(ICON_WARN, "Mực nước téc THẤP", String(pct) + "% (còn lại)"));
            waterLowAlertSent = true; waterHighAlertSent = false;
            lastWaterAlertMs = now;
        } else if (pct >= WATER_HIGH_PERCENT && !waterHighAlertSent) {
            sendTelegramAlert(alertBlock(ICON_OK, "Mực nước téc gần ĐẦY", String(pct) + "%"));
            waterHighAlertSent = true; waterLowAlertSent = false;
            lastWaterAlertMs = now;
        } else if (pct > WATER_LOW_PERCENT && pct < WATER_HIGH_PERCENT) {
            waterLowAlertSent = false; waterHighAlertSent = false;
        }
    }
}

// [WATER-LEVEL] Chuỗi hiển thị % mực nước dùng chung cho /status và
// lệnh /mucnuoc — thống nhất 1 chỗ để dễ sửa định dạng sau này.
String waterLevelText() {
    if (!waterSensorEnabled) {
        return String(ICON_WARN) + " Cảm biến đang TẮT (bật bằng `/water_on`)";
    }
    if (!waterSensorOk || waterLevelPercent < 0) {
        return String(ICON_WARN) + " Không đọc được cảm biến";
    }
    return String(waterLevelPercent) + "% (cách mặt nước ~" + String(waterLevelDistanceCm, 0) + "cm)";
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
// [FIX-18] Lấy "mã ngày" hiện tại theo giờ thực đã đồng bộ NTP. Trả về
// false nếu chưa đồng bộ được giờ (mới boot/mất mạng) — tránh reset
// bậy số liệu khi chưa biết ngày thật là ngày nào.
bool getCurrentDayMarker(int &dayMarker) {
    time_t now = time(nullptr);
    if (now < 1700000000) return false;   // mốc ~2023 — nhỏ hơn nghĩa là NTP chưa chạy
    struct tm ti;
    localtime_r(&now, &ti);
    dayMarker = ti.tm_year * 1000 + ti.tm_yday;   // duy nhất theo năm + ngày trong năm
    return true;
}

// [FIX-18] Cập nhật 1 bơm khi chuyển BẬT/TẮT: ghi mốc lúc bật, cộng dồn
// giây chạy lúc tắt. Gọi trong critical section (biến volatile dùng
// chung giữa loop() và vTelegramTask).
void trackPumpOnOff(bool isOn, volatile unsigned long &onSinceMs, volatile unsigned long &totalSec, unsigned long nowMs) {
    if (isOn && onSinceMs == 0) {
        onSinceMs = nowMs;
    } else if (!isOn && onSinceMs != 0) {
        totalSec += (nowMs - onSinceMs) / 1000UL;
        onSinceMs = 0;
    }
}

// [FIX-18] Chốt sổ thời gian chạy đang tích luỹ (nếu bơm đang bật) rồi
// lưu tổng vào NVS — gọi định kỳ để không mất số liệu nếu Master bị
// treo/reset giữa ngày.
void checkpointPumpRuntime() {
    unsigned long nowMs = millis();
    portENTER_CRITICAL(&stateMux);
    if (pumpM1OnSinceMs != 0) { pumpM1RuntimeTodaySec += (nowMs - pumpM1OnSinceMs) / 1000UL; pumpM1OnSinceMs = nowMs; }
    if (pumpM2OnSinceMs != 0) { pumpM2RuntimeTodaySec += (nowMs - pumpM2OnSinceMs) / 1000UL; pumpM2OnSinceMs = nowMs; }
    if (pumpHOnSinceMs  != 0) { pumpHRuntimeTodaySec  += (nowMs - pumpHOnSinceMs)  / 1000UL; pumpHOnSinceMs  = nowMs; }
    unsigned long m1 = pumpM1RuntimeTodaySec, m2 = pumpM2RuntimeTodaySec, h = pumpHRuntimeTodaySec;
    portEXIT_CRITICAL(&stateMux);

    if (trackedDayMarker != -1) {
        runtimePrefs.begin("runtime_cfg", false);
        runtimePrefs.putInt("day", trackedDayMarker);
        runtimePrefs.putULong("m1", m1);
        runtimePrefs.putULong("m2", m2);
        runtimePrefs.putULong("h",  h);
        runtimePrefs.end();
    }
}

// [FIX-18] Gọi mỗi vòng loop(): phát hiện lần đầu có giờ NTP (nạp lại
// số liệu đã lưu nếu vẫn cùng ngày) và phát hiện sang ngày mới (chốt
// sổ hôm qua rồi reset về 0 cho hôm nay).
void updatePumpRuntimeDayRollover() {
    int dayMarker;
    if (!getCurrentDayMarker(dayMarker)) return;

    if (trackedDayMarker == -1) {
        runtimePrefs.begin("runtime_cfg", true);
        int           savedDay = runtimePrefs.getInt("day", -1);
        unsigned long savedM1  = runtimePrefs.getULong("m1", 0);
        unsigned long savedM2  = runtimePrefs.getULong("m2", 0);
        unsigned long savedH   = runtimePrefs.getULong("h", 0);
        runtimePrefs.end();

        if (savedDay == dayMarker) {
            portENTER_CRITICAL(&stateMux);
            pumpM1RuntimeTodaySec = savedM1;
            pumpM2RuntimeTodaySec = savedM2;
            pumpHRuntimeTodaySec  = savedH;
            portEXIT_CRITICAL(&stateMux);
        }
        trackedDayMarker = dayMarker;
    } else if (dayMarker != trackedDayMarker) {
        // Sang ngày mới: chốt sổ hôm qua trước, rồi reset về 0.
        checkpointPumpRuntime();
        unsigned long nowMs = millis();
        portENTER_CRITICAL(&stateMux);
        pumpM1RuntimeTodaySec = 0; if (pumpM1OnSinceMs != 0) pumpM1OnSinceMs = nowMs;
        pumpM2RuntimeTodaySec = 0; if (pumpM2OnSinceMs != 0) pumpM2OnSinceMs = nowMs;
        pumpHRuntimeTodaySec  = 0; if (pumpHOnSinceMs  != 0) pumpHOnSinceMs  = nowMs;
        portEXIT_CRITICAL(&stateMux);
        trackedDayMarker = dayMarker;

        runtimePrefs.begin("runtime_cfg", false);
        runtimePrefs.putInt("day", dayMarker);
        runtimePrefs.putULong("m1", 0);
        runtimePrefs.putULong("m2", 0);
        runtimePrefs.putULong("h", 0);
        runtimePrefs.end();

        sendTelegramAlert(alertBlock(ICON_INFO, "Sang ngày mới", "Đã reset thống kê thời gian chạy bơm."));
    }
}

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

    // [PHASE-LINK] Trạm điện báo mất pha -> ép TẮT bơm Moong ngay, bất kể
    // AUTO hay thủ công, KHÔNG chờ logic phao/manual bên dưới.
    if (remotePhaseLossActive) {
        localPumpM1 = false;
        localPumpM2 = false;
    }

    // --- [WATER-INTERLOCK] Mực nước téc (đo bằng cảm biến siêu âm) hợp lệ
    // khi nào? Chỉ khi cảm biến đang BẬT và lần đo gần nhất OK. Nếu cảm
    // biến tắt hoặc chưa/mất số liệu -> KHÔNG áp dụng khoá chéo (không có
    // dữ liệu để quyết định), giữ nguyên toàn bộ logic bơm cũ như cũ.
    bool waterLevelValid = waterSensorEnabled && waterSensorOk && waterLevelPercent >= 0;

    // Mực nước < ngưỡng THẤP -> khoá cứng M1+M2, CHỈ áp dụng ở chế độ
    // THỦ CÔNG (đè lên lệnh /bat_m1 /bat_m2 thủ công ở trên). Ở chế độ
    // TỰ ĐỘNG, cảm biến siêu âm KHÔNG can thiệp — logic bơm M1/M2 hoàn
    // toàn theo phao như cũ. Bơm Hồ KHÔNG bị ép — chỉ đơn thuần không bị
    // khoá, vẫn theo logic sẵn có (auto theo phao, hoặc thủ công theo
    // lệnh admin).
    bool waterLowBlockM1M2 = waterLevelValid && (currentMode == MODE_MANUAL)
                           && (waterLevelPercent < waterPumpBlockLowPercent);
    if (waterLowBlockM1M2) {
        localPumpM1 = false;
        localPumpM2 = false;
    }

    // --- Tính trạng thái bơm H ---
    bool    targetPumpH  = false;
    uint8_t targetSpeedH = 0;

    if (remotePhaseLossActive) {
        // [PHASE-LINK] Mất pha: khoá luôn bơm Trạm Hồ, không hút/không đẩy.
        targetPumpH  = false;
        targetSpeedH = 0;
    } else if (currentMode == MODE_AUTO) {
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

    // --- [WATER-INTERLOCK] Mực nước >= ngưỡng CAO (téc ĐẦY) -> khoá cứng
    // bơm Hồ, CHỈ áp dụng ở chế độ THỦ CÔNG. Ở chế độ TỰ ĐỘNG, cảm biến
    // siêu âm KHÔNG can thiệp vào bơm Hồ — giữ nguyên logic phao cũ. Có
    // độ trễ (hysteresis) bằng latch waterPumpHBlockedByFull: một khi đã
    // khoá do đầy, chỉ mở khoá lại khi mực nước rút xuống <= ngưỡng HỒI
    // PHỤC, tránh đóng/cắt rung liên tục ngay sát ngưỡng CAO. Giữa 2
    // ngưỡng thì giữ nguyên trạng thái latch hiện tại (không đổi). Latch
    // vẫn được cập nhật ở mọi chế độ để không bị "trễ" khi chuyển từ
    // AUTO sang THỦ CÔNG, nhưng chỉ THỦ CÔNG mới áp dụng khoá thực tế.
    if (waterLevelValid) {
        if (waterLevelPercent >= waterPumpBlockHighPercent) {
            waterPumpHBlockedByFull = true;
        } else if (waterLevelPercent <= waterPumpHighRecoverPercent) {
            waterPumpHBlockedByFull = false;
        }
    }
    if (waterLevelValid && waterPumpHBlockedByFull && (currentMode == MODE_MANUAL)) {
        targetPumpH  = false;
        targetSpeedH = 0;
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

    // --- [FIX-18] Theo dõi thời gian chạy bơm trong ngày ---
    portENTER_CRITICAL(&stateMux);
    trackPumpOnOff(localPumpM1, pumpM1OnSinceMs, pumpM1RuntimeTodaySec, now);
    trackPumpOnOff(localPumpM2, pumpM2OnSinceMs, pumpM2RuntimeTodaySec, now);
    trackPumpOnOff(targetPumpH,  pumpHOnSinceMs,  pumpHRuntimeTodaySec,  now);
    portEXIT_CRITICAL(&stateMux);

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
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    delay(100);

    logResetReason();

    randomSeed(esp_random());
    telegramQueueMutex = xSemaphoreCreateMutex();
    loraMutex          = xSemaphoreCreateMutex();
    coreSyncMutex       = xSemaphoreCreateMutex();

    // [LORA-OTA] Mount LittleFS ngay lúc boot (format nếu lần đầu/lỗi)
    // để sẵn sàng làm nơi lưu tạm file .bin slave trước khi relay qua
    // LoRa — không phụ thuộc lúc nào user gõ /lora_update mới mount.
    if (!LittleFS.begin(true)) {
        Serial.println("[LORA-OTA] CANH BAO: khong mount duoc LittleFS.");
    }

    for (int i = 0; i < QUEUE_SIZE; i++) pendingTable[i].used = false;
    for (int i = 0; i < ALERT_LOG_SIZE; i++) alertLog[i] = "";

    loadWifiCredentials();
    loadTelegramCredentials();
    loadUsersFromNVS();   // [MULTI-USER] Nạp danh sách Admin/User đã lưu trong NVS

    configPrefs.begin("pump_cfg", true);
    telegramSpeed = configPrefs.getInt("h_speed", 100);
    configPrefs.end();

    // [WATER-CFG] Nạp chiều cao téc/offset/trạng thái bật-tắt cảm biến
    // đã lưu (nếu chưa từng chỉnh qua Telegram thì dùng các giá trị
    // *_DEFAULT khai báo cùng chỗ với WATER_TRIG_PIN/WATER_ECHO_PIN).
    waterCfgPrefs.begin("water_cfg", true);
    waterTankHeightCm  = waterCfgPrefs.getInt("height", WATER_TANK_HEIGHT_CM_DEFAULT);
    waterFullOffsetCm  = waterCfgPrefs.getInt("offset", WATER_FULL_OFFSET_CM_DEFAULT);
    waterSensorEnabled = waterCfgPrefs.getBool("enabled", WATER_SENSOR_ENABLED_DEFAULT);
    // [WATER-INTERLOCK] Nạp 3 ngưỡng khoá chéo bơm Moong <-> bơm Hồ.
    waterPumpBlockLowPercent    = waterCfgPrefs.getInt("pump_low",  WATER_PUMP_BLOCK_LOW_PERCENT_DEFAULT);
    waterPumpBlockHighPercent   = waterCfgPrefs.getInt("pump_high", WATER_PUMP_BLOCK_HIGH_PERCENT_DEFAULT);
    waterPumpHighRecoverPercent = waterCfgPrefs.getInt("pump_rec",  WATER_PUMP_HIGH_RECOVER_PERCENT_DEFAULT);
    waterCfgPrefs.end();

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
    // [FIX-17] WiFi/Telegram khởi tạo TRƯỚC LoRa — để nếu LoRa hỏng
    // phần cứng, hệ thống vẫn gửi được cảnh báo từ xa thay vì lặp
    // reboot trong im lặng (xem chi tiết ở initLoRa()).
    setupWiFiSingle();
    initLoRa();
    initWatchdog();

    esp_task_wdt_add(NULL);

    bot.longPoll = 0;
    unsigned long ms = millis();
    slaveM1LastSeen = ms; slaveM2LastSeen = ms; slaveHLastSeen = ms; bootTime = ms;
    lastResourceCheck = ms; lastAutoRebootCheck = ms;
    lastRuntimeCheckpoint = ms;   // [FIX-18]

    // lastMasterFloatSent = FLOAT_FULL (đã khởi tạo khi khai báo)
    // → lần đầu phao cạn sẽ luôn trigger gửi lệnh H

    xTaskCreatePinnedToCore(vLoRaRealtimeTask, "LoRaTask",     4096, NULL, 5, &loraTaskHandle,     0);
    // [OTA] Tăng từ 8192 -> 16384 byte: /update chạy WiFiClientSecure (TLS)
    // + HTTPClient + HTTPUpdate lồng bên trong task này. 8192 byte quá sát,
    // dễ gây stack overflow (crash/reset im lặng, không kịp gửi lỗi về Telegram)
    // đúng lúc đang tải/flash firmware.
    xTaskCreatePinnedToCore(vTelegramTask,     "TelegramTask", 16384, NULL, 1, &telegramTaskHandle, 1);
}

void loop() {
    esp_task_wdt_reset();
    checkSlaveTimeout();
    readMasterFloat();
    debounceSlaveFloats();
    checkFloatUnknownAlert();
    if (millis() - lastPhaseLinkPollMs >= PHASE_LINK_POLL_MS) {   // [PHASE-LINK]
        lastPhaseLinkPollMs = millis();
        pollTramDienPhaseStatus();
    }
    if (millis() - lastWaterReadMs >= WATER_READ_INTERVAL_MS) {   // [WATER-LEVEL]
        lastWaterReadMs = millis();
        readWaterLevelSensor();
    }
    processControl();
    updatePumpRuntimeDayRollover();   // [FIX-18]
    if (millis() - lastRuntimeCheckpoint >= RUNTIME_CHECKPOINT_MS) {   // [FIX-18]
        checkpointPumpRuntime();
        lastRuntimeCheckpoint = millis();
    }
    monitorSystemResources();
    checkScheduledReboot();
    vTaskDelay(pdMS_TO_TICKS(20));
}
