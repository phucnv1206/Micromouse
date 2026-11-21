#ifndef MICROMOUSE_ASTAR_H
#define MICROMOUSE_ASTAR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * TRẠNG THÁI HỆ THỐNG ĐIỀU KHIỂN TỰ ĐỘNG (AUTO STATE MACHINE)
 * ========================================================================== */
typedef enum {
    AUTO_STATE_IDLE = 0,        /* Xe đứng yên chờ bấm nút */
    AUTO_STATE_STARTING,        /* Nút 1 đã bấm: LED 2 nhấp nháy ~1s đếm ngược */
    AUTO_STATE_RUNNING,         /* Đèn 2 sáng liên tục, xe đang chạy theo lộ trình */
    AUTO_STATE_AVOIDING,        /* Gặp cản bất thường: phanh dừng và tính đường né A* */
    AUTO_STATE_COMPLETED,       /* Xe đã về đích thành công, dừng xe và LED 2 sáng liên tục */
    AUTO_STATE_ERROR            /* Báo lỗi nếu gặp sự cố không tìm được đường né */
} AutoState;

/* Cấu trúc tọa độ điểm mốc (Waypoint) tính bằng mm */
typedef struct {
    float x_mm;
    float y_mm;
} AutoWaypoint;

/* Số điểm mốc tối đa cho lộ trình */
#define AUTO_MAX_WAYPOINTS 24U

/* ==========================================================================
 * GIAO DIỆN HÀM ĐIỀU KHIỂN CHẠY TỰ ĐỘNG VÀ THUẬT TOÁN A*
 * ========================================================================== */

/**
 * @brief Khởi tạo hệ thống tự động
 */
void Auto_Init(void);

/**
 * @brief Cập nhật dữ liệu cảm biến khoảng cách ToF từ chu kỳ debug/telemetry
 */
void Auto_UpdateSensors(float leftMm, bool leftValid,
                        float midMm, bool midValid,
                        float rightMm, bool rightValid);

/**
 * @brief Vòng lặp điều khiển tự động, gọi định kỳ trong loop() (100 Hz)
 */
void Auto_Update(float dt_s);

/**
 * @brief Sự kiện bấm Nút 1: Kích hoạt chạy tự động
 *        - Đèn LED 2 nhấp nháy ~1 giây (đếm ngược xuất phát)
 *        - Sau 1 giây: LED 2 sáng liên tục và xe bắt đầu chạy ngay lập tức
 *        - Xe chạy theo lộ trình 5 điểm mốc tối ưu
 *        - Nếu gặp vật cản bất ngờ chặn đường: dừng phanh, chạy A* né cản rồi nhập lại lộ trình
 *        - Khi cán đích (Y <= -80 mm, X >= 1200 mm): Dừng xe, LED 2 giữ sáng liên tục
 */
void Auto_OnButton1(void);

/**
 * @brief Sự kiện bấm Nút 2: Dừng khẩn cấp / Reset
 *        - Dừng động cơ ngay lập tức
 *        - Reset Odometry về vị trí xuất phát (300, -45, 90°)
 *        - Tắt toàn bộ đèn LED
 *        - Đưa hệ thống về trạng thái IDLE
 */
void Auto_OnButton2(void);

/**
 * @brief Lấy trạng thái hiện tại của bộ điều khiển tự động
 */
AutoState Auto_GetState(void);

/**
 * @brief Kiểm tra xem có đang trong chế độ quét bản đồ hay không
 */
bool Auto_IsMappingActive(void);

/**
 * @brief Dừng toàn bộ tiến trình tự động
 */
void Auto_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MICROMOUSE_ASTAR_H */
