#ifndef MAP_GRID_H
#define MAP_GRID_H

#include <stdint.h>
#include <stdbool.h>
#include "tof_xshut.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * ĐỊNH NGHĨA GIÁ TRỊ Ô MA TRẬN (CELL VALUES)
 * ========================================================================== */
#define MAP_VAL_FREE           0U    /* 0: Không vật cản (Đường trống, đã quét sạch) */
#define MAP_VAL_UNEXPLORED     1U    /* 1: Map chưa quét (Vùng chưa thám hiểm) */
#define MAP_VAL_NEW_OBSTACLE   10U   /* 10: Mới phát hiện (ToF vừa quét thấy cản mới) */
#define MAP_VAL_OBSTACLE       255U  /* 255: Vật cản cố định / Tường / Vùng cấm chữ U */

/* ==========================================================================
 * THÔNG SỐ KÍCH THƯỚC BẢN ĐỒ (SA BÀN 180 cm x 120 cm, ĐỘ PHÂN GIẢI 3 cm)
 * ========================================================================== */
#define MAP_WIDTH_MM           1800.0f  /* 180 cm = 1800 mm */
#define MAP_HEIGHT_MM          1200.0f  /* 120 cm = 1200 mm */
#define MAP_CELL_SIZE_MM       30.0f    /* 3 cm = 30 mm / ô */

#define MAP_COLS               60U      /* 1800 / 30 = 60 cột (X: 0 -> 59) */
#define MAP_ROWS               40U      /* 1200 / 30 = 40 hàng (Y: 0 -> 39) */
#define MAP_TOTAL_CELLS        (MAP_COLS * MAP_ROWS) /* 2400 ô = 2400 bytes RAM */

/* Kích thước trụ vật cản chuẩn 10 cm x 10 cm (100 mm x 100 mm) */
#define STATIC_OBS_SIZE_MM     100.0f

/* Số lượng vật cản tĩnh cố định */
#define STATIC_OBSTACLE_COUNT  17U

/* Cấu trúc tọa độ vật cản tĩnh */
typedef struct {
    const char *name;
    float x_mm;
    float y_mm;
} Map_ObstacleDef;

/* Ma trận lưu trữ bản đồ điểm (2400 bytes) */
extern uint8_t g_map_grid[MAP_ROWS][MAP_COLS];

/* ==========================================================================
 * CÁC HÀM XỬ LÝ BẢN ĐỒ
 * ========================================================================== */

/**
 * @brief Khởi tạo bản đồ:
 *        - Đặt toàn bộ map về giá trị 1 (Chưa quét)
 *        - Đánh dấu vùng cấm chữ U (X: 60-120cm, Y: 0-60cm) thành 255
 *        - Đánh dấu 17 trụ cản tĩnh T1 -> T17 thành 255
 *        - Kích hoạt 2 vị trí cản ngẫu nhiên được báo trước thành 255
 *        - Đặt vị trí xuất phát của xe (quanh X=30cm, Y=0cm) thành 0 (Không cản)
 *
 * @param slot_top Chọn vị trí cản nhóm trên: 1 hoặc 2 (hoặc 0 nếu chưa biết)
 * @param slot_bottom Chọn vị trí cản nhóm dưới: 3 hoặc 4 (hoặc 0 nếu chưa biết)
 */
void Map_Init(uint8_t slot_top, uint8_t slot_bottom);

/**
 * @brief Chuyển tọa độ thực tế (mm) sang chỉ số (cột, hàng) trong ma trận
 * @return true nếu nằm trong giới hạn bản đồ, false nếu ngoài biên
 */
bool Map_MmToGrid(float x_mm, float y_mm, uint8_t *col, uint8_t *row);

/**
 * @brief Chuyển chỉ số (cột, hàng) sang tọa độ tâm ô thực tế (mm)
 */
void Map_GridToMm(uint8_t col, uint8_t row, float *x_mm, float *y_mm);

/**
 * @brief Đọc giá trị ô tại tọa độ lưới (col, row)
 */
uint8_t Map_GetCell(uint8_t col, uint8_t row);

/**
 * @brief Ghi giá trị vào ô tại tọa độ lưới (col, row)
 */
void Map_SetCell(uint8_t col, uint8_t row, uint8_t val);

/**
 * @brief Kiểm tra xem một tọa độ có bị coi là cản không
 *        (Giá trị 255 hoặc 10 coi là cản, ngoài biên cũng là cản)
 */
bool Map_IsBlocked(float x_mm, float y_mm);

/**
 * @brief Đánh dấu một vùng hình chữ nhật vào ma trận
 */
void Map_MarkBox(float x_min_mm, float x_max_mm, float y_min_mm, float y_max_mm, uint8_t val);

/**
 * @brief Cập nhật bản đồ bằng tia quét ToF (Bresenham Raycasting):
 *        - Các ô trên đường tia đi qua: gán = 0 (Không cản)
 *        - Ô tại điểm va đập (cuối tia): gán = 10 (Mới phát hiện cản)
 *
 * @param sensor_x_mm Tọa độ X đầu cảm biến ToF (mm)
 * @param sensor_y_mm Tọa độ Y đầu cảm biến ToF (mm)
 * @param hit_x_mm    Tọa độ X điểm tia chạm cản (mm)
 * @param hit_y_mm    Tọa độ Y điểm tia chạm cản (mm)
 * @param hit_valid   true nếu ToF đo được khoảng cách hợp lệ (< MAX_RANGE), false nếu out-of-range
 */
void Map_UpdateRay(float sensor_x_mm, float sensor_y_mm, float hit_x_mm, float hit_y_mm, bool hit_valid);

/**
 * @brief Hàm tiện ích cập nhật trực tiếp từ số liệu đo của cảm biến ToF
 *
 * @param robot_x_mm     Tọa độ X hiện tại của xe (mm)
 * @param robot_y_mm     Tọa độ Y hiện tại của xe (mm)
 * @param robot_heading  Góc hướng của xe (độ, 0 -> 360)
 * @param tof_dist_mm    Khoảng cách đo được từ cảm biến ToF (mm)
 * @param sensor_id      Cảm biến trái (0), giữa (1) hoặc phải (2)
 * @param max_range_mm   Tầm đo tin cậy tối đa của ToF (ví dụ 1200 mm)
 */
void Map_UpdateFromTof(float robot_x_mm, float robot_y_mm, float robot_heading,
                       float tof_dist_mm, TofXshut_Sensor sensor_id, float max_range_mm);

/**
 * @brief Cập nhật điểm chạm ToF đã tính toán tọa độ (x_hit, y_hit) và quét tia:
 *        - Tia đi từ vị trí cảm biến tới (hit_x, hit_y): các ô trên đường tia gán = 0 (Free)
 *        - Ô tại (hit_x, hit_y) gán = 10 (Mới phát hiện) nếu is_valid_hit == true
 */
void Map_UpdateHitPoint(float robot_x_mm, float robot_y_mm, float robot_heading_deg,
                        float hit_x_mm, float hit_y_mm,
                        TofXshut_Sensor sensor_id, bool is_valid_hit);

/**
 * @brief Cập nhật đồng thời cả 3 cảm biến ToF từ các tọa độ đã tính toán sẵn (x_left, y_left,...)
 */
void Map_UpdateAllSensors(float robot_x_mm, float robot_y_mm, float robot_heading_deg,
                          float x_left, float y_left, bool left_valid,
                          float x_mid, float y_mid, bool mid_valid,
                          float x_right, float y_right, bool right_valid);

/**
 * @brief Xác nhận vật cản mới phát hiện (từ 10 nâng lên 255)
 */
void Map_ConfirmObstacle(uint8_t col, uint8_t row);

/**
 * @brief Trích xuất toàn bộ dữ liệu bản đồ 60x40 dạng ASCII qua UART2 sang ESP32
 *        để debug trên Web Serial (thống kê số ô, tọa độ cản mới, lưới ma trận)
 */
void Map_DumpToUart(void);

#ifdef __cplusplus
}
#endif

#endif /* MAP_GRID_H */
