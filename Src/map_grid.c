#include "map_grid.h"
#include "navigation.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

/* Ma trận bản đồ 60 cột x 40 hàng (2400 bytes) */
uint8_t g_map_grid[MAP_ROWS][MAP_COLS];

/* Danh sách 17 vật cản tĩnh cố định T1 -> T17 (tọa độ mm) */
static const Map_ObstacleDef g_static_obstacles[STATIC_OBSTACLE_COUNT] = {
    {"T1",   50.0f,  150.0f},
    {"T2",  550.0f,  250.0f},
    {"T3",  300.0f,  350.0f},
    {"T4",  550.0f,  450.0f},
    {"T5",   50.0f,  550.0f},
    {"T6",   50.0f,  750.0f},
    {"T7",  350.0f,  900.0f},
    {"T8",  450.0f, 1150.0f},
    {"T9",  650.0f,  750.0f},
    {"T10", 750.0f,  650.0f},
    {"T11", 850.0f, 1000.0f},
    {"T12", 950.0f,  650.0f},
    {"T13", 1150.0f, 1150.0f},
    {"T14", 1250.0f,  850.0f},
    {"T15", 1450.0f,  450.0f},
    {"T16", 1450.0f,  150.0f},
    {"T17", 1750.0f,  250.0f}
};

/* 4 vị trí cản ngẫu nhiên tiềm năng */
static const Map_ObstacleDef g_candidate_slots[4] = {
    {"Slot1", 1450.0f, 650.0f}, /* Nhóm trên 1 */
    {"Slot2", 1550.0f, 950.0f}, /* Nhóm trên 2 */
    {"Slot3", 1550.0f, 350.0f}, /* Nhóm dưới 3 */
    {"Slot4", 1350.0f, 550.0f}  /* Nhóm dưới 4 */
};

/* Chuyển tọa độ mm sang chỉ số cột và hàng */
bool Map_MmToGrid(float x_mm, float y_mm, uint8_t *col, uint8_t *row) {
    if (x_mm < 0.0f || x_mm >= MAP_WIDTH_MM || y_mm < 0.0f || y_mm >= MAP_HEIGHT_MM) {
        return false;
    }

    int c = (int)(x_mm / MAP_CELL_SIZE_MM);
    int r = (int)(y_mm / MAP_CELL_SIZE_MM);

    if (c < 0) c = 0;
    if (c >= (int)MAP_COLS) c = (int)MAP_COLS - 1;
    if (r < 0) r = 0;
    if (r >= (int)MAP_ROWS) r = (int)MAP_ROWS - 1;

    *col = (uint8_t)c;
    *row = (uint8_t)r;
    return true;
}

/* Chuyển chỉ số cột và hàng sang tọa độ tâm ô (mm) */
void Map_GridToMm(uint8_t col, uint8_t row, float *x_mm, float *y_mm) {
    if (col >= MAP_COLS) col = MAP_COLS - 1;
    if (row >= MAP_ROWS) row = MAP_ROWS - 1;

    *x_mm = ((float)col + 0.5f) * MAP_CELL_SIZE_MM;
    *y_mm = ((float)row + 0.5f) * MAP_CELL_SIZE_MM;
}

/* Đọc giá trị ô */
uint8_t Map_GetCell(uint8_t col, uint8_t row) {
    if (col >= MAP_COLS || row >= MAP_ROWS) {
        return MAP_VAL_OBSTACLE;
    }
    return g_map_grid[row][col];
}

/* Ghi giá trị ô */
void Map_SetCell(uint8_t col, uint8_t row, uint8_t val) {
    if (col < MAP_COLS && row < MAP_ROWS) {
        g_map_grid[row][col] = val;
    }
}

/* Kiểm tra xem một điểm mm có bị cản không */
bool Map_IsBlocked(float x_mm, float y_mm) {
    uint8_t c, r;
    if (!Map_MmToGrid(x_mm, y_mm, &c, &r)) {
        return true; /* Ngoài biên map coi như cản */
    }
    uint8_t val = g_map_grid[r][c];
    return (val == MAP_VAL_OBSTACLE || val == MAP_VAL_NEW_OBSTACLE);
}

/* Đánh dấu một vùng hình chữ nhật vào bản đồ */
void Map_MarkBox(float x_min_mm, float x_max_mm, float y_min_mm, float y_max_mm, uint8_t val) {
    int col_start = (int)(x_min_mm / MAP_CELL_SIZE_MM);
    /* [FIX 3.5] Trừ epsilon để cạnh trên không tràn sang ô kề (phình +30mm) */
    int col_end   = (int)((x_max_mm - 0.01f) / MAP_CELL_SIZE_MM);
    int row_start = (int)(y_min_mm / MAP_CELL_SIZE_MM);
    int row_end   = (int)((y_max_mm - 0.01f) / MAP_CELL_SIZE_MM);

    if (col_start < 0) col_start = 0;
    if (col_end >= (int)MAP_COLS) col_end = (int)MAP_COLS - 1;
    if (row_start < 0) row_start = 0;
    if (row_end >= (int)MAP_ROWS) row_end = (int)MAP_ROWS - 1;

    for (int r = row_start; r <= row_end; r++) {
        for (int c = col_start; c <= col_end; c++) {
            g_map_grid[r][c] = val;
        }
    }
}

/* Khởi tạo toàn bộ bản đồ theo chuẩn sa bàn */
void Map_Init(uint8_t slot_top, uint8_t slot_bottom) {
    /* 1. Mặc định toàn bộ sa bàn là 1 (Chưa quét) */
    memset(g_map_grid, MAP_VAL_UNEXPLORED, sizeof(g_map_grid));

    /* 2. Điền VÙNG CẤM TRUNG TÂM CHỮ U:
     *    X: [600, 1200] mm -> Cột 20 đến 39 (20 cột)
     *    Y: [0, 600] mm    -> Hàng 0 đến 19 (20 hàng)
     *    Giá trị = 255 (Vật cản)
     */
    Map_MarkBox(600.0f, 1200.0f, 0.0f, 600.0f, MAP_VAL_OBSTACLE);

    /* 3. Điền 17 TRỤ CẢN TĨNH T1 -> T17 (mỗi trụ 100 x 100 mm)
     *    Giá trị = 255 (Vật cản)
     */
    for (uint8_t i = 0; i < STATIC_OBSTACLE_COUNT; i++) {
        float cx = g_static_obstacles[i].x_mm;
        float cy = g_static_obstacles[i].y_mm;
        Map_MarkBox(cx - 50.0f, cx + 50.0f, cy - 50.0f, cy + 50.0f, MAP_VAL_OBSTACLE);
    }

    /* 4. Kích hoạt 2 vị trí cản ngẫu nhiên được báo trước:
     *    slot_top: 1 hoặc 2
     *    slot_bottom: 3 hoặc 4
     */
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t slotId = i + 1;
        if (slotId == slot_top || slotId == slot_bottom) {
            float cx = g_candidate_slots[i].x_mm;
            float cy = g_candidate_slots[i].y_mm;
            Map_MarkBox(cx - 50.0f, cx + 50.0f, cy - 50.0f, cy + 50.0f, MAP_VAL_OBSTACLE);
        }
    }

    /* 5. Vùng xuất phát của xe (quanh tọa độ 300 mm, 0 mm):
     *    Đánh dấu là 0 (Không vật cản) vì xe đang đỗ tại đó
     */
    Map_MarkBox(240.0f, 360.0f, 0.0f, 90.0f, MAP_VAL_FREE);
}

/* Cập nhật tia ToF bằng thuật toán Bresenham Raycasting */
void Map_UpdateRay(float sensor_x_mm, float sensor_y_mm, float hit_x_mm, float hit_y_mm, bool hit_valid) {
    int c0 = (int)(sensor_x_mm / MAP_CELL_SIZE_MM);
    int r0 = (int)(sensor_y_mm / MAP_CELL_SIZE_MM);
    int c1 = (int)(hit_x_mm / MAP_CELL_SIZE_MM);
    int r1 = (int)(hit_y_mm / MAP_CELL_SIZE_MM);

    /* [FIX 3.6] Không clamp c0/r0/c1/r1 trước khi chạy Bresenham.
     * Việc clamp trước sẽ bẻ cong tia khi sensor nằm ngoài biên map (ví dụ Y=-45mm)
     * dẫn đến xóa nhầm ô và đánh dấu nhầm ô biên thành vật cản mới.
     * Thay vào đó: chỉ ghi vào g_map_grid khi chỉ số nằm trong biên hợp lệ. */

    int dc = abs(c1 - c0);
    int dr = abs(r1 - r0);
    int sc = (c0 < c1) ? 1 : -1;
    int sr = (r0 < r1) ? 1 : -1;
    int err = dc - dr;

    int cur_c = c0;
    int cur_r = r0;

    while (1) {
        /* Chỉ ghi vào bản đồ khi ô nằm trong biên hợp lệ */
        bool in_bounds = (cur_c >= 0 && cur_c < (int)MAP_COLS &&
                          cur_r >= 0 && cur_r < (int)MAP_ROWS);

        if (cur_c == c1 && cur_r == r1) {
            /* Điểm cuối cùng của tia */
            if (in_bounds) {
                if (hit_valid) {
                    /* Gặp cản: Nếu không phải tường/vật cản cứng cố định (255) thì gán là 10 (Mới phát hiện) */
                    if (g_map_grid[cur_r][cur_c] != MAP_VAL_OBSTACLE) {
                        g_map_grid[cur_r][cur_c] = MAP_VAL_NEW_OBSTACLE;
                    }
                } else {
                    /* Không gặp cản (quá tầm): gán là 0 (Không vật cản) */
                    if (g_map_grid[cur_r][cur_c] != MAP_VAL_OBSTACLE) {
                        g_map_grid[cur_r][cur_c] = MAP_VAL_FREE;
                    }
                }
            }
            break;
        }

        /* Các ô trên đường tia đi qua: Chắc chắn là không gian trống -> gán là 0 */
        if (in_bounds && g_map_grid[cur_r][cur_c] != MAP_VAL_OBSTACLE) {
            g_map_grid[cur_r][cur_c] = MAP_VAL_FREE;
        }

        int e2 = 2 * err;
        if (e2 > -dr) {
            err -= dr;
            cur_c += sc;
        }
        if (e2 < dc) {
            err += dc;
            cur_r += sr;
        }
    }
}

/* Cập nhật từ cảm biến ToF tích hợp với hệ tọa độ xe trong navigation.h */
void Map_UpdateFromTof(float robot_x_mm, float robot_y_mm, float robot_heading,
                       float tof_dist_mm, TofXshut_Sensor sensor_id, float max_range_mm) {
    float sensor_origin_x = 0.0f;
    float sensor_origin_y = 0.0f;
    float hit_x = 0.0f;
    float hit_y = 0.0f;

    bool hit_valid = (tof_dist_mm > 10.0f && tof_dist_mm <= max_range_mm);
    float ray_dist = hit_valid ? tof_dist_mm : max_range_mm;

    /* Tính tọa độ đầu cảm biến (dis = 0) và tọa độ điểm phản xạ tia */
    switch (sensor_id) {
        case TOF_SENSOR_LEFT:
            leftpose(0.0f, robot_x_mm, robot_y_mm, robot_heading, &sensor_origin_x, &sensor_origin_y);
            leftpose(ray_dist, robot_x_mm, robot_y_mm, robot_heading, &hit_x, &hit_y);
            break;

        case TOF_SENSOR_MID:
            midpose(0.0f, robot_x_mm, robot_y_mm, robot_heading, &sensor_origin_x, &sensor_origin_y);
            midpose(ray_dist, robot_x_mm, robot_y_mm, robot_heading, &hit_x, &hit_y);
            break;

        case TOF_SENSOR_RIGHT:
            rightpose(0.0f, robot_x_mm, robot_y_mm, robot_heading, &sensor_origin_x, &sensor_origin_y);
            rightpose(ray_dist, robot_x_mm, robot_y_mm, robot_heading, &hit_x, &hit_y);
            break;

        default:
            return;
    }

    /* Quét tia trên ma trận bản đồ */
    Map_UpdateRay(sensor_origin_x, sensor_origin_y, hit_x, hit_y, hit_valid);
}

/* Cập nhật từ điểm chạm (hit_x, hit_y) đã tính toán sẵn */
void Map_UpdateHitPoint(float robot_x_mm, float robot_y_mm, float robot_heading_deg,
                        float hit_x_mm, float hit_y_mm,
                        TofXshut_Sensor sensor_id, bool is_valid_hit) {
    float sensor_ox = 0.0f;
    float sensor_oy = 0.0f;

    switch (sensor_id) {
        case TOF_SENSOR_LEFT:
            leftpose(0.0f, robot_x_mm, robot_y_mm, robot_heading_deg, &sensor_ox, &sensor_oy);
            break;
        case TOF_SENSOR_MID:
            midpose(0.0f, robot_x_mm, robot_y_mm, robot_heading_deg, &sensor_ox, &sensor_oy);
            break;
        case TOF_SENSOR_RIGHT:
            rightpose(0.0f, robot_x_mm, robot_y_mm, robot_heading_deg, &sensor_ox, &sensor_oy);
            break;
        default:
            return;
    }

    Map_UpdateRay(sensor_ox, sensor_oy, hit_x_mm, hit_y_mm, is_valid_hit);
}

/* Cập nhật cả 3 cảm biến cùng lúc */
void Map_UpdateAllSensors(float robot_x_mm, float robot_y_mm, float robot_heading_deg,
                          float x_left, float y_left, bool left_valid,
                          float x_mid, float y_mid, bool mid_valid,
                          float x_right, float y_right, bool right_valid) {
    /* Chỉ cập nhật khi cảm biến có dữ liệu hợp lệ (tránh xóa nhầm ô khi sensor chưa có mẫu mới) */
    if (left_valid) {
        Map_UpdateHitPoint(robot_x_mm, robot_y_mm, robot_heading_deg, x_left, y_left, TOF_SENSOR_LEFT, true);
    }
    if (mid_valid) {
        Map_UpdateHitPoint(robot_x_mm, robot_y_mm, robot_heading_deg, x_mid, y_mid, TOF_SENSOR_MID, true);
    }
    if (right_valid) {
        Map_UpdateHitPoint(robot_x_mm, robot_y_mm, robot_heading_deg, x_right, y_right, TOF_SENSOR_RIGHT, true);
    }
}

/* Xác nhận nâng cấp ô từ 10 (mới phát hiện) lên 255 (vật cản cứng) */
void Map_ConfirmObstacle(uint8_t col, uint8_t row) {
    if (col < MAP_COLS && row < MAP_ROWS) {
        if (g_map_grid[row][col] == MAP_VAL_NEW_OBSTACLE) {
            g_map_grid[row][col] = MAP_VAL_OBSTACLE;
        }
    }
}

/* Trích xuất toàn bộ bản đồ 60x40 qua UART2 */
void Map_DumpToUart(void) {
    char buf[160];

    /* Đợi DMA TX hoàn tất nếu có gói telemetry đang gửi dở (tối đa 20ms) */
    uint32_t waitStart = HAL_GetTick();
    while (huart2.gState != HAL_UART_STATE_READY && (HAL_GetTick() - waitStart < 20U)) {
        __NOP();
    }

    /* 1. Đếm thống kê các ô */
    uint16_t count_free = 0;
    uint16_t count_unexplored = 0;
    uint16_t count_new = 0;
    uint16_t count_obstacle = 0;

    for (uint8_t r = 0; r < MAP_ROWS; r++) {
        for (uint8_t c = 0; c < MAP_COLS; c++) {
            uint8_t v = g_map_grid[r][c];
            if (v == MAP_VAL_FREE) count_free++;
            else if (v == MAP_VAL_UNEXPLORED) count_unexplored++;
            else if (v == MAP_VAL_NEW_OBSTACLE) count_new++;
            else if (v == MAP_VAL_OBSTACLE) count_obstacle++;
        }
    }

    /* 2. Header và thống kê */
    snprintf(buf, sizeof(buf), "\r\n[MAP] === BẮT ĐẦU TRÍCH XUẤT MAP STM32 (60x40, 3cm/ô) ===\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

    snprintf(buf, sizeof(buf), "[MAP] Thống kê: Trống(0)=%u | Chưa quét(1)=%u | Mới phát hiện(10)=%u | Vật cản(255)=%u\r\n",
             count_free, count_unexplored, count_new, count_obstacle);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

    /* 3. Liệt kê danh sách các ô mới phát hiện (10) */
    if (count_new > 0) {
        snprintf(buf, sizeof(buf), "[MAP] 📍 Danh sách %u ô cản mới phát hiện (10):\r\n", count_new);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

        for (uint8_t r = 0; r < MAP_ROWS; r++) {
            for (uint8_t c = 0; c < MAP_COLS; c++) {
                if (g_map_grid[r][c] == MAP_VAL_NEW_OBSTACLE) {
                    float x_mm, y_mm;
                    Map_GridToMm(c, r, &x_mm, &y_mm);
                    snprintf(buf, sizeof(buf), "[MAP-OBS] Cột %02u, Hàng %02u -> Tọa độ (%.0f, %.0f) mm\r\n",
                             c, r, x_mm, y_mm);
                    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 50);
                }
            }
        }
    }

    /* 4. Truyền 40 dòng ma trận (mỗi dòng 60 ký tự) */
    /* Dùng ký hiệu: . = Trống(0), ? = Chưa quét(1), ! = Mới phát hiện(10), # = Vật cản(255) */
    for (int8_t r = (int8_t)(MAP_ROWS - 1); r >= 0; r--) {
        char row_str[MAP_COLS + 1];
        for (uint8_t c = 0; c < MAP_COLS; c++) {
            uint8_t v = g_map_grid[r][c];
            if (v == MAP_VAL_FREE) row_str[c] = '.';
            else if (v == MAP_VAL_UNEXPLORED) row_str[c] = '?';
            else if (v == MAP_VAL_NEW_OBSTACLE) row_str[c] = '!';
            else row_str[c] = '#';
        }
        row_str[MAP_COLS] = '\0';

        snprintf(buf, sizeof(buf), "[MAP-ROW] %02d: %s\r\n", r, row_str);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 50);
    }

    /* 5. Footer */
    snprintf(buf, sizeof(buf), "[MAP] === KẾT THÚC TRÍCH XUẤT MAP STM32 ===\r\n\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
}
