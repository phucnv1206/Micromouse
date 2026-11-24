#include "astar.h"
#include "map_grid.h"
#include "odometry.h"
#include "pid_controller.h"
#include "led.h"
#include "navigation.h"
#include "config.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
 * THÔNG SỐ CẤU HÌNH THUẬT TOÁN A* VÀ NÉ CẢN
 * ========================================================================== */
#define HEAP_MAX_SIZE          512U
#define HEAP_KEY_INF           0xFFFFU
#define INFLATION_CELLS        1U      /* Nở cản 1 ô (30 mm) quanh vật cản */

#define OBSTACLE_DETECT_MIN_MM 40.0f   /* Cự ly phát hiện cản tối thiểu (mm) */
#define MID_LOOKAHEAD_MAX_MM   400.0f  /* Tầm nhìn ToF giữa quét dọc hành lang (mm) - quét trước 40 cm để xe gom mẫu */
#define SIDE_LOOKAHEAD_MAX_MM  250.0f  /* Tầm nhìn ToF 2 bên (mm) - tránh quét lệch vào tường sa bàn */
#define FINISH_ZONE_SAFE_Y_MM  180.0f  /* Khi xe ở X >= 1200 và Y < 180 mm: Khóa ToF tránh bắt nhầm vạch đích */
#define GOAL_FINISH_Y_MM       -100.0f /* Tọa độ Y đích đến hoàn thành (mm) */

#define REPLAN_COOLDOWN_MS     1000U   /* Thời gian tối thiểu giữa 2 lần tính lại đường (ms) */
#define START_COUNTDOWN_MS     1000U   /* Đếm ngược nhấp nháy đèn 1 giây trước khi chạy (ms) */

/* Cột bắt đầu và kết thúc của vạch đích an toàn (cách xa vạch biên 1200 mm và tường ngoài 1800 mm) */
#define FINISH_LINE_COL_MIN    42U     /* 42 * 30 mm = 1260 mm (Tâm ô 1275 mm, cách an toàn vạch biên vùng cấm 1200 mm) */
#define FINISH_LINE_COL_MAX    57U     /* 57 * 30 mm = 1710 mm (cách an toàn tường sa bàn phải 1800 mm) */

/* Thông số lọc cụm điểm (Spatial Clustering) và kiểm tra hành lang an toàn */
#define CLUSTER_MAX_HITS        16U
#define CLUSTER_MIN_CONFIRM     3U     /* Tối thiểu 3 điểm đỏ gần nhau mới xác nhận cản */
#define CLUSTER_DIST_THRESH_MM  75.0f  /* Bán kính gom cụm điểm đỏ (mm) */
#define CLUSTER_MAX_AGE_MS      350U   /* Thời gian sống tối đa của mẫu điểm (ms) */
#define OBSTACLE_DIAMETER_MM    100.0f /* Đường kính khoanh vùng cản 10 cm */
#define CORRIDOR_BLOCK_DIST_MM  110.0f /* Bán kính dải an toàn hành lang (mm) */
#define REPLAN_OBSTACLE_MAX_DIST_MM 100.0f /* Chỉ tìm đường mới khi cản bất ngờ cách xe dưới 10 cm (100 mm) */

/* ==========================================================================
 * LỘ TRÌNH TỐI ƯU CỐ ĐỊNH BAN ĐẦU QUA 5 MỐC
 * ========================================================================== */
static const AutoWaypoint s_universalWaypoints[] = {
    {  430.0f,  250.0f },  /* Mốc 0 (Điểm 2 trên sa bàn) */
    {  430.0f,  740.0f },  /* Mốc 1 (Điểm 3 trên sa bàn) */
    {  590.0f,  930.0f },  /* Mốc 2 (Điểm 4 trên sa bàn - Vòng qua đỉnh T9) */
    { 1250.0f,  675.0f },  /* Mốc 3 (Điểm 5 trên sa bàn - Đón góc cua gắt sát góc vùng cấm) */
    { 1265.0f, GOAL_FINISH_Y_MM } /* Mốc 4 (Điểm 6 trên sa bàn - Chạy thẳng tắp tim hành lang X=1265 mm qua đích) */
};
#define NUM_UNIVERSAL_WAYPOINTS (sizeof(s_universalWaypoints) / sizeof(s_universalWaypoints[0]))

/* ==========================================================================
 * BIẾN TOÀN CỤC NỘI BỘ
 * ========================================================================== */
static AutoState s_autoState = AUTO_STATE_IDLE;
static uint32_t s_startTick = 0U;
static uint32_t s_lastReplanTick = 0U;
static uint8_t  s_obstacleDetectCount = 0U;

static float s_lastDetectedObsX = 0.0f;
static float s_lastDetectedObsY = 0.0f;

/* Mảng lưu lộ trình điểm mốc đang thực thi */
static AutoWaypoint s_activeWaypoints[AUTO_MAX_WAYPOINTS];
static uint8_t s_numActiveWaypoints = 0U;
static uint8_t s_activeWaypointIdx = 0U;

/* Dữ liệu 3 cảm biến khoảng cách ToF */
static float s_leftDistanceMm = 0.0f;
static float s_midDistanceMm = 0.0f;
static float s_rightDistanceMm = 0.0f;
static bool s_leftValid = false;
static bool s_midValid = false;
static bool s_rightValid = false;

/* ==========================================================================
 * BỘ NHỚ VÀ CẤU TRÚC CHO THUẬT TOÁN A*
 * ========================================================================== */
static uint8_t  s_inflated[MAP_ROWS][MAP_COLS];
static uint16_t s_gScore[MAP_TOTAL_CELLS];
static uint16_t s_cameFrom[MAP_TOTAL_CELLS];
static uint8_t  s_closed[MAP_TOTAL_CELLS / 8U];

static uint16_t s_heapNodes[HEAP_MAX_SIZE];
static uint16_t s_heapKeys[HEAP_MAX_SIZE];
static uint16_t s_heapSize = 0U;

static uint16_t s_rawGridPath[256];
static uint16_t s_rawPathLen = 0U;

/* ==========================================================================
 * CHUYỂN ĐỔI TỌA ĐỘ AN TOÀN (CÓ CLAMP CHO BIÊN VÀ ĐÍCH ĐẾN ÂM)
 * ========================================================================== */
static bool Auto_MmToGrid(float x_mm, float y_mm, uint8_t *col, uint8_t *row) {
    if (x_mm < 0.0f) x_mm = 0.0f;
    if (x_mm >= MAP_WIDTH_MM) x_mm = MAP_WIDTH_MM - 1.0f;
    if (y_mm < 0.0f) y_mm = 0.0f;
    if (y_mm >= MAP_HEIGHT_MM) y_mm = MAP_HEIGHT_MM - 1.0f;

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

/* ==========================================================================
 * KIỂM TRA ĐIỂM CÓ PHẢI LÀ TRỤ TĨNH CỐ ĐỊNH HOẶC HỘP U ĐÃ BIẾT TRƯỚC
 * ========================================================================== */
static bool Auto_IsKnownStaticObstacle(float x_mm, float y_mm) {
    /* Sa bàn biên ngoài: không coi tường ngoài sa bàn là cản động */
    if (x_mm <= 60.0f || x_mm >= 1740.0f || y_mm <= -60.0f || y_mm >= 1140.0f) {
        return true;
    }

    /* Vùng cấm chữ U: [600, 1200] x [0, 600] mm */
    if (x_mm >= 560.0f && x_mm <= 1240.0f && y_mm >= 0.0f && y_mm <= 640.0f) {
        return true;
    }

    uint8_t c, r;
    if (Auto_MmToGrid(x_mm, y_mm, &c, &r)) {
        for (int8_t dr = -1; dr <= 1; dr++) {
            for (int8_t dc = -1; dc <= 1; dc++) {
                int nr = (int)r + dr;
                int nc = (int)c + dc;
                if (nr >= 0 && nr < (int)MAP_ROWS && nc >= 0 && nc < (int)MAP_COLS) {
                    if (g_map_grid[nr][nc] == MAP_VAL_OBSTACLE) {
                        return true; /* Trùng với 17 trụ tĩnh đã biết */
                    }
                }
            }
        }
    }
    return false;
}

/* ==========================================================================
 * XỬ LÝ HÀNG ĐỢI ƯU TIÊN MIN-HEAP & BITMASK
 * ========================================================================== */
static void Heap_Clear(void) {
    s_heapSize = 0U;
}

static void Heap_Push(uint16_t node, uint16_t key) {
    if (s_heapSize >= HEAP_MAX_SIZE) return;
    uint16_t i = s_heapSize++;
    while (i > 0U) {
        uint16_t p = (i - 1U) / 2U;
        if (s_heapKeys[p] <= key) break;
        s_heapNodes[i] = s_heapNodes[p];
        s_heapKeys[i]  = s_heapKeys[p];
        i = p;
    }
    s_heapNodes[i] = node;
    s_heapKeys[i]  = key;
}

static bool Heap_Pop(uint16_t *node, uint16_t *key) {
    if (s_heapSize == 0U) return false;
    *node = s_heapNodes[0];
    *key  = s_heapKeys[0];
    s_heapSize--;
    if (s_heapSize > 0U) {
        uint16_t lastNode = s_heapNodes[s_heapSize];
        uint16_t lastKey  = s_heapKeys[s_heapSize];
        uint16_t i = 0U;
        while ((2U * i + 1U) < s_heapSize) {
            uint16_t left = 2U * i + 1U;
            uint16_t right = left + 1U;
            uint16_t smallest = left;
            if (right < s_heapSize && s_heapKeys[right] < s_heapKeys[left]) {
                smallest = right;
            }
            if (s_heapKeys[smallest] >= lastKey) break;
            s_heapNodes[i] = s_heapNodes[smallest];
            s_heapKeys[i]  = s_heapKeys[smallest];
            i = smallest;
        }
        s_heapNodes[i] = lastNode;
        s_heapKeys[i]  = lastKey;
    }
    return true;
}

static inline void ClosedSet_Clear(void) {
    memset(s_closed, 0, sizeof(s_closed));
}

static inline void ClosedSet_Set(uint16_t node) {
    s_closed[node >> 3U] |= (uint8_t)(1U << (node & 7U));
}

static inline bool ClosedSet_Contains(uint16_t node) {
    return (s_closed[node >> 3U] & (uint8_t)(1U << (node & 7U))) != 0U;
}

/* ==========================================================================
 * NỞ VẬT CẢN (OBSTACLE INFLATION)
 * ========================================================================== */
static void Auto_InflateMap(void) {
    memset(s_inflated, 0, sizeof(s_inflated));

    for (uint8_t r = 0; r < MAP_ROWS; r++) {
        for (uint8_t c = 0; c < MAP_COLS; c++) {
            /* [CHỈ LẤY ĐIỂM ĐỎ XÁC NHẬN]: Chỉ nở vật cản cho điểm đỏ (MAP_VAL_OBSTACLE = 255),
             * không nở vật cản từ điểm vàng chưa xác nhận (MAP_VAL_NEW_OBSTACLE) để tránh nhận diện sai */
            if (g_map_grid[r][c] == MAP_VAL_OBSTACLE) {
                int r_min = (int)r - (int)INFLATION_CELLS;
                int r_max = (int)r + (int)INFLATION_CELLS;
                int c_min = (int)c - (int)INFLATION_CELLS;
                int c_max = (int)c + (int)INFLATION_CELLS;

                if (r_min < 0) r_min = 0;
                if (r_max >= (int)MAP_ROWS) r_max = (int)MAP_ROWS - 1;
                if (c_min < 0) c_min = 0;
                if (c_max >= (int)MAP_COLS) c_max = (int)MAP_COLS - 1;

                for (int ir = r_min; ir <= r_max; ir++) {
                    for (int ic = c_min; ic <= c_max; ic++) {
                        s_inflated[ir][ic] = MAP_VAL_OBSTACLE;
                    }
                }
            }
        }
    }
}

/* ==========================================================================
 * KIỂM TRA ĐƯỜNG TẦM NHÌN (LINE OF SIGHT - CHỐNG CẮT GÓC)
 * ========================================================================== */
static bool Auto_LineOfSight(int16_t c0, int16_t r0, int16_t c1, int16_t r1) {
    int16_t dc = (c1 > c0) ? (c1 - c0) : (c0 - c1);
    int16_t dr = (r1 > r0) ? (r1 - r0) : (r0 - r1);
    int16_t sc = (c0 < c1) ? 1 : -1;
    int16_t sr = (r0 < r1) ? 1 : -1;
    int16_t err = dc - dr;

    int16_t x = c0;
    int16_t y = r0;

    while (1) {
        if (x >= 0 && x < (int16_t)MAP_COLS && y >= 0 && y < (int16_t)MAP_ROWS) {
            if (s_inflated[y][x] == MAP_VAL_OBSTACLE) {
                return false;
            }
        }
        if (x == c1 && y == r1) {
            break;
        }

        int16_t e2 = 2 * err;
        int16_t prevX = x;
        int16_t prevY = y;

        if (e2 > -dr) {
            err -= dr;
            x += sc;
        }
        if (e2 < dc) {
            err += dc;
            y += sr;
        }

        /* [CHỐNG CẮT GÓC]: Nếu bước chéo, kiểm tra 2 ô kề bên để tuyệt đối không liếm vào mép trụ */
        if (x != prevX && y != prevY) {
            if (s_inflated[prevY][x] == MAP_VAL_OBSTACLE || s_inflated[y][prevX] == MAP_VAL_OBSTACLE) {
                return false;
            }
        }
    }
    return true;
}

/* ==========================================================================
 * THUẬT TOÁN A* TÌM ĐƯỜNG TỪ (startX, startY) ĐẾN (goalX, goalY)
 * HỖ TRỢ ĐÍCH ĐẾN ĐA ĐIỂM DẢI VẠCH ĐÍCH (X: 1200 - 1800 mm, Y = 0 mm)
 * ========================================================================== */
static bool Auto_PlanAStar(float startX, float startY, float goalX, float goalY,
                          bool isGoalFinishLine,
                          AutoWaypoint *outWp, uint8_t *outCount, uint8_t maxCount) {
    uint8_t startCol, startRow;
    uint8_t goalCol, goalRow;

    if (!Auto_MmToGrid(startX, startY, &startCol, &startRow) ||
        !Auto_MmToGrid(goalX, goalY, &goalCol, &goalRow)) {
        return false;
    }

    Auto_InflateMap();

    /* Xóa ô cản tại vị trí xuất phát để xe không bị kẹt vì mép nở cản */
    s_inflated[startRow][startCol] = MAP_VAL_FREE;
    for (int8_t dr = -1; dr <= 1; dr++) {
        for (int8_t dc = -1; dc <= 1; dc++) {
            int nr = (int)startRow + dr;
            int nc = (int)startCol + dc;
            if (nr >= 0 && nr < (int)MAP_ROWS && nc >= 0 && nc < (int)MAP_COLS) {
                if (g_map_grid[nr][nc] != MAP_VAL_OBSTACLE) {
                    s_inflated[nr][nc] = MAP_VAL_FREE;
                }
            }
        }
    }

    /* Mở đích đến: nếu là toàn bộ dải vạch đích (X: 1200-1800, Y=0), mở toàn bộ dải row 0 */
    if (isGoalFinishLine) {
        for (uint8_t c = FINISH_LINE_COL_MIN; c <= FINISH_LINE_COL_MAX; c++) {
            if (g_map_grid[0][c] != MAP_VAL_OBSTACLE) {
                s_inflated[0][c] = MAP_VAL_FREE;
            }
        }
    } else {
        s_inflated[goalRow][goalCol] = MAP_VAL_FREE;
    }

    uint16_t startNode = (uint16_t)startRow * MAP_COLS + startCol;
    uint16_t goalNode  = (uint16_t)goalRow * MAP_COLS + goalCol;

    for (uint16_t i = 0U; i < MAP_TOTAL_CELLS; i++) {
        s_gScore[i]   = HEAP_KEY_INF;
        s_cameFrom[i] = 0xFFFFU;
    }

    ClosedSet_Clear();
    Heap_Clear();

    s_gScore[startNode] = 0U;

    /* Tính heuristic h0 ban đầu */
    uint16_t h0;
    if (isGoalFinishLine) {
        int16_t dc = 0;
        if (startCol < FINISH_LINE_COL_MIN) dc = (int16_t)FINISH_LINE_COL_MIN - (int16_t)startCol;
        else if (startCol > FINISH_LINE_COL_MAX) dc = (int16_t)startCol - (int16_t)FINISH_LINE_COL_MAX;
        int16_t dr = (int16_t)startRow;
        h0 = (dc > dr) ? (14U * dr + 10U * (dc - dr)) : (14U * dc + 10U * (dr - dc));
    } else {
        int16_t dcol = (int16_t)goalCol - (int16_t)startCol;
        int16_t drow = (int16_t)goalRow - (int16_t)startRow;
        if (dcol < 0) dcol = -dcol;
        if (drow < 0) drow = -drow;
        h0 = (dcol > drow) ? (14U * drow + 10U * (dcol - drow)) : (14U * dcol + 10U * (drow - dcol));
    }
    Heap_Push(startNode, h0);

    static const int8_t dir[8][2] = {
        { 0,  1}, { 0, -1}, { 1,  0}, {-1,  0},
        { 1,  1}, { 1, -1}, {-1,  1}, {-1, -1}
    };
    static const uint16_t cost[8] = { 10U, 10U, 10U, 10U, 14U, 14U, 14U, 14U };

    bool found = false;

    while (s_heapSize > 0U) {
        uint16_t current, curKey;
        if (!Heap_Pop(&current, &curKey)) break;

        uint8_t c = (uint8_t)(current % MAP_COLS);
        uint8_t r = (uint8_t)(current / MAP_COLS);

        /* Kiểm tra điều kiện đích đến */
        if (isGoalFinishLine) {
            /* Cán vào bất kỳ ô nào thuộc vạch đích (hàng 0, cột 40 -> 59) là HOÀN THÀNH NGAY! */
            if (r == 0U && c >= FINISH_LINE_COL_MIN && c <= FINISH_LINE_COL_MAX) {
                goalNode = current;
                found = true;
                break;
            }
        } else {
            if (current == goalNode) {
                found = true;
                break;
            }
        }

        if (ClosedSet_Contains(current)) continue;
        ClosedSet_Set(current);

        for (uint8_t d = 0U; d < 8U; d++) {
            int8_t dc = dir[d][0];
            int8_t dr = dir[d][1];
            int16_t nc = (int16_t)c + dc;
            int16_t nr = (int16_t)r + dr;

            if (nc < 0 || nc >= (int16_t)MAP_COLS || nr < 0 || nr >= (int16_t)MAP_ROWS) continue;

            if (s_inflated[nr][nc] == MAP_VAL_OBSTACLE) continue;

            /* Tránh cắt góc chéo */
            if (dc != 0 && dr != 0) {
                if (s_inflated[r][nc] == MAP_VAL_OBSTACLE || s_inflated[nr][c] == MAP_VAL_OBSTACLE) {
                    continue;
                }
            }

            uint16_t neighbor = (uint16_t)nr * MAP_COLS + (uint16_t)nc;
            if (ClosedSet_Contains(neighbor)) continue;

            uint16_t tentative_g = s_gScore[current] + cost[d];
            if (tentative_g < s_gScore[neighbor]) {
                s_cameFrom[neighbor] = current;
                s_gScore[neighbor]   = tentative_g;

                /* Tính heuristic tới đích */
                uint16_t h;
                if (isGoalFinishLine) {
                    int16_t ndc = 0;
                    if (nc < (int16_t)FINISH_LINE_COL_MIN) ndc = (int16_t)FINISH_LINE_COL_MIN - nc;
                    else if (nc > (int16_t)FINISH_LINE_COL_MAX) ndc = nc - (int16_t)FINISH_LINE_COL_MAX;
                    int16_t ndr = nr;
                    h = (ndc > ndr) ? (14U * ndr + 10U * (ndc - ndr)) : (14U * ndc + 10U * (ndr - ndc));
                } else {
                    int16_t ndc = (int16_t)goalCol - nc;
                    int16_t ndr = (int16_t)goalRow - nr;
                    if (ndc < 0) ndc = -ndc;
                    if (ndr < 0) ndr = -ndr;
                    h = (ndc > ndr) ? (14U * ndr + 10U * (ndc - ndr)) : (14U * ndc + 10U * (ndr - ndc));
                }
                Heap_Push(neighbor, tentative_g + h);
            }
        }
    }

    if (!found) return false;

    /* Tái dựng chuỗi ô lưới */
    s_rawPathLen = 0U;
    uint16_t curr = goalNode;
    while (curr != 0xFFFFU && s_rawPathLen < 256U) {
        s_rawGridPath[s_rawPathLen++] = curr;
        if (curr == startNode) break;
        curr = s_cameFrom[curr];
    }

    /* Đảo ngược mảng để đi từ start -> goal */
    for (uint16_t i = 0U; i < s_rawPathLen / 2U; i++) {
        uint16_t tmp = s_rawGridPath[i];
        s_rawGridPath[i] = s_rawGridPath[s_rawPathLen - 1U - i];
        s_rawGridPath[s_rawPathLen - 1U - i] = tmp;
    }

    /* Làm mượt đường bằng Line-Of-Sight (Bresenham) */
    uint8_t count = 0U;
    uint16_t anchorIdx = 0U;

    while (anchorIdx < (s_rawPathLen - 1U)) {
        uint16_t furthestIdx = anchorIdx + 1U;
        for (uint16_t i = s_rawPathLen - 1U; i > anchorIdx; i--) {
            uint8_t ac = (uint8_t)(s_rawGridPath[anchorIdx] % MAP_COLS);
            uint8_t ar = (uint8_t)(s_rawGridPath[anchorIdx] / MAP_COLS);
            uint8_t tc = (uint8_t)(s_rawGridPath[i] % MAP_COLS);
            uint8_t tr = (uint8_t)(s_rawGridPath[i] / MAP_COLS);

            if (Auto_LineOfSight(ac, ar, tc, tr)) {
                furthestIdx = i;
                break;
            }
        }

        uint8_t fc = (uint8_t)(s_rawGridPath[furthestIdx] % MAP_COLS);
        uint8_t fr = (uint8_t)(s_rawGridPath[furthestIdx] / MAP_COLS);

        if (count < maxCount) {
            outWp[count].x_mm = ((float)fc + 0.5f) * MAP_CELL_SIZE_MM;
            outWp[count].y_mm = ((float)fr + 0.5f) * MAP_CELL_SIZE_MM;
            count++;
        }
        anchorIdx = furthestIdx;
    }

    /* Gán điểm đích cuối cùng */
    if (count > 0U) {
        if (isGoalFinishLine) {
            /* Tọa độ X đúng theo cột thoát thẳng, Y = GOAL_FINISH_Y_MM (-100 mm) để lao hẳn qua vạch đích */
            uint8_t finalC = (uint8_t)(goalNode % MAP_COLS);
            outWp[count - 1U].x_mm = ((float)finalC + 0.5f) * MAP_CELL_SIZE_MM;
            outWp[count - 1U].y_mm = GOAL_FINISH_Y_MM;
        } else {
            outWp[count - 1U].x_mm = goalX;
            outWp[count - 1U].y_mm = goalY;
        }
    }

    *outCount = count;
    return (count > 0U);
}

/* ==========================================================================
 * BỘ ĐỆM VÀ THUẬT TOÁN LỌC CỤM ĐIỂM (SPATIAL CLUSTERING) & TÍNH TOÁN HÀNH LANG
 * ========================================================================== */
typedef struct {
    float x_mm;
    float y_mm;
    uint32_t tick;
} HitSample;

static HitSample s_hitSamples[CLUSTER_MAX_HITS];
static uint8_t s_hitSampleCount = 0U;

static bool Auto_ProcessSensorHit(float hx, float hy, uint32_t now, float *outObsX, float *outObsY) {
    /* 1. Lọc bỏ các mẫu quá hạn thời gian */
    uint8_t writeIdx = 0U;
    for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
        if ((now - s_hitSamples[i].tick) <= CLUSTER_MAX_AGE_MS) {
            s_hitSamples[writeIdx++] = s_hitSamples[i];
        }
    }
    s_hitSampleCount = writeIdx;

    /* 2. Thêm mẫu mới */
    if (s_hitSampleCount < CLUSTER_MAX_HITS) {
        s_hitSamples[s_hitSampleCount].x_mm = hx;
        s_hitSamples[s_hitSampleCount].y_mm = hy;
        s_hitSamples[s_hitSampleCount].tick = now;
        s_hitSampleCount++;
    }

    /* 3. Đếm số lượng điểm đỏ lân cận điểm mới (Spatial Clustering) */
    uint8_t clusterCount = 0U;
    float sumX = 0.0f;
    float sumY = 0.0f;

    for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
        float dx = s_hitSamples[i].x_mm - hx;
        float dy = s_hitSamples[i].y_mm - hy;
        if ((dx * dx + dy * dy) <= (CLUSTER_DIST_THRESH_MM * CLUSTER_DIST_THRESH_MM)) {
            clusterCount++;
            sumX += s_hitSamples[i].x_mm;
            sumY += s_hitSamples[i].y_mm;
        }
    }

    if (clusterCount >= CLUSTER_MIN_CONFIRM) {
        *outObsX = sumX / (float)clusterCount;
        *outObsY = sumY / (float)clusterCount;
        s_hitSampleCount = 0U; /* Đã xác nhận cụm -> reset bộ đệm */
        return true;
    }

    return false;
}

/* Khoảng cách từ một điểm (px, py) tới đoạn thẳng nối (x0, y0) và (x1, y1) */
static float Auto_PointToSegmentDist(float px, float py, float x0, float y0, float x1, float y1) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float lenSq = dx * dx + dy * dy;
    if (lenSq < 1.0f) {
        float ex = px - x0;
        float ey = py - y0;
        return sqrtf(ex * ex + ey * ey);
    }
    float t = ((px - x0) * dx + (py - y0) * dy) / lenSq;
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;
    float projX = x0 + t * dx;
    float projY = y0 + t * dy;
    float rx = px - projX;
    float ry = py - projY;
    return sqrtf(rx * rx + ry * ry);
}

/* Thực thi tìm đường né toàn cục khi phát hiện hành lang bị cản */
static bool Auto_ExecuteReroute(float obsX, float obsY, const Odometry_State *pOdom) {
    /* 1. Dừng xe lập tức và kích hoạt chế độ không gian hẹp (quay tại chỗ để thoát cản) */
    Motion_Stop();
    Motion_SetTightSpace(true);
    s_autoState = AUTO_STATE_AVOIDING;

    /* 2. Khoanh vùng cản đường kính 10 cm (bán kính 50 mm) vào bản đồ số:
     * [CHỈ LẤY ĐIỂM ĐỎ XÁC NHẬN]: Đánh dấu MAP_VAL_OBSTACLE (255 - điểm đỏ đã xác nhận cụm ToF) */
    float halfDim = OBSTACLE_DIAMETER_MM * 0.5f;
    Map_MarkBox(obsX - halfDim, obsX + halfDim, obsY - halfDim, obsY + halfDim, MAP_VAL_OBSTACLE);

    /* 3. Lựa chọn mục tiêu A* thông minh */
    bool isGoalFinish = false;
    AutoWaypoint detourWp[AUTO_MAX_WAYPOINTS];
    uint8_t detourCount = 0U;
    bool detourFound = false;
    uint8_t targetIdx = s_activeWaypointIdx;

    /*
     * NẾU XE ĐÃ Ở KHU VỰC NỬA SAU SA BÀN (X > 500 VÀ Y > 700, TỨC TỪ ĐIỂM 4 TRỞ ĐI):
     * Mọi vật cản chắn đường đi về đích -> Tìm đường thoát thẳng về Vạch Đích (Y = -100)!
     * Thuật toán sẽ tự động vòng qua phía bắc T11/T13 rồi chạy dọc theo làn ngoài cùng bên phải.
     */
    if ((pOdom->x_mm >= 500.0f && pOdom->y_mm >= 700.0f) ||
        (s_activeWaypointIdx >= (s_numActiveWaypoints > 2U ? (s_numActiveWaypoints - 2U) : 0U))) {
        isGoalFinish = true;
        detourFound = Auto_PlanAStar(pOdom->x_mm, pOdom->y_mm, 1265.0f, GOAL_FINISH_Y_MM, true, detourWp, &detourCount, 16U);
        targetIdx = s_numActiveWaypoints;
    } else {
        /*
         * Ở NỬA ĐẦU SA BÀN: Tìm mốc lộ trình tiếp theo nằm sau vật cản (khoảng cách > 120 mm)
         * Để A* tìm hành lang trống (ví dụ luồn sang trái T3) rồi nhập lại lộ trình chuẩn.
         */
        while (targetIdx < s_numActiveWaypoints) {
            float tx = s_activeWaypoints[targetIdx].x_mm;
            float ty = s_activeWaypoints[targetIdx].y_mm;
            float distToObs = sqrtf((tx - obsX) * (tx - obsX) + (ty - obsY) * (ty - obsY));

            if (distToObs > 120.0f) {
                isGoalFinish = (targetIdx >= (s_numActiveWaypoints - 1U)) || (ty <= 0.0f && tx >= 1200.0f);
                if (Auto_PlanAStar(pOdom->x_mm, pOdom->y_mm, tx, ty, isGoalFinish, detourWp, &detourCount, 16U)) {
                    detourFound = true;
                    break;
                }
            }
            targetIdx++;
        }

        if (!detourFound) {
            isGoalFinish = true;
            detourFound = Auto_PlanAStar(pOdom->x_mm, pOdom->y_mm, 1265.0f, GOAL_FINISH_Y_MM, true, detourWp, &detourCount, 16U);
            targetIdx = s_numActiveWaypoints;
        }
    }

    if (detourFound && detourCount > 0U) {
        AutoWaypoint newRoute[AUTO_MAX_WAYPOINTS];
        uint8_t newCount = 0U;

        for (uint8_t i = 0U; i < detourCount && newCount < AUTO_MAX_WAYPOINTS; i++) {
            newRoute[newCount++] = detourWp[i];
        }
        if (!isGoalFinish) {
            for (uint8_t i = targetIdx + 1U; i < s_numActiveWaypoints && newCount < AUTO_MAX_WAYPOINTS; i++) {
                newRoute[newCount++] = s_activeWaypoints[i];
            }
        }

        memcpy(s_activeWaypoints, newRoute, sizeof(AutoWaypoint) * newCount);
        s_numActiveWaypoints = newCount;
        s_activeWaypointIdx = 0U;
        s_autoState = AUTO_STATE_RUNNING;
        s_lastDetectedObsX = 0.0f;
        s_lastDetectedObsY = 0.0f;
        s_hitSampleCount = 0U;

        bool isFinal = (s_numActiveWaypoints <= 1U);
        Motion_NavToPoint(s_activeWaypoints[0].x_mm, s_activeWaypoints[0].y_mm, isFinal);
        return true;
    } else {
        /* Không tìm được đường né khả thi: Tiếp tục lộ trình */
        s_autoState = AUTO_STATE_RUNNING;
        bool isFinal = (s_activeWaypointIdx >= (s_numActiveWaypoints > 1U ? (s_numActiveWaypoints - 1U) : 0U));
        Motion_NavToPoint(s_activeWaypoints[s_activeWaypointIdx].x_mm, s_activeWaypoints[s_activeWaypointIdx].y_mm, isFinal);
        return false;
    }
}

/* ==========================================================================
 * CÁC HÀM GIAO TIẾP ĐIỀU KHIỂN TỰ ĐỘNG
 * ========================================================================== */

void Auto_Init(void) {
    s_autoState = AUTO_STATE_IDLE;
    s_startTick = 0U;
    s_lastReplanTick = 0U;
    s_obstacleDetectCount = 0U;
    s_lastDetectedObsX = 0.0f;
    s_lastDetectedObsY = 0.0f;
    s_numActiveWaypoints = 0U;
    s_activeWaypointIdx = 0U;
    s_hitSampleCount = 0U;
}

void Auto_UpdateSensors(float leftMm, bool leftValid,
                        float midMm, bool midValid,
                        float rightMm, bool rightValid) {
    s_leftDistanceMm = leftMm;
    s_leftValid = leftValid;
    s_midDistanceMm = midMm;
    s_midValid = midValid;
    s_rightDistanceMm = rightMm;
    s_rightValid = rightValid;
}

void Auto_OnButton1(void) {
    if (s_autoState == AUTO_STATE_STARTING || s_autoState == AUTO_STATE_RUNNING || s_autoState == AUTO_STATE_AVOIDING) {
        return;
    }

    Auto_Stop();

    /* Nạp bản đồ sa bàn chuẩn (17 trụ tĩnh + vùng cấm U) */
    Map_Init(0, 0);

    /* Nạp lộ trình 5 mốc tối ưu ban đầu */
    s_numActiveWaypoints = NUM_UNIVERSAL_WAYPOINTS;
    memcpy(s_activeWaypoints, s_universalWaypoints, sizeof(s_universalWaypoints));
    s_activeWaypointIdx = 0U;
    s_obstacleDetectCount = 0U;
    s_lastDetectedObsX = 0.0f;
    s_lastDetectedObsY = 0.0f;
    s_hitSampleCount = 0U;
    s_lastReplanTick = 0U;

    /* Bắt đầu đếm ngược 1 giây: LED 2 nháy nhanh (100ms ON / 100ms OFF) */
    s_startTick = HAL_GetTick();
    s_autoState = AUTO_STATE_STARTING;

    Led_SetPattern(1U, 0U, 0U, 0U, 0U);
    Led_SetPattern(3U, 0U, 0U, 0U, 0U);
    Led_SetPattern(2U, 1U, 100U, 100U, 100U);
}

void Auto_OnButton2(void) {
    /* Dừng khẩn cấp / Hủy bỏ và đặt lại ban đầu */
    Auto_Stop();
    Odometry_Reset();

    Led_SetPattern(1U, 0U, 0U, 0U, 0U);
    Led_SetPattern(2U, 0U, 0U, 0U, 0U);
    Led_SetPattern(3U, 0U, 0U, 0U, 0U);
}

void Auto_Update(float dt_s) {
    (void)dt_s;

    uint32_t now = HAL_GetTick();

    /* ======================================================================
     * 1. GIAI ĐOẠN ĐẾM NGƯỢC 1 GIÂY (AUTO_STATE_STARTING)
     * ====================================================================== */
    if (s_autoState == AUTO_STATE_STARTING) {
        if ((now - s_startTick) >= START_COUNTDOWN_MS) {
            /* Hết 1 giây:
             * - LED 2 sáng liên tục
             * - Reset Odometry chuẩn xác
             * - CÙNG LÚC XE CHẠY NGAY LẬP TỨC!
             */
            Led_SetPattern(2U, 1U, 1000U, 0U, 0U); /* Sáng liên tục */

            Odometry_Reset();

            s_activeWaypointIdx = 0U;
            s_autoState = AUTO_STATE_RUNNING;

            bool isFinal = (s_numActiveWaypoints <= 1U);
            Motion_NavToPoint(s_activeWaypoints[0].x_mm, s_activeWaypoints[0].y_mm, isFinal);
        }
        return;
    }

    if (s_autoState != AUTO_STATE_RUNNING) {
        return;
    }

    Odometry_State odom;
    if (Odometry_GetState(&odom) != HAL_OK) {
        return;
    }

    /* ======================================================================
     * 2. KIỂM TRA ĐÃ CẮT QUA VẠCH ĐÍCH (X >= 1200 mm, Y <= -80 mm)
     * ====================================================================== */
    /* Xe chạy qua vạch đích Y=0, tới vùng Y <= -80 mm (hướng tới đích Y=-100) thì dừng xe báo hoàn thành */
    if (odom.x_mm >= 1200.0f && odom.y_mm <= -80.0f &&
        s_activeWaypointIdx >= (s_numActiveWaypoints > 1U ? (s_numActiveWaypoints - 1U) : 0U)) {
        Motion_Stop();
        s_autoState = AUTO_STATE_COMPLETED;
        Led_SetPattern(2U, 1U, 1000U, 0U, 0U); /* LED 2 giữ sáng liên tục */
        return;
    }

    /* ======================================================================
     * 3. PHÁT HIỆN CẢN TỪ XA, LỌC CỤM ĐIỂM (SPATIAL CLUSTERING) & KIỂM TRA HÀNH LANG
     * ====================================================================== */
    /*
     * NGUYÊN LÝ HOẠT ĐỘNG:
     * - Khóa ToF duy nhất tại vùng chạy đà cán đích: (X >= 1200 mm và Y <= FINISH_ZONE_SAFE_Y_MM).
     * - Tại mọi khu vực khác (bao gồm cả ĐOẠN ĐẦU Y >= -60 mm, xuất phát tại Y = -45 mm):
     *   Hệ thống quét cản HOẠT ĐỘNG 100%.
     * - Quét trước bằng 3 cảm biến ToF với tầm nhìn xa (Mid nhìn xa tới 600 mm, 2 bên tới 350 mm).
     * - Lọc bỏ 17 trụ tĩnh, tường ngoài và vùng cấm U bằng Auto_IsKnownStaticObstacle().
     * - Gom cụm điểm đỏ (Spatial Clustering): Khi có ít nhất 3 điểm đỏ lân cận trong bán kính 75 mm
     *   -> Xác nhận là khối cản thực tế, khoanh vùng cản đường kính 10 cm (100x100 mm) vào bản đồ số.
     * - Kiểm tra thông tuyến hành lang di chuyển: Nếu cản nằm chắn ngang phân đoạn hành lang hiện tại
     *   hoặc phân đoạn hành lang sắp đi vào (khoảng cách tới trục hành lang <= 110 mm):
     *   -> KHÔNG BAO GIỜ CHUI VÀO HÀNH LANG BỊ PHONG TỎA!
     *   -> Dừng xe ngay tại cửa ngõ/ngã ba và kích hoạt A* toàn cục tìm hành lang thoáng mới!
     */
    bool inFinishZone = (odom.x_mm >= 1200.0f && odom.y_mm <= FINISH_ZONE_SAFE_Y_MM);

    if (!inFinishZone && odom.y_mm >= -60.0f && odom.y_mm <= 1150.0f &&
        s_activeWaypointIdx < s_numActiveWaypoints) {

        float headingRad = odom.heading_deg * (3.141592653589793f / 180.0f);
        float cosHeading = cosf(headingRad);
        float sinHeading = sinf(headingRad);

        bool hasHit = false;
        float hitX = 0.0f;
        float hitY = 0.0f;

        /* 1. Kiểm tra cảm biến giữa (ToF Mid - VL53L1X): Tầm nhìn xa tới 600 mm dọc hành lang */
        if (s_midValid && s_midDistanceMm >= OBSTACLE_DETECT_MIN_MM && s_midDistanceMm <= MID_LOOKAHEAD_MAX_MM) {
            float mx, my;
            midpose(s_midDistanceMm, odom.x_mm, odom.y_mm, odom.heading_deg, &mx, &my);
            if (!Auto_IsKnownStaticObstacle(mx, my)) {
                hitX = mx;
                hitY = my;
                hasHit = true;
            }
        }

        /* 2. Kiểm tra cảm biến trái (ToF Left - VL53L0X): Tầm nhìn tới 350 mm, góc lệch +45 độ */
        if (!hasHit && s_leftValid && s_leftDistanceMm >= OBSTACLE_DETECT_MIN_MM && s_leftDistanceMm <= SIDE_LOOKAHEAD_MAX_MM) {
            float lx, ly;
            leftpose(s_leftDistanceMm, odom.x_mm, odom.y_mm, odom.heading_deg, &lx, &ly);
            if (!Auto_IsKnownStaticObstacle(lx, ly)) {
                float fwd = (lx - odom.x_mm) * cosHeading + (ly - odom.y_mm) * sinHeading;
                if (fwd > 20.0f) {
                    hitX = lx;
                    hitY = ly;
                    hasHit = true;
                }
            }
        }

        /* 3. Kiểm tra cảm biến phải (ToF Right - VL53L0X): Tầm nhìn tới 350 mm, góc lệch -45 độ */
        if (!hasHit && s_rightValid && s_rightDistanceMm >= OBSTACLE_DETECT_MIN_MM && s_rightDistanceMm <= SIDE_LOOKAHEAD_MAX_MM) {
            float rx, ry;
            rightpose(s_rightDistanceMm, odom.x_mm, odom.y_mm, odom.heading_deg, &rx, &ry);
            if (!Auto_IsKnownStaticObstacle(rx, ry)) {
                float fwd = (rx - odom.x_mm) * cosHeading + (ry - odom.y_mm) * sinHeading;
                if (fwd > 20.0f) {
                    hitX = rx;
                    hitY = ry;
                    hasHit = true;
                }
            }
        }

        /* 4. Đưa điểm chạm vào bộ lọc cụm điểm (Spatial Clustering) */
        if (hasHit) {
            float confirmedObsX = 0.0f;
            float confirmedObsY = 0.0f;
            if (Auto_ProcessSensorHit(hitX, hitY, now, &confirmedObsX, &confirmedObsY)) {
                s_lastDetectedObsX = confirmedObsX;
                s_lastDetectedObsY = confirmedObsY;

                /* Tính khoảng cách từ xe tới vật cản đã xác nhận */
                float dxObs = confirmedObsX - odom.x_mm;
                float dyObs = confirmedObsY - odom.y_mm;
                float distCarToObs = sqrtf(dxObs * dxObs + dyObs * dyObs);
                float distFrontToObs = distCarToObs - 34.0f; /* Khoảng cách từ đầu xe / ToF sensor */
                if (distFrontToObs < 0.0f) distFrontToObs = 0.0f;
                float fwdDist = dxObs * cosHeading + dyObs * sinHeading;

                /* Kiểm tra xem cản này có nằm trong hành lang di chuyển hay không */
                bool corridorBlocked = false;

                /* Đoạn 1: Từ xe tới mốc mục tiêu hiện tại */
                float distCur = Auto_PointToSegmentDist(confirmedObsX, confirmedObsY,
                                                        odom.x_mm, odom.y_mm,
                                                        s_activeWaypoints[s_activeWaypointIdx].x_mm,
                                                        s_activeWaypoints[s_activeWaypointIdx].y_mm);
                if (distCur <= CORRIDOR_BLOCK_DIST_MM) {
                    corridorBlocked = true;
                }

                /* Đoạn 2: Phân đoạn hành lang kế tiếp ngay phía trước */
                if (!corridorBlocked && (s_activeWaypointIdx + 1U) < s_numActiveWaypoints) {
                    float distNext = Auto_PointToSegmentDist(confirmedObsX, confirmedObsY,
                                                             s_activeWaypoints[s_activeWaypointIdx].x_mm,
                                                             s_activeWaypoints[s_activeWaypointIdx].y_mm,
                                                             s_activeWaypoints[s_activeWaypointIdx + 1U].x_mm,
                                                             s_activeWaypoints[s_activeWaypointIdx + 1U].y_mm);
                    if (distNext <= CORRIDOR_BLOCK_DIST_MM) {
                        /* Chỉ kích hoạt né sớm nếu xe đã tiến gần đến cửa ngõ rẽ (<= 180 mm) */
                        float dx = s_activeWaypoints[s_activeWaypointIdx].x_mm - odom.x_mm;
                        float dy = s_activeWaypoints[s_activeWaypointIdx].y_mm - odom.y_mm;
                        if ((dx * dx + dy * dy) <= (180.0f * 180.0f)) {
                            corridorBlocked = true;
                        }
                    }
                }

                /*
                 * [YÊU CẦU: VẬT CẢN BẤT NGỜ CÁCH XE DƯỚI 10 CM (100 MM) THÌ MỚI TÌM ĐƯỜNG MỚI]:
                 * - Loại bỏ hoàn toàn nhiễu từ xa (phản xạ xa, mép trụ tĩnh, tia quét lệch).
                 * - Tăng thời gian để xe tiến tới gần và quét liên tục chùm tia ToF chính xác.
                 * - Chỉ khi khoảng cách từ đầu xe tới vật cản <= 100 mm (< 10 cm) VÀ vật cản nằm phía trước:
                 *   mới dừng xe và gọi Auto_ExecuteReroute để tìm đường mới!
                 */
                bool isUnder10cm = (distFrontToObs <= REPLAN_OBSTACLE_MAX_DIST_MM) && (fwdDist > 0.0f);

                /* Nếu hành lang bị phong tỏa VÀ cản cách xe dưới 10cm VÀ đã hết cooldown -> Né ngay! */
                if (corridorBlocked && isUnder10cm && (now - s_lastReplanTick) >= REPLAN_COOLDOWN_MS) {
                    s_lastReplanTick = now;
                    Auto_ExecuteReroute(confirmedObsX, confirmedObsY, &odom);
                    return;
                }
            }
        }
    }

    /* ======================================================================
     * 4. CHUYỂN TIẾP MỐC MƯỢT CUNG TRÒN LIÊN TỤC (WAYPOINT BLENDING)
     * ====================================================================== */
    /*
     * NGUYÊN LÝ HOẠT ĐỘNG:
     * - Đối với các mốc trung gian (trước mốc vạch đích cuối cùng):
     *   Xe KHÔNG BAO GIỜ dừng lại ở mốc để xoay tại chỗ!
     * - Khi xe tiến vào bán kính chuyển tiếp fillet (blendRadius <= 45 mm)
     *   hoặc đã chạy cắt qua mặt phẳng mốc (approachProj <= 0):
     *   -> Lập tức chuyển sang mốc tiếp theo và gửi lệnh CMD_NAV_TO_POINT ngay khi xe đang chạy!
     * - pid_controller sẽ chủ động giảm tốc và phanh bánh trong để bo góc fillet gắt và gọn gàng,
     *   không bị văng rộng tạo thành hình vòng cung quá đà!
     */
    if (s_activeWaypointIdx < (s_numActiveWaypoints > 1U ? (s_numActiveWaypoints - 1U) : 0U)) {
        float curWpX = s_activeWaypoints[s_activeWaypointIdx].x_mm;
        float curWpY = s_activeWaypoints[s_activeWaypointIdx].y_mm;
        float dx = curWpX - odom.x_mm;
        float dy = curWpY - odom.y_mm;
        float distToCurWp = sqrtf(dx * dx + dy * dy);

        /* Điểm mốc trước đó (để tính vector định hướng tiếp cận và hình chiếu) */
        float prevWpX = (s_activeWaypointIdx == 0U) ? ODOMETRY_INITIAL_X_MM : s_activeWaypoints[s_activeWaypointIdx - 1U].x_mm;
        float prevWpY = (s_activeWaypointIdx == 0U) ? ODOMETRY_INITIAL_Y_MM : s_activeWaypoints[s_activeWaypointIdx - 1U].y_mm;
        float segDx = curWpX - prevWpX;
        float segDy = curWpY - prevWpY;
        float segLen = sqrtf(segDx * segDx + segDy * segDy);

        /*
         * [CHUYỂN TIẾP MỐC CUA GẮT FILLET - XE ĐI QUA MỐC MỘT TÝ]:
         * - Không rẽ non / cắt góc sớm trước điểm (loại bỏ blendRadius lớn làm góc cua phình rộng thành vòng cung).
         * - Xe chạy thẳng tiến sâu tới mốc và vượt qua mốc một khoảng ngắn (pastDistMm >= NAV_WAYPOINT_PASS_DIST_MM)
         *   rồi mới chuyển mốc để vào cua fillet gắt tại đúng vị trí đỉnh góc cua!
         */
        float approachProj = segDx * dx + segDy * dy;
        float pastDistMm = (segLen > 1.0f) ? (-approachProj / segLen) : 0.0f;

        bool shouldSwitch = false;
        if (segLen > 10.0f) {
            /* Chuyển mốc khi xe đã tới sát mốc (<= 28mm) hoặc đã đi qua mặt phẳng mốc (pastDistMm >= 0) */
            if (pastDistMm >= NAV_WAYPOINT_PASS_DIST_MM ||
                (distToCurWp <= NAV_CORNER_BLEND_RADIUS_MM && pastDistMm >= -5.0f) ||
                (distToCurWp <= 18.0f)) {
                shouldSwitch = true;
            }
        } else {
            if (distToCurWp <= NAV_CORNER_BLEND_RADIUS_MM) {
                shouldSwitch = true;
            }
        }

        if (shouldSwitch) {
            s_activeWaypointIdx++;

            /* KIỂM TRA AN TOÀN HÀNH LANG KẾ TIẾP TRƯỚC KHI VÀO */
            bool nextSegmentBlocked = false;
            if (s_lastDetectedObsX != 0.0f || s_lastDetectedObsY != 0.0f) {
                float d = Auto_PointToSegmentDist(s_lastDetectedObsX, s_lastDetectedObsY,
                                                  odom.x_mm, odom.y_mm,
                                                  s_activeWaypoints[s_activeWaypointIdx].x_mm,
                                                  s_activeWaypoints[s_activeWaypointIdx].y_mm);
                if (d <= CORRIDOR_BLOCK_DIST_MM) {
                    float dxObs = s_lastDetectedObsX - odom.x_mm;
                    float dyObs = s_lastDetectedObsY - odom.y_mm;
                    float dToCar = sqrtf(dxObs * dxObs + dyObs * dyObs) - 34.0f;
                    if (dToCar < 0.0f) dToCar = 0.0f;
                    if (dToCar <= REPLAN_OBSTACLE_MAX_DIST_MM) {
                        nextSegmentBlocked = true;
                    }
                }
            }

            if (nextSegmentBlocked) {
                s_lastReplanTick = now;
                Auto_ExecuteReroute(s_lastDetectedObsX, s_lastDetectedObsY, &odom);
                return;
            }

            /* Gửi lệnh chạy tới mốc tiếp theo ngay khi xe đang di chuyển để ôm cua mượt mà */
            bool isFinal = (s_activeWaypointIdx >= (s_numActiveWaypoints > 1U ? (s_numActiveWaypoints - 1U) : 0U));
            Motion_NavToPoint(s_activeWaypoints[s_activeWaypointIdx].x_mm,
                              s_activeWaypoints[s_activeWaypointIdx].y_mm,
                              isFinal);
            return;
        }
    }

    /* ======================================================================
     * 5. KIỂM TRA HOÀN THÀNH MỐC TỌA ĐỘ (DỰ PHÒNG HOẶC MỐC CUỐI CÙNG)
     * ====================================================================== */
    int16_t statusWord = Motion_GetFeedbackStatusWord();
    uint8_t activeCmd = (uint8_t)((statusWord >> 8U) & 0xFFU);
    uint8_t cmdStatus = (uint8_t)(statusWord & 0xFFU);

    if (activeCmd == CMD_NAV_TO_POINT && cmdStatus == STATUS_COMPLETED) {
        s_activeWaypointIdx++;

        if (s_activeWaypointIdx < s_numActiveWaypoints) {
            /*
             * KIỂM TRA CỬA NGÕ TRƯỚC KHI VÀO HÀNH LANG MỚI:
             * Nếu cản đã phát hiện trước đó nằm chắn ngang phân đoạn sắp chạy vào:
             * -> QUAY XE / CHUYỂN LÀN NGAY TẠI ĐÂY, KHÔNG ĐÂM VÀO NGÕ CỤT!
             */
            bool nextSegmentBlocked = false;
            if (s_lastDetectedObsX != 0.0f || s_lastDetectedObsY != 0.0f) {
                float d = Auto_PointToSegmentDist(s_lastDetectedObsX, s_lastDetectedObsY,
                                                  odom.x_mm, odom.y_mm,
                                                  s_activeWaypoints[s_activeWaypointIdx].x_mm,
                                                  s_activeWaypoints[s_activeWaypointIdx].y_mm);
                if (d <= CORRIDOR_BLOCK_DIST_MM) {
                    float dxObs = s_lastDetectedObsX - odom.x_mm;
                    float dyObs = s_lastDetectedObsY - odom.y_mm;
                    float dToCar = sqrtf(dxObs * dxObs + dyObs * dyObs) - 34.0f;
                    if (dToCar < 0.0f) dToCar = 0.0f;
                    if (dToCar <= REPLAN_OBSTACLE_MAX_DIST_MM) {
                        nextSegmentBlocked = true;
                    }
                }
            }

            if (nextSegmentBlocked) {
                s_lastReplanTick = now;
                Auto_ExecuteReroute(s_lastDetectedObsX, s_lastDetectedObsY, &odom);
                return;
            }

            bool isFinal = (s_activeWaypointIdx >= (s_numActiveWaypoints > 1U ? (s_numActiveWaypoints - 1U) : 0U));
            Motion_NavToPoint(s_activeWaypoints[s_activeWaypointIdx].x_mm,
                              s_activeWaypoints[s_activeWaypointIdx].y_mm,
                              isFinal);
        } else {
            /* Đã hoàn thành tất cả các mốc lộ trình */
            Motion_Stop();
            s_autoState = AUTO_STATE_COMPLETED;
            Led_SetPattern(2U, 1U, 1000U, 0U, 0U); /* LED 2 giữ sáng liên tục */
        }
    } else if (activeCmd == CMD_NAV_TO_POINT && cmdStatus == STATUS_ERROR) {
        /* [BẢO VỆ VẠCH ĐÍCH]: Nếu xe đã ở sát hoặc qua vạch đích, hoàn thành an toàn */
        if (odom.x_mm >= 1200.0f && odom.y_mm <= 60.0f) {
            Motion_Stop();
            s_autoState = AUTO_STATE_COMPLETED;
            Led_SetPattern(2U, 1U, 1000U, 0U, 0U);
            return;
        }

        /* Lỗi chấp hành lệnh giữa đường: dừng an toàn báo lỗi */
        Motion_Stop();
        s_autoState = AUTO_STATE_ERROR;
        Led_SetPattern(1U, 1U, 80U, 80U, 80U);
    }
}

AutoState Auto_GetState(void) {
    return s_autoState;
}

bool Auto_IsMappingActive(void) {
    return false;
}

void Auto_Stop(void) {
    s_autoState = AUTO_STATE_IDLE;
    s_activeWaypointIdx = 0U;
    s_numActiveWaypoints = 0U;
    s_obstacleDetectCount = 0U;
    s_lastDetectedObsX = 0.0f;
    s_lastDetectedObsY = 0.0f;
    s_hitSampleCount = 0U;
    Motion_Stop();
}
