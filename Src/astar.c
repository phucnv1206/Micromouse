#include "astar.h"
#include "map_grid.h"
#include "odometry.h"
#include "pid_controller.h"
#include "motor_pwm.h"
#include "led.h"
#include "navigation.h"
#include "config.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
 * HẰNG SỐ CẤU HÌNH A*, KHOẢNG HỞ AN TOÀN VÀ PHÁT HIỆN BIÊN TƯỜNG BẤT NGỜ 3 TOF
 * (Chỉ đánh dấu biên tường lên g_map_grid, KHÔNG tạo khối hộp, chia lát cắt A*)
 * ========================================================================== */
#define HEAP_MAX_SIZE               (MAP_TOTAL_CELLS / 2U) /* 1200 phần tử cho nửa bản đồ */
#define HEAP_KEY_INF                0xFFFFU

#define OBSTACLE_DETECT_MIN_MM      25.0f
#define MID_DETECT_MAX_MM           245.0f  /* Tầm bắt tường bất ngờ tin cậy cho ToF giữa (VL53L1X ROI 8x8) */
#define SIDE_DETECT_MAX_MM          215.0f  /* Tầm bắt tường bất ngờ tin cậy cho ToF trái/phải (VL53L0X) */
#define FINISH_ZONE_SAFE_Y_MM       180.0f
#define TOF_ACTIVE_CAR_X_MIN_MM     900.0f
#define DETECT_ZONE_X_MIN_MM        1135.0f /* Chỉ ghi nhận tường lạ thuộc nửa phải sa bàn (bù trôi odom) */
#define GOAL_FINISH_Y_MM            -100.0f

#define START_COUNTDOWN_MS          1000U

#define FINISH_LINE_COL_MIN         42U     /* 1275 mm (cách vùng cấm chữ U 75 mm) */
#define FINISH_LINE_COL_MAX         55U     /* 1665 mm (cách biên phải 135 mm & tránh T17) */

#define HARD_OBS_CLEAR_MM           54.0f   /* < 54 mm quanh cản tĩnh / ô biên tường -> khóa cứng 255 */
#define HARD_WALL_CLEAR_MM          70.0f   /* < 70 mm sát tường ngoài -> khóa cứng 255 */
#define LOS_SAFE_CLEAR_MM           66.0f   /* Khoảng hở tối thiểu khi gộp đoạn thẳng LOS */
#define ARC_SAFE_CLEAR_MM           54.0f   /* Khoảng hở tối thiểu dọc cung tròn ICR */

#define CLUSTER_MAX_HITS            24U
#define CLUSTER_VERY_NEAR_DIST_MM   95.0f
#define CLUSTER_NEAR_DIST_MM        155.0f
#define CLUSTER_MIN_CONFIRM_VNEAR   2U      /* <= 95 mm: phản ứng hãm tức thì trước mép tường sát mũi xe */
#define CLUSTER_MIN_CONFIRM_NEAR    3U      /* <= 155 mm: 3 mẫu hội tụ để bắt kịp mép tường gần */
#define CLUSTER_MIN_CONFIRM_FAR     4U      /* > 155 mm: 4 mẫu hội tụ chống nhiễu hoàn toàn */
#define CLUSTER_DIST_THRESH_MM      45.0f   /* Bán kính hội tụ 45 mm */
#define CLUSTER_MAX_AGE_MS          350U

#define MAX_DYN_WALL_PTS            96U     /* Tối đa 96 điểm biên tường bất ngờ lưu trong lượt chạy */
#define WALL_PT_MERGE_DIST_MM       15.0f   /* Gộp các điểm biên trùng lặp < 15 mm */
#define WALL_PT_HALF_SIZE_MM        15.0f   /* Bán kính an toàn điểm biên tường (54+15=69 mm > chéo thân xe 58.5 mm) */
#define WALL_EDGE_TANGENT_HALF_MM   20.0f   /* Trải mỏng ±20 mm dọc tiếp tuyến mặt tường (0 mm chiều sâu) */
#define ARC_DYN_WALL_EXTRA_MM       18.0f   /* Không cắt cua ICR sát mép tường lạ chưa quét hết góc */
#define OBS_BLOCK_CLEAR_MM          48.0f   /* Ngưỡng kiểm tra biên tường mới chắn hành lang lộ trình */
#define KNOWN_WALL_CLOSE_CLEAR_MM   42.0f   /* Ngưỡng bảo vệ sát bánh xe cho điểm biên đã biết */
#define ROUTE_BLOCK_LOOKAHEAD_MM    280.0f  /* Chỉ xét chắn đường trong tầm 280 mm phía trước dọc lộ trình */

#define AVOID_SETTLE_MAX_MS         180U    /* Thời gian chờ hãm quán tính tối đa trước khi chạy A* */
#define AVOID_SETTLE_LIN_MM_S       40.0f   /* Vận tốc tuyến tính đủ chậm để chốt pose lập đường A* */
#define AVOID_SETTLE_ANG_RAD_S      0.35f   /* Vận tốc góc đủ chậm (~20 dps) trước khi lập đường A* */
#define TOF_MAX_SCAN_ANG_RAD_S      0.55f   /* Ngưỡng tốc độ góc tối đa cho phép gom mẫu ToF chống nhòe */

#define INFLATE_ROWS_PER_SLICE      8U      /* 8 hàng / lát cắt -> 5 lát cắt cho 40 hàng */
#define ASTAR_NODES_PER_SLICE       80U     /* Tối đa 80 nút heap / lát cắt (< 300 us) */
#define MAX_PLAN_CANDIDATES         4U      /* Tối đa 3 mốc nối tiếp + 1 đường thẳng về vạch đích */

#define DEG_TO_RAD_F                (3.141592653589793f / 180.0f)
#define RAD_TO_DEG_F                (180.0f / 3.141592653589793f)

/* ==========================================================================
 * LỘ TRÌNH MẶC ĐỊNH & BIẾN TRẠNG THÁI NỘI BỘ
 * ========================================================================== */
static const AutoWaypoint s_universalWaypoints[] = {
    {  425.0f,  250.0f },        /* Mốc 0: giữa khe T3 - T4 */
    {  430.0f,  740.0f },        /* Mốc 1: cửa vào nhánh trái */
    {  590.0f,  930.0f },        /* Mốc 2: đỉnh vòng qua T9 */
    { 1285.0f,  680.0f },        /* Mốc 3: cửa hành lang đích */
    { 1285.0f, GOAL_FINISH_Y_MM } /* Mốc 4: cán qua vạch đích */
};
#define NUM_UNIVERSAL_WAYPOINTS (sizeof(s_universalWaypoints) / sizeof(s_universalWaypoints[0]))

typedef enum {
    CORNER_NONE = 0,
    CORNER_ARC,
    CORNER_SPIN
} AutoCornerType;

typedef enum {
    AVOID_PHASE_NONE = 0,
    AVOID_PHASE_SETTLE,
    AVOID_PHASE_BACKOFF,
    AVOID_PHASE_POST_BACKOFF_SETTLE,
    AVOID_PHASE_INFLATE,
    AVOID_PHASE_SEARCH,
    AVOID_PHASE_APPLY
} AvoidPhase;

typedef struct {
    AutoCornerType type;
    AutoWaypoint wp;
    AutoWaypoint Tin;
    AutoWaypoint Tout;
    float R;
    int8_t dir;
    float dth_deg;
    float th1_deg;
    float th2_deg;
    float u1_x, u1_y;
    float u2_x, u2_y;
} AutoCorner;

typedef struct {
    float x_mm;
    float y_mm;
    float rx;
    float ry;
    uint32_t tick;
} HitSample;

typedef struct {
    float   goalX;
    float   goalY;
    uint8_t goalCol;
    uint8_t goalRow;
    bool    isFinishLine;
    uint8_t rejoinWpIdx;
    uint8_t maxDetour;
} PlanCandidate;

static AutoState      s_autoState = AUTO_STATE_IDLE;
static uint32_t       s_startTick = 0U;
static uint32_t       s_avoidStartTick = 0U;

static AutoWaypoint   s_activeWaypoints[AUTO_MAX_WAYPOINTS];
static uint8_t        s_numActiveWaypoints = 0U;
static uint8_t        s_activeWaypointIdx = 0U;

static AutoCorner     s_corners[AUTO_MAX_WAYPOINTS];
static uint8_t        s_numCorners = 0U;

static float          s_leftDistanceMm = 0.0f;
static float          s_midDistanceMm = 0.0f;
static float          s_rightDistanceMm = 0.0f;
static bool           s_leftValid = false;
static bool           s_midValid = false;
static bool           s_rightValid = false;
static bool           s_sensorsFresh = false;

/* Danh sách điểm biên tường bất ngờ (0 mm chiều sâu) đồng bộ với g_map_grid trong suốt lượt chạy */
static AutoWaypoint   s_dynWallPts[MAX_DYN_WALL_PTS];
static uint8_t        s_numDynWallPts = 0U;

static HitSample      s_hitSamples[CLUSTER_MAX_HITS];
static uint8_t        s_hitSampleCount = 0U;

/* Trạng thái máy lập đường A* chia lát cắt không chặn (Time-Sliced Planner) */
static AvoidPhase     s_avoidPhase = AVOID_PHASE_NONE;
static uint32_t       s_backoffStartTick = 0U;
static float          s_backoffOriginX = 0.0f;
static float          s_backoffOriginY = 0.0f;
static float          s_planStartX = 0.0f;
static float          s_planStartY = 0.0f;
static float          s_planStartHeadingDeg = 0.0f;
static uint8_t        s_planStartCol = 0U;
static uint8_t        s_planStartRow = 0U;
static uint8_t        s_inflateNextRow = 0U;

static PlanCandidate  s_candidates[MAX_PLAN_CANDIDATES];
static uint8_t        s_numCandidates = 0U;
static uint8_t        s_curCandidateIdx = 0U;
static bool           s_curSearchInitialized = false;

static AutoWaypoint   s_bestRoute[AUTO_MAX_WAYPOINTS];
static uint8_t        s_bestCandCount = 0U;
static float          s_bestCandCost = 1e9f;

/* Bộ nhớ tĩnh cho thuật toán A* */
static uint8_t        s_inflated[MAP_ROWS][MAP_COLS];
static uint16_t       s_gScore[MAP_TOTAL_CELLS];
static uint16_t       s_cameFrom[MAP_TOTAL_CELLS];
static uint8_t        s_closed[MAP_TOTAL_CELLS / 8U];
static uint16_t       s_heapNodes[HEAP_MAX_SIZE];
static uint16_t       s_heapKeys[HEAP_MAX_SIZE];
static uint16_t       s_heapSize = 0U;
static uint16_t       s_rawGridPath[256];
static uint16_t       s_rawPathLen = 0U;

/* ==========================================================================
 * CÁC HÀM TIỆN ÍCH NỘI BỘ & HÌNH HỌC AN TOÀN (MM)
 * ========================================================================== */
static inline bool Auto_IsBusyState(void) {
    return (s_autoState == AUTO_STATE_STARTING ||
            s_autoState == AUTO_STATE_RUNNING ||
            s_autoState == AUTO_STATE_AVOIDING);
}

static inline bool Auto_IsLastWaypoint(uint8_t idx) {
    return idx >= ((s_numActiveWaypoints > 1U) ? (s_numActiveWaypoints - 1U) : 0U);
}

static void Auto_ClearAllLeds(void) {
    Led_SetPattern(1U, 0U, 0U, 0U, 0U);
    Led_SetPattern(2U, 0U, 0U, 0U, 0U);
    Led_SetPattern(3U, 0U, 0U, 0U, 0U);
}

static void Auto_ClearDynamicWalls(void) {
    s_numDynWallPts = 0U;
}

static void Auto_ResetActiveSession(void) {
    s_activeWaypointIdx = 0U;
    s_avoidStartTick = 0U;
    s_backoffStartTick = 0U;
    s_avoidPhase = AVOID_PHASE_NONE;
    s_hitSampleCount = 0U;
    s_sensorsFresh = false;
    s_inflateNextRow = 0U;
    s_numCandidates = 0U;
    s_curCandidateIdx = 0U;
    s_curSearchInitialized = false;
    s_bestCandCount = 0U;
    s_bestCandCost = 1e9f;
}

static float Auto_NormalizeAngleDiff(float target_deg, float current_deg) {
    float diff = target_deg - current_deg;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

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

static float Auto_DistSqToBox(float px, float py, float xmin, float xmax, float ymin, float ymax) {
    float dx = (px < xmin) ? (xmin - px) : ((px > xmax) ? (px - xmax) : 0.0f);
    float dy = (py < ymin) ? (ymin - py) : ((py > ymax) ? (py - ymax) : 0.0f);
    return dx * dx + dy * dy;
}

/* Lọc điểm chạm thuộc tường biên, vùng cấm chữ U hoặc 17 trụ tĩnh T1..T17 (bù trôi odom thực tế) */
static bool Auto_IsKnownStaticObstacle(float x_mm, float y_mm, float extraMarginMm) {
    float wallMargin = 95.0f + extraMarginMm * 0.5f;
    if (x_mm <= wallMargin || x_mm >= (MAP_WIDTH_MM - wallMargin) ||
        y_mm <= -60.0f || y_mm >= (MAP_HEIGHT_MM - wallMargin)) {
        return true;
    }
    /* Vùng cấm chữ U [600, 1200] x [0, 600]: lề X<=1262 giữ nguyên hành lang X=1285 và khe T14 (Y=700) */
    if (x_mm >= (540.0f - extraMarginMm) && x_mm <= (1262.0f + extraMarginMm * 0.3f) &&
        y_mm >= -20.0f && y_mm <= (652.0f + extraMarginMm * 0.4f)) {
        return true;
    }

    float pillarMargin = 57.0f + extraMarginMm * 0.5f;
    float pillarMarginSq = pillarMargin * pillarMargin;
    for (uint8_t i = 0U; i < STATIC_OBSTACLE_COUNT; i++) {
        float cx = g_static_obstacles[i].x_mm;
        float cy = g_static_obstacles[i].y_mm;
        if (Auto_DistSqToBox(x_mm, y_mm, cx - 50.0f, cx + 50.0f, cy - 50.0f, cy + 50.0f) <= pillarMarginSq) {
            return true;
        }
    }
    return false;
}

/* Lưu điểm biên tường bất ngờ lên g_map_grid (255) và danh sách biên mỏng (không tạo khối hộp) */
static bool Auto_RecordWallPoint(float wx_mm, float wy_mm, bool *outIsNew) {
    uint8_t wc = 0U, wr = 0U;
    if (!Map_MmToGrid(wx_mm, wy_mm, &wc, &wr)) {
        return false;
    }
    g_map_grid[wr][wc] = MAP_VAL_OBSTACLE;

    const float mergeSq = WALL_PT_MERGE_DIST_MM * WALL_PT_MERGE_DIST_MM;
    for (uint8_t i = 0U; i < s_numDynWallPts; i++) {
        float dx = s_dynWallPts[i].x_mm - wx_mm;
        float dy = s_dynWallPts[i].y_mm - wy_mm;
        if ((dx * dx + dy * dy) <= mergeSq) {
            /* Làm mịn nhẹ tọa độ điểm biên đã biết */
            s_dynWallPts[i].x_mm = 0.7f * s_dynWallPts[i].x_mm + 0.3f * wx_mm;
            s_dynWallPts[i].y_mm = 0.7f * s_dynWallPts[i].y_mm + 0.3f * wy_mm;
            if (outIsNew != NULL) *outIsNew = false;
            return true;
        }
    }

    if (s_numDynWallPts < MAX_DYN_WALL_PTS) {
        s_dynWallPts[s_numDynWallPts++] = (AutoWaypoint){ wx_mm, wy_mm };
    } else {
        memmove(&s_dynWallPts[0], &s_dynWallPts[1], sizeof(AutoWaypoint) * (MAX_DYN_WALL_PTS - 1U));
        s_dynWallPts[MAX_DYN_WALL_PTS - 1U] = (AutoWaypoint){ wx_mm, wy_mm };
    }
    if (outIsNew != NULL) *outIsNew = true;
    return true;
}

/* Bình phương khoảng cách nhỏ nhất (mm^2) tới cản tĩnh + các điểm biên tường bất ngờ đã lưu */
static void Auto_ComputeClearanceSqMm(float px, float py, float *outObsClearSqMm, float *outWallClearMm) {
    float wallMin = fminf(px, fminf(MAP_WIDTH_MM - px, MAP_HEIGHT_MM - py));
    if (px < 600.0f && (px < 210.0f || px > 390.0f) && py < wallMin) {
        wallMin = py;
    }

    float minObsSq = Auto_DistSqToBox(px, py, 600.0f, 1200.0f, -150.0f, 600.0f);

    for (uint8_t i = 0U; i < STATIC_OBSTACLE_COUNT; i++) {
        float cx = g_static_obstacles[i].x_mm;
        float cy = g_static_obstacles[i].y_mm;
        float dSq = Auto_DistSqToBox(px, py, cx - 50.0f, cx + 50.0f, cy - 50.0f, cy + 50.0f);
        if (dSq < minObsSq) minObsSq = dSq;
    }

    for (uint8_t i = 0U; i < s_numDynWallPts; i++) {
        float wx = s_dynWallPts[i].x_mm;
        float wy = s_dynWallPts[i].y_mm;
        if (fabsf(px - wx) > 125.0f || fabsf(py - wy) > 125.0f) continue;
        float dSq = Auto_DistSqToBox(px, py, wx - WALL_PT_HALF_SIZE_MM, wx + WALL_PT_HALF_SIZE_MM,
                                     wy - WALL_PT_HALF_SIZE_MM, wy + WALL_PT_HALF_SIZE_MM);
        if (dSq < minObsSq) minObsSq = dSq;
    }

    *outObsClearSqMm = minObsSq;
    *outWallClearMm = wallMin;
}

static void Auto_ComputeClearanceMm(float px, float py, float *outObsClearMm, float *outWallClearMm) {
    float obsSq = 0.0f;
    Auto_ComputeClearanceSqMm(px, py, &obsSq, outWallClearMm);
    *outObsClearMm = sqrtf(obsSq);
}

static bool Auto_IsSegmentSafeMm(float x0, float y0, float x1, float y1,
                                 float reqObsClearMm, float reqWallClearMm) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float segLen = sqrtf(dx * dx + dy * dy);

    float startObsClear = 0.0f, startWallClear = 0.0f;
    Auto_ComputeClearanceMm(x0, y0, &startObsClear, &startWallClear);

    int steps = (int)ceilf(segLen / 12.0f);
    if (steps < 1) steps = 1;

    for (int s = 1; s <= steps; s++) {
        float t = (float)s / (float)steps;
        float px = x0 + t * dx;
        float py = y0 + t * dy;
        float obsC = 0.0f, wallC = 0.0f;
        Auto_ComputeClearanceMm(px, py, &obsC, &wallC);

        if ((t * segLen) < 65.0f && (startObsClear < reqObsClearMm || startWallClear < reqWallClearMm)) {
            float minAllowedObs  = fminf(reqObsClearMm,  fmaxf(45.0f, startObsClear  - 1.0f));
            float minAllowedWall = fminf(reqWallClearMm, fmaxf(48.0f, startWallClear - 1.0f));
            if (obsC < minAllowedObs || wallC < minAllowedWall) return false;
        } else {
            if (obsC < reqObsClearMm || wallC < reqWallClearMm) return false;
        }
    }
    return true;
}

static bool Auto_IsWallPointBlockingSegment(float wx, float wy,
                                            float x0, float y0, float x1, float y1,
                                            float clearMm, float maxCheckLenMm) {
    float dx = x1 - x0, dy = y1 - y0;
    float segLen = sqrtf(dx * dx + dy * dy);
    if (segLen < 1.0f || maxCheckLenMm <= 0.0f) return false;

    float cellXMin = wx - WALL_PT_HALF_SIZE_MM;
    float cellXMax = wx + WALL_PT_HALF_SIZE_MM;
    float cellYMin = wy - WALL_PT_HALF_SIZE_MM;
    float cellYMax = wy + WALL_PT_HALF_SIZE_MM;

    float checkLen = (segLen < maxCheckLenMm) ? segLen : maxCheckLenMm;
    int steps = (int)ceilf(checkLen / 12.0f);
    if (steps < 1) steps = 1;
    float threshSq = clearMm * clearMm;
    float startDist = sqrtf(Auto_DistSqToBox(x0, y0, cellXMin, cellXMax, cellYMin, cellYMax));

    float ux = dx / segLen, uy = dy / segLen;
    for (int s = 1; s <= steps; s++) {
        float sDist = checkLen * ((float)s / (float)steps);
        float px = x0 + sDist * ux;
        float py = y0 + sDist * uy;
        float dSq = Auto_DistSqToBox(px, py, cellXMin, cellXMax, cellYMin, cellYMax);
        if (sDist < 65.0f && startDist < clearMm) {
            float minAllow = (startDist > 2.0f) ? (startDist - 2.0f) : 0.0f;
            if (dSq < (minAllow * minAllow)) return true;
        } else {
            if (dSq < threshSq) return true;
        }
    }
    return false;
}

static bool Auto_IsWallPointBlockingFirstSegment(float wx, float wy,
                                                 float x0, float y0, float headingDeg,
                                                 float x1, float y1, float clearMm, float maxCheckLenMm) {
    float dx = x1 - x0, dy = y1 - y0;
    float segLen = sqrtf(dx * dx + dy * dy);
    if (segLen < 1.0f || maxCheckLenMm <= 0.0f) return false;

    float cellXMin = wx - WALL_PT_HALF_SIZE_MM;
    float cellXMax = wx + WALL_PT_HALF_SIZE_MM;
    float cellYMin = wy - WALL_PT_HALF_SIZE_MM;
    float cellYMax = wy + WALL_PT_HALF_SIZE_MM;

    float ux = dx / segLen, uy = dy / segLen;
    if (((wx - x0) * ux + (wy - y0) * uy) <= 0.0f) {
        return false;
    }

    float headRad = headingDeg * DEG_TO_RAD_F;
    float fwdX = cosf(headRad), fwdY = sinf(headRad);

    float checkLen = (segLen < maxCheckLenMm) ? segLen : maxCheckLenMm;
    int steps = (int)ceilf(checkLen / 12.0f);
    if (steps < 1) steps = 1;
    float threshSq = clearMm * clearMm;
    float startDist = sqrtf(Auto_DistSqToBox(x0, y0, cellXMin, cellXMax, cellYMin, cellYMax));

    for (int s = 1; s <= steps; s++) {
        float sDist = checkLen * ((float)s / (float)steps);
        float px = x0 + sDist * ux;
        float py = y0 + sDist * uy;
        if (((px - x0) * fwdX + (py - y0) * fwdY) <= 0.0f) {
            continue;
        }
        float dSq = Auto_DistSqToBox(px, py, cellXMin, cellXMax, cellYMin, cellYMax);
        if (sDist < 65.0f && startDist < clearMm) {
            float minAllow = (startDist > 2.0f) ? (startDist - 2.0f) : 0.0f;
            if (dSq < (minAllow * minAllow)) return true;
        } else {
            if (dSq < threshSq) return true;
        }
    }
    return false;
}

static bool Auto_IsWallPointBlockingRoute(float wx, float wy,
                                          float startX, float startY, float headingDeg,
                                          const AutoWaypoint *wps, uint8_t firstIdx, uint8_t count, float clearMm) {
    if (firstIdx >= count) return false;
    float remLookahead = ROUTE_BLOCK_LOOKAHEAD_MM;

    if (Auto_IsWallPointBlockingFirstSegment(wx, wy, startX, startY, headingDeg,
                                             wps[firstIdx].x_mm, wps[firstIdx].y_mm, clearMm, remLookahead)) {
        return true;
    }

    float dx0 = wps[firstIdx].x_mm - startX;
    float dy0 = wps[firstIdx].y_mm - startY;
    remLookahead -= sqrtf(dx0 * dx0 + dy0 * dy0);

    for (uint8_t k = firstIdx; (k + 1U) < count && remLookahead > 0.0f; k++) {
        if (Auto_IsWallPointBlockingSegment(wx, wy,
                                            wps[k].x_mm, wps[k].y_mm,
                                            wps[k + 1U].x_mm, wps[k + 1U].y_mm, clearMm, remLookahead)) {
            return true;
        }
        float dxk = wps[k + 1U].x_mm - wps[k].x_mm;
        float dyk = wps[k + 1U].y_mm - wps[k].y_mm;
        remLookahead -= sqrtf(dxk * dxk + dyk * dyk);
    }
    return false;
}

/* ==========================================================================
 * LẬP KẾ HOẠCH CUNG TRÒN TIẾP TUYẾN ICR
 * ========================================================================== */
static bool Auto_CheckArcSafe(AutoWaypoint Tin, AutoWaypoint Tout, AutoWaypoint C, float R, int8_t dir) {
    float aIn = atan2f(Tin.y_mm - C.y_mm, Tin.x_mm - C.x_mm);
    float aOut = atan2f(Tout.y_mm - C.y_mm, Tout.x_mm - C.x_mm);
    float sweep = aOut - aIn;
    if (dir > 0 && sweep < 0.0f) sweep += 2.0f * 3.141592653589793f;
    if (dir < 0 && sweep > 0.0f) sweep -= 2.0f * 3.141592653589793f;

    const int N = 16;
    for (int i = 0; i <= N; i++) {
        float a = aIn + sweep * ((float)i / (float)N);
        float qx = C.x_mm + R * cosf(a);
        float qy = C.y_mm + R * sinf(a);
        if (qy < -80.0f) return false;

        float obsClear = 0.0f, wallClear = 0.0f;
        Auto_ComputeClearanceMm(qx, qy, &obsClear, &wallClear);
        if (obsClear < ARC_SAFE_CLEAR_MM || wallClear < 65.0f) return false;

        /* Không ôm cung tròn ICR sát điểm biên tường lạ (ưu tiên quay tại chỗ để ToF quét tiếp mép tường) */
        const float reqDynArcSq = (ARC_SAFE_CLEAR_MM + WALL_PT_HALF_SIZE_MM + ARC_DYN_WALL_EXTRA_MM) *
                                  (ARC_SAFE_CLEAR_MM + WALL_PT_HALF_SIZE_MM + ARC_DYN_WALL_EXTRA_MM);
        for (uint8_t w = 0U; w < s_numDynWallPts; w++) {
            float dxw = qx - s_dynWallPts[w].x_mm;
            float dyw = qy - s_dynWallPts[w].y_mm;
            if ((dxw * dxw + dyw * dyw) < reqDynArcSq) return false;
        }
    }
    return true;
}

static void Auto_BuildPlan(const AutoWaypoint *wps, uint8_t numWps, float startX, float startY) {
    s_numCorners = 0U;
    if (numWps == 0U) return;

    AutoWaypoint pts[AUTO_MAX_WAYPOINTS + 1U];
    pts[0].x_mm = startX;
    pts[0].y_mm = startY;
    uint8_t capped = (numWps > AUTO_MAX_WAYPOINTS) ? AUTO_MAX_WAYPOINTS : numWps;
    memcpy(&pts[1], wps, sizeof(AutoWaypoint) * capped);
    uint8_t totalPts = capped + 1U;

    for (uint8_t i = 1U; i < (totalPts - 1U); i++) {
        AutoWaypoint pPrev = pts[i - 1U], p = pts[i], pNext = pts[i + 1U];

        float v1_x = p.x_mm - pPrev.x_mm, v1_y = p.y_mm - pPrev.y_mm;
        float L1 = sqrtf(v1_x * v1_x + v1_y * v1_y);
        float u1_x = (L1 > 0.0f) ? (v1_x / L1) : 1.0f;
        float u1_y = (L1 > 0.0f) ? (v1_y / L1) : 0.0f;

        float v2_x = pNext.x_mm - p.x_mm, v2_y = pNext.y_mm - p.y_mm;
        float L2 = sqrtf(v2_x * v2_x + v2_y * v2_y);
        float u2_x = (L2 > 0.0f) ? (v2_x / L2) : 1.0f;
        float u2_y = (L2 > 0.0f) ? (v2_y / L2) : 0.0f;

        float th1_deg = atan2f(u1_y, u1_x) * RAD_TO_DEG_F;
        float th2_deg = atan2f(u2_y, u2_x) * RAD_TO_DEG_F;
        float dth = Auto_NormalizeAngleDiff(th2_deg, th1_deg);

        AutoCorner *cn = &s_corners[s_numCorners++];
        cn->wp = p;
        cn->th1_deg = th1_deg;
        cn->th2_deg = th2_deg;
        cn->u1_x = u1_x;
        cn->u1_y = u1_y;
        cn->u2_x = u2_x;
        cn->u2_y = u2_y;
        cn->dth_deg = dth;

        if (fabsf(dth) < 18.0f) {
            cn->type = CORNER_NONE;
            continue;
        }

        float tanHalfBeta = tanf(fabsf(dth) * DEG_TO_RAD_F * 0.5f);
        float maxD1 = (i == 1U) ? (L1 * 0.85f) : (L1 * 0.48f);
        float maxD2 = (i == (totalPts - 2U)) ? (L2 * 0.85f) : (L2 * 0.48f);
        float dMax = fminf(maxD1, fminf(maxD2, 120.0f));
        int8_t dir = (dth > 0.0f) ? 1 : -1;
        float n1_x = (dir > 0) ? -u1_y : u1_y;
        float n1_y = (dir > 0) ?  u1_x : -u1_x;

        bool arcFound = false;
        for (float R_try = NAV_ARC_RADIUS_NOMINAL_MM; R_try >= 25.0f; R_try -= 5.0f) {
            float R = R_try;
            float d = R * tanHalfBeta;
            if (d > dMax && tanHalfBeta > 0.001f) {
                R = dMax / tanHalfBeta;
                d = dMax;
            }
            if (R < 25.0f || d < 12.0f) break;

            AutoWaypoint Tin  = { p.x_mm - d * u1_x, p.y_mm - d * u1_y };
            AutoWaypoint Tout = { p.x_mm + d * u2_x, p.y_mm + d * u2_y };
            AutoWaypoint C    = { Tin.x_mm + R * n1_x, Tin.y_mm + R * n1_y };

            if (Auto_CheckArcSafe(Tin, Tout, C, R, dir)) {
                cn->type = CORNER_ARC;
                cn->Tin = Tin;
                cn->Tout = Tout;
                cn->R = R;
                cn->dir = dir;
                arcFound = true;
                break;
            }
        }
        if (!arcFound) {
            cn->type = (fabsf(dth) < 30.0f) ? CORNER_NONE : CORNER_SPIN;
        }
    }
}

static void Auto_GetAimTarget(uint8_t idx, float *outX, float *outY) {
    if (idx < s_numCorners && s_corners[idx].type == CORNER_ARC) {
        *outX = s_corners[idx].Tin.x_mm;
        *outY = s_corners[idx].Tin.y_mm;
    } else if (idx < s_numActiveWaypoints) {
        *outX = s_activeWaypoints[idx].x_mm;
        *outY = s_activeWaypoints[idx].y_mm;
    }
}

/* ==========================================================================
 * MIN-HEAP, LOS & TÌM ĐƯỜNG A* CHIA LÁT CẮT KHÔNG CHẶN (TIME-SLICED A*)
 * ========================================================================== */
static inline void Heap_Clear(void) { s_heapSize = 0U; }
static inline void ClosedSet_Clear(void) { memset(s_closed, 0, sizeof(s_closed)); }
static inline void ClosedSet_Set(uint16_t node) { s_closed[node >> 3U] |= (uint8_t)(1U << (node & 7U)); }
static inline bool ClosedSet_Contains(uint16_t node) { return (s_closed[node >> 3U] & (uint8_t)(1U << (node & 7U))) != 0U; }

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
            uint16_t sm = (right < s_heapSize && s_heapKeys[right] < s_heapKeys[left]) ? right : left;
            if (s_heapKeys[sm] >= lastKey) break;
            s_heapNodes[i] = s_heapNodes[sm];
            s_heapKeys[i]  = s_heapKeys[sm];
            i = sm;
        }
        s_heapNodes[i] = lastNode;
        s_heapKeys[i]  = lastKey;
    }
    return true;
}

static inline uint16_t Auto_Heuristic(int16_t c, int16_t r, int16_t goalC, int16_t goalR, bool isFinishLine) {
    int16_t dc = 0, dr = 0;
    if (isFinishLine) {
        if (c < (int16_t)FINISH_LINE_COL_MIN) dc = (int16_t)FINISH_LINE_COL_MIN - c;
        else if (c > (int16_t)FINISH_LINE_COL_MAX) dc = c - (int16_t)FINISH_LINE_COL_MAX;
        dr = r;
    } else {
        dc = (goalC >= c) ? (goalC - c) : (c - goalC);
        dr = (goalR >= r) ? (goalR - r) : (r - goalR);
    }
    return (uint16_t)((dc > dr) ? (14 * dr + 10 * (dc - dr)) : (14 * dc + 10 * (dr - dc)));
}

static bool Auto_LineOfSight(int16_t c0, int16_t r0, int16_t c1, int16_t r1) {
    int16_t dc = (c1 > c0) ? (c1 - c0) : (c0 - c1);
    int16_t dr = (r1 > r0) ? (r1 - r0) : (r0 - r1);
    int16_t sc = (c0 < c1) ? 1 : -1;
    int16_t sr = (r0 < r1) ? 1 : -1;
    int16_t err = dc - dr;
    int16_t x = c0, y = r0;

    while (1) {
        if (x >= 0 && x < (int16_t)MAP_COLS && y >= 0 && y < (int16_t)MAP_ROWS) {
            if (s_inflated[y][x] == MAP_VAL_OBSTACLE) return false;
            if (s_inflated[y][x] >= 60U && !(x == c0 && y == r0)) return false;
        }
        if (x == c1 && y == r1) break;

        int16_t e2 = 2 * err, prevX = x, prevY = y;
        if (e2 > -dr) { err -= dr; x += sc; }
        if (e2 < dc)  { err += dc; y += sr; }

        if (x != prevX && y != prevY) {
            if (s_inflated[prevY][x] == MAP_VAL_OBSTACLE || s_inflated[y][prevX] == MAP_VAL_OBSTACLE) {
                return false;
            }
        }
    }
    return true;
}

static void Auto_PruneRouteByLOS(float startX, float startY, AutoWaypoint *wps, uint8_t *pCount, bool aggressiveLos) {
    uint8_t count = *pCount;
    if (count < 2U) return;

    uint8_t i = 0U;
    while (i < (count - 1U)) {
        AutoWaypoint pPrev = (i == 0U) ? (AutoWaypoint){ startX, startY } : wps[i - 1U];
        AutoWaypoint pCurr = wps[i];
        AutoWaypoint pNext = wps[i + 1U];

        float v1_x = pCurr.x_mm - pPrev.x_mm, v1_y = pCurr.y_mm - pPrev.y_mm;
        float d1 = sqrtf(v1_x * v1_x + v1_y * v1_y);
        float th1_deg = atan2f(v1_y, v1_x) * RAD_TO_DEG_F;
        float th2_deg = atan2f(pNext.y_mm - pCurr.y_mm, pNext.x_mm - pCurr.x_mm) * RAD_TO_DEG_F;
        float dth = fabsf(Auto_NormalizeAngleDiff(th2_deg, th1_deg));

        bool crossesFinishAngled = (pNext.y_mm < 0.0f && fabsf(pNext.x_mm - pPrev.x_mm) > 35.0f);
        if (!crossesFinishAngled && (aggressiveLos || dth < 10.0f || d1 < 60.0f) &&
            Auto_IsSegmentSafeMm(pPrev.x_mm, pPrev.y_mm, pNext.x_mm, pNext.y_mm, LOS_SAFE_CLEAR_MM, 74.0f)) {
            memmove(&wps[i], &wps[i + 1U], sizeof(AutoWaypoint) * (count - 1U - i));
            count--;
            continue;
        }
        i++;
    }
    *pCount = count;
}

static float Auto_EvaluateRouteCost(float startX, float startY, float startHeadingDeg,
                                    const AutoWaypoint *wps, uint8_t count) {
    if (count == 0U) return 1e9f;
    float cost = 0.0f, prevX = startX, prevY = startY, prevHeading = startHeadingDeg;

    for (uint8_t i = 0U; i < count; i++) {
        float dx = wps[i].x_mm - prevX, dy = wps[i].y_mm - prevY;
        float segLen = sqrtf(dx * dx + dy * dy);
        cost += segLen;
        if (segLen > 1.0f) {
            float segHeading = atan2f(dy, dx) * RAD_TO_DEG_F;
            float dth = fabsf(Auto_NormalizeAngleDiff(segHeading, prevHeading));
            cost += dth * 1.8f + ((dth >= 45.0f) ? 120.0f : 0.0f);
            prevHeading = segHeading;
        }
        prevX = wps[i].x_mm;
        prevY = wps[i].y_mm;
    }
    return cost;
}

static uint8_t Auto_ReconstructAndSmoothPath(uint16_t startNode, uint16_t reachedGoalNode,
                                             const PlanCandidate *cand,
                                             AutoWaypoint *outWp, uint8_t maxCount) {
    s_rawPathLen = 0U;
    uint16_t curr = reachedGoalNode;
    while (curr != 0xFFFFU && s_rawPathLen < 256U) {
        s_rawGridPath[s_rawPathLen++] = curr;
        if (curr == startNode) break;
        curr = s_cameFrom[curr];
    }
    for (uint16_t i = 0U; i < (s_rawPathLen / 2U); i++) {
        uint16_t tmp = s_rawGridPath[i];
        s_rawGridPath[i] = s_rawGridPath[s_rawPathLen - 1U - i];
        s_rawGridPath[s_rawPathLen - 1U - i] = tmp;
    }

    uint8_t count = 0U;
    uint16_t anchorIdx = 0U;
    while (s_rawPathLen > 1U && anchorIdx < (s_rawPathLen - 1U)) {
        uint16_t furthestIdx = anchorIdx + 1U;
        uint8_t ac = (uint8_t)(s_rawGridPath[anchorIdx] % MAP_COLS);
        uint8_t ar = (uint8_t)(s_rawGridPath[anchorIdx] / MAP_COLS);
        float ax = (anchorIdx == 0U) ? s_planStartX : (((float)ac + 0.5f) * MAP_CELL_SIZE_MM);
        float ay = (anchorIdx == 0U) ? s_planStartY : (((float)ar + 0.5f) * MAP_CELL_SIZE_MM);

        for (uint16_t i = s_rawPathLen - 1U; i > (anchorIdx + 1U); i--) {
            uint8_t tc = (uint8_t)(s_rawGridPath[i] % MAP_COLS);
            uint8_t tr = (uint8_t)(s_rawGridPath[i] / MAP_COLS);
            float tx = ((float)tc + 0.5f) * MAP_CELL_SIZE_MM;
            float ty = ((float)tr + 0.5f) * MAP_CELL_SIZE_MM;
            if (Auto_LineOfSight(ac, ar, tc, tr) &&
                Auto_IsSegmentSafeMm(ax, ay, tx, ty, LOS_SAFE_CLEAR_MM, 74.0f)) {
                furthestIdx = i;
                break;
            }
        }

        if (count < maxCount) {
            outWp[count].x_mm = ((float)(s_rawGridPath[furthestIdx] % MAP_COLS) + 0.5f) * MAP_CELL_SIZE_MM;
            outWp[count].y_mm = ((float)(s_rawGridPath[furthestIdx] / MAP_COLS) + 0.5f) * MAP_CELL_SIZE_MM;
            count++;
        }
        anchorIdx = furthestIdx;
    }

    if (count == 0U && s_rawPathLen == 1U && maxCount > 0U) {
        count = 1U;
        outWp[0] = (AutoWaypoint){ cand->goalX, cand->goalY };
    }

    if (count > 0U) {
        if (cand->isFinishLine) {
            float finalX = ((float)(reachedGoalNode % MAP_COLS) + 0.5f) * MAP_CELL_SIZE_MM;
            float prevX  = (count >= 2U) ? outWp[count - 2U].x_mm : s_planStartX;
            float prevY  = (count >= 2U) ? outWp[count - 2U].y_mm : s_planStartY;

            if (fabsf(finalX - prevX) <= 35.0f &&
                Auto_IsSegmentSafeMm(prevX, prevY, finalX, GOAL_FINISH_Y_MM, HARD_OBS_CLEAR_MM, HARD_WALL_CLEAR_MM)) {
                outWp[count - 1U] = (AutoWaypoint){ finalX, GOAL_FINISH_Y_MM };
            } else {
                outWp[count - 1U] = (AutoWaypoint){ finalX, 45.0f };
                if (count < maxCount) {
                    outWp[count++] = (AutoWaypoint){ finalX, GOAL_FINISH_Y_MM };
                } else {
                    outWp[count - 1U].y_mm = GOAL_FINISH_Y_MM;
                }
            }
        } else {
            outWp[count - 1U] = (AutoWaypoint){ cand->goalX, cand->goalY };
        }
    }

    Auto_PruneRouteByLOS(s_planStartX, s_planStartY, outWp, &count, false);
    return count;
}

static void Auto_PreparePlannerCandidates(void) {
    s_numCandidates = 0U;
    s_curCandidateIdx = 0U;
    s_curSearchInitialized = false;
    s_bestCandCount = 0U;
    s_bestCandCost = 1e9f;

    /* 1. Tối đa 3 mốc nối tiếp lộ trình khi xe còn ở X < 1200 mm */
    if (s_planStartX < 1200.0f &&
        s_activeWaypointIdx < ((s_numActiveWaypoints > 2U) ? (s_numActiveWaypoints - 2U) : 0U)) {
        uint8_t maxCandEnd = s_activeWaypointIdx + 3U;
        if (maxCandEnd > s_numActiveWaypoints) maxCandEnd = s_numActiveWaypoints;

        for (uint8_t candIdx = s_activeWaypointIdx;
             candIdx < maxCandEnd && s_numCandidates < (MAX_PLAN_CANDIDATES - 1U);
             candIdx++) {
            float tx = s_activeWaypoints[candIdx].x_mm;
            float ty = s_activeWaypoints[candIdx].y_mm;
            bool isGoalFinish = (candIdx >= (s_numActiveWaypoints - 1U)) || (ty <= 0.0f && tx >= 1200.0f);
            if (isGoalFinish) continue;

            uint8_t gc = 0U, gr = 0U;
            if (!Auto_MmToGrid(tx, ty, &gc, &gr) || s_inflated[gr][gc] >= 60U) {
                continue;
            }

            uint8_t tailLen = (uint8_t)(s_numActiveWaypoints - (candIdx + 1U));
            uint8_t maxDetour = (tailLen < AUTO_MAX_WAYPOINTS) ? (uint8_t)(AUTO_MAX_WAYPOINTS - tailLen) : 4U;

            s_candidates[s_numCandidates++] = (PlanCandidate){
                .goalX = tx,
                .goalY = ty,
                .goalCol = gc,
                .goalRow = gr,
                .isFinishLine = false,
                .rejoinWpIdx = candIdx,
                .maxDetour = maxDetour
            };
        }
    }

    /* 2. Luôn thêm ứng viên dự phòng trực tiếp về dải vạch đích [FINISH_LINE_COL_MIN..FINISH_LINE_COL_MAX], r = 0 */
    uint8_t finCol = 43U, finRow = 0U;
    Auto_MmToGrid(1305.0f, GOAL_FINISH_Y_MM, &finCol, &finRow);
    s_candidates[s_numCandidates++] = (PlanCandidate){
        .goalX = 1305.0f,
        .goalY = GOAL_FINISH_Y_MM,
        .goalCol = finCol,
        .goalRow = finRow,
        .isFinishLine = true,
        .rejoinWpIdx = 0U,
        .maxDetour = AUTO_MAX_WAYPOINTS
    };
}

bool Auto_StepPlannerSlice(void) {
    if (s_autoState != AUTO_STATE_AVOIDING || s_avoidPhase == AVOID_PHASE_NONE) {
        return false;
    }

    /* Pha 1: Chờ xe hãm quán tính; nếu dừng quá sát mép tường (< 56 mm khoảng hở -> góc chéo thân 58.5 mm dễ quệt khi quay tại chỗ)
     * thì lùi thẳng nhẹ ~35-45 mm dọc vết đường vừa đi tới để lấy khoảng trống xoay đầu an toàn */
    if (s_avoidPhase == AVOID_PHASE_SETTLE) {
        Odometry_State avoidOdom;
        if (Odometry_GetState(&avoidOdom) != HAL_OK) {
            return false;
        }
        uint32_t now = HAL_GetTick();
        float curObsC = 0.0f, curWallC = 0.0f;
        Auto_ComputeClearanceMm(avoidOdom.x_mm, avoidOdom.y_mm, &curObsC, &curWallC);
        bool tooCloseToSpin = (curObsC < 64.0f) || (s_midValid && s_midDistanceMm <= 75.0f);
        if (tooCloseToSpin) {
            /* Hãm ngược chủ động ngay lập tức và lùi thẳng 40 mm để mũi xe không trôi quệt mép tường */
            s_backoffStartTick = now;
            s_backoffOriginX = avoidOdom.x_mm;
            s_backoffOriginY = avoidOdom.y_mm;
            RunMotorLeft(-42);
            RunMotorRight(-42);
            s_avoidPhase = AVOID_PHASE_BACKOFF;
            return true;
        }

        bool settled = (fabsf(avoidOdom.linear_mm_s) <= AVOID_SETTLE_LIN_MM_S &&
                        fabsf(avoidOdom.angular_rad_s) <= AVOID_SETTLE_ANG_RAD_S);
        if (!settled && (now - s_avoidStartTick) < AVOID_SETTLE_MAX_MS) {
            return false;
        }

        s_planStartX = avoidOdom.x_mm;
        s_planStartY = avoidOdom.y_mm;
        s_planStartHeadingDeg = avoidOdom.heading_deg;
        Auto_MmToGrid(s_planStartX, s_planStartY, &s_planStartCol, &s_planStartRow);
        s_inflateNextRow = 0U;
        s_avoidPhase = AVOID_PHASE_INFLATE;
        return true;
    }

    if (s_avoidPhase == AVOID_PHASE_BACKOFF) {
        Odometry_State avoidOdom;
        if (Odometry_GetState(&avoidOdom) != HAL_OK) {
            return false;
        }
        uint32_t now = HAL_GetTick();
        float dxB = avoidOdom.x_mm - s_backoffOriginX;
        float dyB = avoidOdom.y_mm - s_backoffOriginY;
        float backedDistSq = dxB * dxB + dyB * dyB;
        float curObsC = 0.0f, curWallC = 0.0f;
        Auto_ComputeClearanceMm(avoidOdom.x_mm, avoidOdom.y_mm, &curObsC, &curWallC);

        float headRadB = avoidOdom.heading_deg * DEG_TO_RAD_F;
        float rearX = avoidOdom.x_mm - 45.0f * cosf(headRadB);
        float rearY = avoidOdom.y_mm - 45.0f * sinf(headRadB);
        float rearObsC = 0.0f, rearWallC = 0.0f;
        Auto_ComputeClearanceMm(rearX, rearY, &rearObsC, &rearWallC);

        bool enoughRoom = (curObsC >= 66.0f && backedDistSq >= (42.0f * 42.0f)) ||
                          backedDistSq >= (60.0f * 60.0f) ||
                          rearObsC < 20.0f || rearWallC < 22.0f ||
                          ((now - s_backoffStartTick) >= 340U);
        if (!enoughRoom) {
            return false;
        }

        Motion_Stop();
        s_avoidStartTick = now;
        s_avoidPhase = AVOID_PHASE_POST_BACKOFF_SETTLE;
        return true;
    }

    if (s_avoidPhase == AVOID_PHASE_POST_BACKOFF_SETTLE) {
        Odometry_State avoidOdom;
        if (Odometry_GetState(&avoidOdom) != HAL_OK) {
            return false;
        }
        uint32_t now = HAL_GetTick();
        bool settled = (fabsf(avoidOdom.linear_mm_s) <= AVOID_SETTLE_LIN_MM_S &&
                        fabsf(avoidOdom.angular_rad_s) <= AVOID_SETTLE_ANG_RAD_S);
        if (!settled && (now - s_avoidStartTick) < 120U) {
            return false;
        }

        s_planStartX = avoidOdom.x_mm;
        s_planStartY = avoidOdom.y_mm;
        s_planStartHeadingDeg = avoidOdom.heading_deg;
        Auto_MmToGrid(s_planStartX, s_planStartY, &s_planStartCol, &s_planStartRow);
        s_inflateNextRow = 0U;
        s_avoidPhase = AVOID_PHASE_INFLATE;
        return true;
    }

    /* Pha 2: Nở bản đồ 8 hàng mỗi lát cắt từ g_map_grid (cản tĩnh + các điểm biên tường bất ngờ đã lưu) */
    if (s_avoidPhase == AVOID_PHASE_INFLATE) {
        const float hardObsSq   = HARD_OBS_CLEAR_MM * HARD_OBS_CLEAR_MM;
        const float midObsSq    = 82.0f * 82.0f;
        const float softObsSq   = 105.0f * 105.0f;
        const float backRadSq   = 360.0f * 360.0f;
        const float startHeadRad = s_planStartHeadingDeg * DEG_TO_RAD_F;
        const float startFwdX = cosf(startHeadRad);
        const float startFwdY = sinf(startHeadRad);

        uint8_t endRow = (uint8_t)(s_inflateNextRow + INFLATE_ROWS_PER_SLICE);
        if (endRow > MAP_ROWS) endRow = MAP_ROWS;

        for (uint8_t r = s_inflateNextRow; r < endRow; r++) {
            float cy = ((float)r + 0.5f) * MAP_CELL_SIZE_MM;
            float dyStart = cy - s_planStartY;
            for (uint8_t c = 0U; c < MAP_COLS; c++) {
                if (g_map_grid[r][c] == MAP_VAL_OBSTACLE) {
                    s_inflated[r][c] = MAP_VAL_OBSTACLE;
                    continue;
                }
                float cx = ((float)c + 0.5f) * MAP_CELL_SIZE_MM;
                float obsClearSq = 0.0f, wallClear = 0.0f;
                Auto_ComputeClearanceSqMm(cx, cy, &obsClearSq, &wallClear);

                uint8_t cellCost = 0U;
                if (wallClear < HARD_WALL_CLEAR_MM || obsClearSq < hardObsSq) {
                    cellCost = MAP_VAL_OBSTACLE;
                } else if (wallClear < 95.0f || obsClearSq < midObsSq) {
                    cellCost = 60U;
                } else if (wallClear < 115.0f || obsClearSq < softObsSq) {
                    cellCost = 25U;
                }

                /* Phạt mềm vùng quay đầu ngược (> 95°) sau lưng xe (không khóa cứng 255) */
                if (cellCost < 95U) {
                    float dxStart = cx - s_planStartX;
                    float dStartSq = dxStart * dxStart + dyStart * dyStart;
                    if (dStartSq <= backRadSq && (dxStart * startFwdX + dyStart * startFwdY) < -5.0f) {
                        cellCost = 95U;
                    }
                }
                s_inflated[r][c] = cellCost;
            }
        }

        s_inflateNextRow = endRow;
        if (s_inflateNextRow < MAP_ROWS) {
            return true;
        }

        /* Đã nở xong 40 hàng: mở ô xuất phát & kiểm tra lối thoát hiểm nếu xe phanh sát biên tường */
        s_inflated[s_planStartRow][s_planStartCol] = MAP_VAL_FREE;

        static const int8_t cdir[4][2] = { {0, 1}, {0, -1}, {1, 0}, {-1, 0} };
        bool hasCardinalExit = false;
        for (uint8_t k = 0U; k < 4U; k++) {
            int nr = (int)s_planStartRow + cdir[k][1];
            int nc = (int)s_planStartCol + cdir[k][0];
            if (nr >= 0 && nr < (int)MAP_ROWS && nc >= 0 && nc < (int)MAP_COLS &&
                s_inflated[nr][nc] != MAP_VAL_OBSTACLE) {
                hasCardinalExit = true;
                break;
            }
        }
        if (!hasCardinalExit) {
            float sObs = 0.0f, sWall = 0.0f;
            Auto_ComputeClearanceMm(s_planStartX, s_planStartY, &sObs, &sWall);
            float startMinC = fminf(sObs, sWall);
            for (int8_t dr = -2; dr <= 2; dr++) {
                for (int8_t dc = -2; dc <= 2; dc++) {
                    if (dr == 0 && dc == 0) continue;
                    int nr = (int)s_planStartRow + dr;
                    int nc = (int)s_planStartCol + dc;
                    if (nr >= 0 && nr < (int)MAP_ROWS && nc >= 0 && nc < (int)MAP_COLS &&
                        g_map_grid[nr][nc] != MAP_VAL_OBSTACLE && s_inflated[nr][nc] == MAP_VAL_OBSTACLE) {
                        float nObs = 0.0f, nWall = 0.0f;
                        Auto_ComputeClearanceMm(((float)nc + 0.5f) * MAP_CELL_SIZE_MM,
                                                ((float)nr + 0.5f) * MAP_CELL_SIZE_MM, &nObs, &nWall);
                        float nMinC = fminf(nObs, nWall);
                        if ((nMinC >= 44.0f && nMinC > (startMinC + 3.0f)) ||
                            (startMinC < 44.0f && nMinC > (startMinC + 2.0f))) {
                            s_inflated[nr][nc] = 80U;
                        }
                    }
                }
            }
        }

        Auto_PreparePlannerCandidates();
        s_avoidPhase = (s_numCandidates > 0U) ? AVOID_PHASE_SEARCH : AVOID_PHASE_APPLY;
        return true;
    }

    /* Pha 3: Mở rộng tối đa 80 nút heap mỗi lát cắt */
    if (s_avoidPhase == AVOID_PHASE_SEARCH) {
        if (s_curCandidateIdx >= s_numCandidates) {
            s_avoidPhase = AVOID_PHASE_APPLY;
            return true;
        }

        const PlanCandidate *cand = &s_candidates[s_curCandidateIdx];
        if (cand->isFinishLine && s_bestCandCount > 0U) {
            s_curCandidateIdx++;
            s_curSearchInitialized = false;
            if (s_curCandidateIdx >= s_numCandidates) {
                s_avoidPhase = AVOID_PHASE_APPLY;
            }
            return true;
        }

        uint16_t startNode = (uint16_t)s_planStartRow * MAP_COLS + s_planStartCol;
        uint16_t targetGoalNode = (uint16_t)cand->goalRow * MAP_COLS + cand->goalCol;

        if (!s_curSearchInitialized) {
            memset(s_gScore, 0xFF, sizeof(s_gScore));
            memset(s_cameFrom, 0xFF, sizeof(s_cameFrom));
            ClosedSet_Clear();
            Heap_Clear();

            s_gScore[startNode] = 0U;
            Heap_Push(startNode, Auto_Heuristic((int16_t)s_planStartCol, (int16_t)s_planStartRow,
                                                (int16_t)cand->goalCol, (int16_t)cand->goalRow,
                                                cand->isFinishLine));
            s_curSearchInitialized = true;
        }

        static const int8_t dir[8][2] = {
            { 0, 1}, { 0, -1}, { 1, 0}, {-1, 0},
            { 1, 1}, { 1, -1}, {-1, 1}, {-1, -1}
        };
        static const uint16_t stepCost[8] = { 10U, 10U, 10U, 10U, 14U, 14U, 14U, 14U };

        uint16_t reachedNode = 0xFFFFU;
        bool searchDone = false;
        bool searchFound = false;

        uint16_t iter = 0U;
        while (s_heapSize > 0U && iter < ASTAR_NODES_PER_SLICE) {
            iter++;
            uint16_t current = 0U, curKey = 0U;
            if (!Heap_Pop(&current, &curKey)) break;

            uint8_t c = (uint8_t)(current % MAP_COLS);
            uint8_t r = (uint8_t)(current / MAP_COLS);

            if (cand->isFinishLine) {
                if (r == 0U && c >= FINISH_LINE_COL_MIN && c <= FINISH_LINE_COL_MAX) {
                    reachedNode = current;
                    searchFound = true;
                    searchDone = true;
                    break;
                }
            } else if (current == targetGoalNode) {
                reachedNode = current;
                searchFound = true;
                searchDone = true;
                break;
            }

            if (ClosedSet_Contains(current)) continue;
            ClosedSet_Set(current);

            int8_t prevDc = 0, prevDr = 0;
            bool hasPrev = (s_cameFrom[current] != 0xFFFFU);
            if (hasPrev) {
                uint16_t prevNode = s_cameFrom[current];
                prevDc = (int8_t)c - (int8_t)(prevNode % MAP_COLS);
                prevDr = (int8_t)r - (int8_t)(prevNode / MAP_COLS);
            }

            for (uint8_t d = 0U; d < 8U; d++) {
                int8_t dc = dir[d][0], dr = dir[d][1];
                int16_t nc = (int16_t)c + dc, nr = (int16_t)r + dr;
                if (nc < 0 || nc >= (int16_t)MAP_COLS || nr < 0 || nr >= (int16_t)MAP_ROWS) continue;
                if (s_inflated[nr][nc] == MAP_VAL_OBSTACLE) continue;
                if (dc != 0 && dr != 0) {
                    if (g_map_grid[r][nc] == MAP_VAL_OBSTACLE || g_map_grid[nr][c] == MAP_VAL_OBSTACLE) {
                        continue;
                    }
                    if (current != startNode &&
                        (s_inflated[r][nc] == MAP_VAL_OBSTACLE || s_inflated[nr][c] == MAP_VAL_OBSTACLE)) {
                        float midX = ((float)c + (float)nc + 1.0f) * (MAP_CELL_SIZE_MM * 0.5f);
                        float midY = ((float)r + (float)nr + 1.0f) * (MAP_CELL_SIZE_MM * 0.5f);
                        float midObsSq = 0.0f, midWall = 0.0f;
                        Auto_ComputeClearanceSqMm(midX, midY, &midObsSq, &midWall);
                        if (midWall < HARD_WALL_CLEAR_MM || midObsSq < (HARD_OBS_CLEAR_MM * HARD_OBS_CLEAR_MM)) {
                            continue;
                        }
                    }
                }

                uint16_t neighbor = (uint16_t)nr * MAP_COLS + (uint16_t)nc;
                if (ClosedSet_Contains(neighbor)) continue;

                uint16_t turnPenalty = 0U;
                if (hasPrev) {
                    if (!(dc == prevDc && dr == prevDr)) {
                        turnPenalty = ((dc * prevDc + dr * prevDr) > 0) ? 12U : 30U;
                    }
                } else {
                    float stepDeg = atan2f((float)dr, (float)dc) * RAD_TO_DEG_F;
                    float dthStart = fabsf(Auto_NormalizeAngleDiff(stepDeg, s_planStartHeadingDeg));
                    turnPenalty = (uint16_t)(dthStart * 0.85f);
                }

                uint16_t tentativeG = s_gScore[current] + stepCost[d] + (uint16_t)s_inflated[nr][nc] + turnPenalty;
                if (tentativeG < s_gScore[neighbor]) {
                    s_cameFrom[neighbor] = current;
                    s_gScore[neighbor]   = tentativeG;
                    uint16_t h = Auto_Heuristic(nc, nr, (int16_t)cand->goalCol, (int16_t)cand->goalRow, cand->isFinishLine);
                    Heap_Push(neighbor, tentativeG + h);
                }
            }
        }

        if (!searchDone && s_heapSize == 0U) {
            searchDone = true;
            searchFound = false;
        }

        if (searchDone) {
            if (searchFound) {
                AutoWaypoint candRoute[AUTO_MAX_WAYPOINTS];
                uint8_t candCount = Auto_ReconstructAndSmoothPath(startNode, reachedNode, cand,
                                                                  candRoute, cand->maxDetour);
                bool tailBlocked = false;
                if (candCount > 0U && !cand->isFinishLine) {
                    for (uint8_t i = cand->rejoinWpIdx + 1U;
                         i < s_numActiveWaypoints && candCount < AUTO_MAX_WAYPOINTS;
                         i++) {
                        AutoWaypoint pPrevTail = candRoute[candCount - 1U];
                        if (!Auto_IsSegmentSafeMm(pPrevTail.x_mm, pPrevTail.y_mm,
                                                  s_activeWaypoints[i].x_mm, s_activeWaypoints[i].y_mm,
                                                  HARD_OBS_CLEAR_MM, HARD_WALL_CLEAR_MM)) {
                            tailBlocked = true;
                            break;
                        }
                        candRoute[candCount++] = s_activeWaypoints[i];
                    }
                    if (!tailBlocked) {
                        Auto_PruneRouteByLOS(s_planStartX, s_planStartY, candRoute, &candCount, true);
                    }
                }

                if (candCount > 0U && !tailBlocked) {
                    float candCost = Auto_EvaluateRouteCost(s_planStartX, s_planStartY,
                                                            s_planStartHeadingDeg, candRoute, candCount);
                    if (candCost < s_bestCandCost) {
                        s_bestCandCost = candCost;
                        s_bestCandCount = candCount;
                        memcpy(s_bestRoute, candRoute, sizeof(AutoWaypoint) * candCount);
                    }
                }
            }

            s_curCandidateIdx++;
            s_curSearchInitialized = false;
            if (s_curCandidateIdx >= s_numCandidates) {
                s_avoidPhase = AVOID_PHASE_APPLY;
            }
        }
        return true;
    }

    /* Pha 4: Áp dụng lộ trình mới -> lập cung tròn ICR (vẫn giữ biên tường đã phát hiện trên g_map_grid) */
    if (s_avoidPhase == AVOID_PHASE_APPLY) {
        Odometry_State curOdom;
        if (Odometry_GetState(&curOdom) != HAL_OK) {
            curOdom.x_mm = s_planStartX;
            curOdom.y_mm = s_planStartY;
        }

        if (s_bestCandCount > 0U) {
            memcpy(s_activeWaypoints, s_bestRoute, sizeof(AutoWaypoint) * s_bestCandCount);
            s_numActiveWaypoints = s_bestCandCount;
            s_activeWaypointIdx = 0U;

            if (s_numActiveWaypoints > 1U) {
                float dx = s_activeWaypoints[0].x_mm - curOdom.x_mm;
                float dy = s_activeWaypoints[0].y_mm - curOdom.y_mm;
                if ((dx * dx + dy * dy) < (25.0f * 25.0f)) {
                    memmove(&s_activeWaypoints[0], &s_activeWaypoints[1],
                            sizeof(AutoWaypoint) * (s_numActiveWaypoints - 1U));
                    s_numActiveWaypoints--;
                }
            }

            Auto_BuildPlan(s_activeWaypoints, s_numActiveWaypoints, curOdom.x_mm, curOdom.y_mm);

            s_avoidPhase = AVOID_PHASE_NONE;
            s_autoState = AUTO_STATE_RUNNING;
            s_hitSampleCount = 0U;

            float targetX = s_activeWaypoints[0].x_mm;
            float targetY = s_activeWaypoints[0].y_mm;
            if (s_numCorners > 0U && s_corners[0].type == CORNER_ARC) {
                float dx = s_corners[0].Tin.x_mm - curOdom.x_mm;
                float dy = s_corners[0].Tin.y_mm - curOdom.y_mm;
                if ((dx * dx + dy * dy) >= (35.0f * 35.0f)) {
                    targetX = s_corners[0].Tin.x_mm;
                    targetY = s_corners[0].Tin.y_mm;
                } else {
                    s_corners[0].type = (fabsf(s_corners[0].dth_deg) < 20.0f) ? CORNER_NONE : CORNER_SPIN;
                }
            }
            Motion_NavToPoint(targetX, targetY, Auto_IsLastWaypoint(0U));
            return true;
        }

        s_avoidPhase = AVOID_PHASE_NONE;
        s_hitSampleCount = 0U;

        if (s_midValid && s_midDistanceMm <= 120.0f) {
            Motion_Stop();
            s_autoState = AUTO_STATE_ERROR;
            Led_SetPattern(1U, 1U, 80U, 80U, 80U);
            return true;
        }

        s_autoState = AUTO_STATE_RUNNING;
        float resumeX = 0.0f, resumeY = 0.0f;
        Auto_GetAimTarget(s_activeWaypointIdx, &resumeX, &resumeY);
        Motion_NavToPoint(resumeX, resumeY, Auto_IsLastWaypoint(s_activeWaypointIdx));
        return true;
    }

    return false;
}

/* ==========================================================================
 * GOM CỤM ĐIỂM CHẠM 3 CẢM BIẾN TOF -> LƯU BIÊN TƯỜNG TRÊN BẢN ĐỒ & TÌM ĐƯỜNG MỚI
 * ========================================================================== */
static bool Auto_ProcessSensorHit(float hx, float hy, float rx, float ry, float rayDistMm, uint32_t now,
                                  float *outHitX, float *outHitY, float *outUx, float *outUy) {
    uint8_t writeIdx = 0U;
    for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
        if ((now - s_hitSamples[i].tick) <= CLUSTER_MAX_AGE_MS) {
            s_hitSamples[writeIdx++] = s_hitSamples[i];
        }
    }
    s_hitSampleCount = writeIdx;

    if (s_hitSampleCount >= CLUSTER_MAX_HITS) {
        memmove(&s_hitSamples[0], &s_hitSamples[1], sizeof(HitSample) * (CLUSTER_MAX_HITS - 1U));
        s_hitSampleCount = CLUSTER_MAX_HITS - 1U;
    }
    s_hitSamples[s_hitSampleCount++] = (HitSample){ hx, hy, rx, ry, now };

    uint8_t clusterCount = 0U;
    float sumX = 0.0f, sumY = 0.0f, sumRx = 0.0f, sumRy = 0.0f;
    const float threshSq = CLUSTER_DIST_THRESH_MM * CLUSTER_DIST_THRESH_MM;
    for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
        float dx = s_hitSamples[i].x_mm - hx;
        float dy = s_hitSamples[i].y_mm - hy;
        if ((dx * dx + dy * dy) <= threshSq) {
            clusterCount++;
            sumX  += s_hitSamples[i].x_mm;
            sumY  += s_hitSamples[i].y_mm;
            sumRx += s_hitSamples[i].rx;
            sumRy += s_hitSamples[i].ry;
        }
    }

    uint8_t minConfirm = (rayDistMm <= CLUSTER_VERY_NEAR_DIST_MM) ? CLUSTER_MIN_CONFIRM_VNEAR :
                         ((rayDistMm <= CLUSTER_NEAR_DIST_MM) ? CLUSTER_MIN_CONFIRM_NEAR : CLUSTER_MIN_CONFIRM_FAR);
    if (clusterCount >= minConfirm) {
        *outHitX = sumX / (float)clusterCount;
        *outHitY = sumY / (float)clusterCount;
        float rLen = sqrtf(sumRx * sumRx + sumRy * sumRy);
        if (rLen > 0.01f) {
            *outUx = sumRx / rLen;
            *outUy = sumRy / rLen;
        } else {
            *outUx = rx;
            *outUy = ry;
        }

        uint8_t keepIdx = 0U;
        for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
            float dx = s_hitSamples[i].x_mm - hx;
            float dy = s_hitSamples[i].y_mm - hy;
            if ((dx * dx + dy * dy) > threshSq) {
                s_hitSamples[keepIdx++] = s_hitSamples[i];
            }
        }
        s_hitSampleCount = keepIdx;
        return true;
    }
    return false;
}

static void Auto_BeginReroute(void) {
    Motion_Stop();
    Motion_SetTightSpace(true);
    s_avoidStartTick = HAL_GetTick();
    s_hitSampleCount = 0U;
    s_avoidPhase = AVOID_PHASE_SETTLE;
    s_inflateNextRow = 0U;
    s_numCandidates = 0U;
    s_curCandidateIdx = 0U;
    s_curSearchInitialized = false;
    s_bestCandCount = 0U;
    s_bestCandCost = 1e9f;
    s_autoState = AUTO_STATE_AVOIDING;
}

/* ==========================================================================
 * GIAO DIỆN ĐIỀU KHIỂN TỰ ĐỘNG & XỬ LÝ NÚT NHẤN TỐI GIẢN
 * ========================================================================== */
void Auto_Init(void) {
    s_autoState = AUTO_STATE_IDLE;
    s_startTick = 0U;
    s_numActiveWaypoints = 0U;
    s_numCorners = 0U;
    Auto_ClearDynamicWalls();
    Auto_ResetActiveSession();
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
    s_sensorsFresh = (leftValid || midValid || rightValid);
}

void Auto_Stop(void) {
    s_autoState = AUTO_STATE_IDLE;
    s_numActiveWaypoints = 0U;
    s_numCorners = 0U;
    Auto_ResetActiveSession();
    Motion_Stop();
}

void Auto_OnButton1(void) {
    if (Auto_IsBusyState()) return;

    Auto_Stop();
    Odometry_Reset();
    Auto_ClearDynamicWalls();
    Auto_ResetActiveSession();
    Map_Init();

    s_numActiveWaypoints = NUM_UNIVERSAL_WAYPOINTS;
    memcpy(s_activeWaypoints, s_universalWaypoints, sizeof(s_universalWaypoints));

    s_startTick = HAL_GetTick();
    s_autoState = AUTO_STATE_STARTING;
    Led_SetPattern(1U, 0U, 0U, 0U, 0U);
    Led_SetPattern(3U, 0U, 0U, 0U, 0U);
    Led_SetPattern(2U, 1U, 100U, 100U, 100U);
}

void Auto_OnButton2(void) {
    Auto_Stop();
    Auto_ClearAllLeds();
}

/* ==========================================================================
 * VÒNG LẶP ĐIỀU KHIỂN TỰ ĐỘNG 100 HZ
 * ========================================================================== */
void Auto_Update(float dt_s) {
    (void)dt_s;
    uint32_t now = HAL_GetTick();

    /* 1. Đếm ngược 1 giây trước khi xuất phát */
    if (s_autoState == AUTO_STATE_STARTING) {
        if ((now - s_startTick) >= START_COUNTDOWN_MS) {
            Led_SetPattern(2U, 1U, 1000U, 0U, 0U);
            Odometry_Reset();
            s_activeWaypointIdx = 0U;
            s_autoState = AUTO_STATE_RUNNING;

            Auto_BuildPlan(s_activeWaypoints, s_numActiveWaypoints, ODOMETRY_INITIAL_X_MM, ODOMETRY_INITIAL_Y_MM);
            float targetX = 0.0f, targetY = 0.0f;
            Auto_GetAimTarget(0U, &targetX, &targetY);
            Motion_NavToPoint(targetX, targetY, Auto_IsLastWaypoint(0U));
        }
        return;
    }

    /* 1b. Chạy một lát cắt lập đường né A* không chặn (hỗ trợ cả host test chỉ gọi Auto_Update) */
    if (s_autoState == AUTO_STATE_AVOIDING) {
        (void)Auto_StepPlannerSlice();
        return;
    }

    if (s_autoState != AUTO_STATE_RUNNING) return;

    Odometry_State odom;
    if (Odometry_GetState(&odom) != HAL_OK) return;

    /* 2. Kiểm tra cán qua vạch đích (X >= 1200 mm, Y <= -80 mm) */
    if (odom.x_mm >= 1200.0f && odom.y_mm <= -80.0f && Auto_IsLastWaypoint(s_activeWaypointIdx)) {
        Motion_Stop();
        s_autoState = AUTO_STATE_COMPLETED;
        Led_SetPattern(1U, 1U, 1000U, 0U, 0U);
        Led_SetPattern(2U, 1U, 1000U, 0U, 0U);
        Led_SetPattern(3U, 1U, 1000U, 0U, 0U);
        return;
    }

    /* Xóa bộ đệm gom mẫu ToF khi xe đang quay gắt tại chỗ để chống nhòe góc */
    if (Motion_IsTurningInPlace()) {
        s_hitSampleCount = 0U;
    }

    /* 3. Phát hiện biên tường bất ngờ bằng cả 3 ToF -> lưu ô tường lên g_map_grid & tìm đường mới nếu chắn lộ trình */
    bool inFinishZone = (odom.x_mm >= 1200.0f && odom.y_mm <= FINISH_ZONE_SAFE_Y_MM);
    if (!inFinishZone &&
        odom.x_mm >= TOF_ACTIVE_CAR_X_MIN_MM &&
        odom.y_mm >= -60.0f && odom.y_mm <= 1150.0f &&
        s_activeWaypointIdx < s_numActiveWaypoints) {

        if (s_sensorsFresh) {
            s_sensorsFresh = false;
            if (!Motion_IsTurningInPlace()) {
                bool calmAngular = (!Motion_IsArcActive() &&
                                    fabsf(odom.angular_rad_s) <= TOF_MAX_SCAN_ANG_RAD_S);
                typedef void (*PoseFn)(float, float, float, float, float *, float *);
                const struct {
                    float  dist;
                    bool   valid;
                    float  maxRange;
                    float  rayOffsetDeg;
                    PoseFn pose;
                } rays[3] = {
                    { s_midDistanceMm,   s_midValid,   MID_DETECT_MAX_MM,    0.0f, midpose   },
                    { s_leftDistanceMm,  s_leftValid,  SIDE_DETECT_MAX_MM,  45.0f, leftpose  },
                    { s_rightDistanceMm, s_rightValid, SIDE_DETECT_MAX_MM, -45.0f, rightpose }
                };

                bool needReroute = false;
                for (uint8_t r = 0U; r < 3U; r++) {
                    float effMaxRange = calmAngular ? rays[r].maxRange : 145.0f;
                    if (!rays[r].valid ||
                        rays[r].dist < OBSTACLE_DETECT_MIN_MM ||
                        rays[r].dist > effMaxRange) {
                        continue;
                    }

                    float px = 0.0f, py = 0.0f;
                    rays[r].pose(rays[r].dist, odom.x_mm, odom.y_mm, odom.heading_deg, &px, &py);

                    float rangeSlackMm = (rays[r].dist > 150.0f) ? ((rays[r].dist - 150.0f) * 0.18f) : 0.0f;
                    if (px < DETECT_ZONE_X_MIN_MM || Auto_IsKnownStaticObstacle(px, py, rangeSlackMm)) {
                        continue;
                    }

                    float rayRad = (odom.heading_deg + rays[r].rayOffsetDeg) * DEG_TO_RAD_F;
                    float rx = cosf(rayRad), ry = sinf(rayRad);
                    float hx = 0.0f, hy = 0.0f, ux = rx, uy = ry;

                    if (Auto_ProcessSensorHit(px, py, rx, ry, rays[r].dist, now, &hx, &hy, &ux, &uy)) {
                        if (hx < DETECT_ZONE_X_MIN_MM || Auto_IsKnownStaticObstacle(hx, hy, rangeSlackMm)) {
                            continue;
                        }

                        /* Mô hình hóa thuần túy đoạn biên tường mỏng tại điểm chạm (0 mm chiều sâu, KHÔNG dựng khối) */
                        const float tx = -uy;
                        const float ty =  ux;
                        const float offsets[3] = { 0.0f, -WALL_EDGE_TANGENT_HALF_MM, WALL_EDGE_TANGENT_HALF_MM };

                        for (uint8_t k = 0U; k < 3U; k++) {
                            float wx = hx + offsets[k] * tx;
                            float wy = hy + offsets[k] * ty;
                            if (wx < DETECT_ZONE_X_MIN_MM || Auto_IsKnownStaticObstacle(wx, wy, rangeSlackMm)) {
                                continue;
                            }

                            bool isNewPt = false;
                            if (!Auto_RecordWallPoint(wx, wy, &isNewPt)) {
                                continue;
                            }

                            float checkClearMm = isNewPt ? OBS_BLOCK_CLEAR_MM : KNOWN_WALL_CLOSE_CLEAR_MM;
                            if (!isNewPt && rays[r].dist > 150.0f) {
                                continue;
                            }
                            if (Auto_IsWallPointBlockingRoute(wx, wy, odom.x_mm, odom.y_mm, odom.heading_deg,
                                                              s_activeWaypoints, s_activeWaypointIdx,
                                                              s_numActiveWaypoints, checkClearMm)) {
                                needReroute = true;
                            }
                        }
                    }
                }

                if (needReroute) {
                    Auto_BeginReroute();
                    return;
                }
            }
        }
    }

    /* 4. Chuyển tiếp góc cua động học ICR hoặc quay tại chỗ */
    if (Motion_IsArcActive() || Motion_IsTurningInPlace()) return;

    if (s_activeWaypointIdx < s_numCorners) {
        AutoCorner *cn = &s_corners[s_activeWaypointIdx];

        if (cn->type == CORNER_ARC) {
            float d_along = (odom.x_mm - cn->Tin.x_mm) * cn->u1_x + (odom.y_mm - cn->Tin.y_mm) * cn->u1_y;
            float d_cross = fabsf((odom.x_mm - cn->Tin.x_mm) * cn->u1_y - (odom.y_mm - cn->Tin.y_mm) * cn->u1_x);
            float angDev  = fabsf(Auto_NormalizeAngleDiff(cn->th1_deg, odom.heading_deg));

            if (d_along >= -60.0f && d_along < 0.0f && d_cross <= NAV_TANGENT_TOL_LATERAL_MM && angDev <= 25.0f) {
                Motion_SetTargetHeading(cn->th1_deg);
            }

            if (d_along >= -NAV_TANGENT_LEAD_DIST_MM && d_along <= NAV_TANGENT_PASS_DIST_MM &&
                d_cross <= NAV_TANGENT_TOL_LATERAL_MM && angDev <= NAV_TANGENT_TOL_ANG_DEG) {
                s_activeWaypointIdx++;
                float nextAimX = 0.0f, nextAimY = 0.0f;
                Auto_GetAimTarget(s_activeWaypointIdx, &nextAimX, &nextAimY);
                Motion_StartArcICR(cn->R, cn->dir, cn->th2_deg, cn->Tout.x_mm, cn->Tout.y_mm,
                                   cn->u2_x, cn->u2_y, nextAimX, nextAimY, Auto_IsLastWaypoint(s_activeWaypointIdx));
                return;
            }

            float projTout   = (odom.x_mm - cn->Tout.x_mm) * cn->u2_x + (odom.y_mm - cn->Tout.y_mm) * cn->u2_y;
            float dCrossTout = fabsf((odom.x_mm - cn->Tout.x_mm) * cn->u2_y - (odom.y_mm - cn->Tout.y_mm) * cn->u2_x);
            float projWp     = (odom.x_mm - cn->wp.x_mm) * cn->u1_x + (odom.y_mm - cn->wp.y_mm) * cn->u1_y;

            if (d_along > NAV_TANGENT_PASS_DIST_MM || projWp > 20.0f ||
                (d_along >= -20.0f && projTout >= 0.0f && dCrossTout <= 40.0f)) {
                s_activeWaypointIdx++;
                float nextAimX = 0.0f, nextAimY = 0.0f;
                Auto_GetAimTarget(s_activeWaypointIdx, &nextAimX, &nextAimY);
                Motion_NavToPoint(nextAimX, nextAimY, Auto_IsLastWaypoint(s_activeWaypointIdx));
                return;
            }
        } else {
            float dx = cn->wp.x_mm - odom.x_mm, dy = cn->wp.y_mm - odom.y_mm;
            float rem = sqrtf(dx * dx + dy * dy);
            float proj = (odom.x_mm - cn->wp.x_mm) * cn->u1_x + (odom.y_mm - cn->wp.y_mm) * cn->u1_y;
            float passRadius = (cn->type == CORNER_SPIN) ? 18.0f : 25.0f;

            if (rem <= passRadius || proj >= NAV_WAYPOINT_PASS_DIST_MM) {
                s_activeWaypointIdx++;
                float nextAimX = 0.0f, nextAimY = 0.0f;
                Auto_GetAimTarget(s_activeWaypointIdx, &nextAimX, &nextAimY);
                if (cn->type == CORNER_SPIN) {
                    Motion_NavToPointEx(nextAimX, nextAimY, Auto_IsLastWaypoint(s_activeWaypointIdx), true);
                } else {
                    Motion_NavToPoint(nextAimX, nextAimY, Auto_IsLastWaypoint(s_activeWaypointIdx));
                }
                return;
            }
        }
    }

    /* 5. Kiểm tra hoàn thành mốc tọa độ cuối / dự phòng */
    int16_t statusWord = Motion_GetFeedbackStatusWord();
    uint8_t activeCmd  = (uint8_t)((statusWord >> 8U) & 0xFFU);
    uint8_t cmdStatus  = (uint8_t)(statusWord & 0xFFU);

    if (activeCmd == CMD_NAV_TO_POINT && cmdStatus == STATUS_COMPLETED) {
        s_activeWaypointIdx++;
        if (s_activeWaypointIdx < s_numActiveWaypoints) {
            float nextAimX = 0.0f, nextAimY = 0.0f;
            Auto_GetAimTarget(s_activeWaypointIdx, &nextAimX, &nextAimY);
            Motion_NavToPoint(nextAimX, nextAimY, Auto_IsLastWaypoint(s_activeWaypointIdx));
        } else {
            Motion_Stop();
            s_autoState = AUTO_STATE_COMPLETED;
            Led_SetPattern(2U, 1U, 1000U, 0U, 0U);
        }
    } else if (activeCmd == CMD_NAV_TO_POINT && cmdStatus == STATUS_ERROR) {
        Motion_Stop();
        if (odom.x_mm >= 1200.0f && odom.y_mm <= 60.0f) {
            s_autoState = AUTO_STATE_COMPLETED;
            Led_SetPattern(2U, 1U, 1000U, 0U, 0U);
        } else {
            s_autoState = AUTO_STATE_ERROR;
            Led_SetPattern(1U, 1U, 80U, 80U, 80U);
        }
    }
}
