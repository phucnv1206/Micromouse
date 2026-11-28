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

/* Cấu hình A* chia lát cắt, khoảng hở an toàn và biên tường bất ngờ 3 ToF */
#define HEAP_MAX_SIZE             (MAP_TOTAL_CELLS / 2U)
#define GOAL_FINISH_Y_MM          -100.0f
#define FINISH_LINE_COL_MIN       42U
#define FINISH_LINE_COL_MAX       55U

#define HARD_OBS_CLEAR_MM         54.0f
#define HARD_WALL_CLEAR_MM        70.0f
#define LOS_SAFE_CLEAR_MM         66.0f
#define ARC_SAFE_CLEAR_MM         54.0f

#define CLUSTER_MAX_HITS          24U
#define MAX_DYN_WALL_PTS          96U
#define WALL_PT_HALF_SIZE_MM      15.0f
#define ROUTE_BLOCK_LOOKAHEAD_MM  280.0f

#define INFLATE_ROWS_PER_SLICE    8U
#define ASTAR_NODES_PER_SLICE     80U
#define MAX_PLAN_CANDIDATES       4U

#define DEG_TO_RAD_F              (3.141592653589793f / 180.0f)
#define RAD_TO_DEG_F              (180.0f / 3.141592653589793f)

enum { RAY_MID = 0, RAY_LEFT = 1, RAY_RIGHT = 2 };

typedef enum { CORNER_NONE = 0, CORNER_ARC, CORNER_SPIN } AutoCornerType;

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
    AutoWaypoint wp, Tin, Tout;
    float R, dth_deg, th1_deg, th2_deg;
    float u1_x, u1_y, u2_x, u2_y;
    int8_t dir;
} AutoCorner;

typedef struct {
    float x_mm, y_mm, rx, ry;
    uint32_t tick;
} HitSample;

typedef struct {
    float goalX, goalY;
    uint8_t goalCol, goalRow, rejoinWpIdx, maxDetour;
    bool isFinishLine;
} PlanCandidate;

static const AutoWaypoint s_universalWaypoints[] = {
    {  425.0f,  250.0f },
    {  430.0f,  740.0f },
    {  590.0f,  930.0f },
    { 1285.0f,  680.0f },
    { 1285.0f, GOAL_FINISH_Y_MM }
};
#define NUM_UNIVERSAL_WAYPOINTS (sizeof(s_universalWaypoints) / sizeof(s_universalWaypoints[0]))

static const int8_t kDir8[8][2] = {
    { 0, 1}, { 0, -1}, { 1, 0}, {-1, 0},
    { 1, 1}, { 1, -1}, {-1, 1}, {-1, -1}
};

static const struct {
    float maxRange;
    float offsetDeg;
    void (*pose)(float, float, float, float, float *, float *);
} kRays[3] = {
    { 245.0f,   0.0f, midpose   },
    { 215.0f,  45.0f, leftpose  },
    { 215.0f, -45.0f, rightpose }
};

static AutoState     s_autoState = AUTO_STATE_IDLE;
static uint32_t      s_startTick = 0U, s_avoidStartTick = 0U, s_backoffStartTick = 0U;
static AutoWaypoint  s_activeWaypoints[AUTO_MAX_WAYPOINTS];
static uint8_t       s_numActiveWaypoints = 0U, s_activeWaypointIdx = 0U;
static AutoCorner    s_corners[AUTO_MAX_WAYPOINTS];
static uint8_t       s_numCorners = 0U;

static float         s_tofDist[3];
static bool          s_tofValid[3];
static bool          s_sensorsFresh = false;

static AutoWaypoint  s_dynWallPts[MAX_DYN_WALL_PTS];
static uint8_t       s_numDynWallPts = 0U;
static HitSample     s_hitSamples[CLUSTER_MAX_HITS];
static uint8_t       s_hitSampleCount = 0U;

static AvoidPhase    s_avoidPhase = AVOID_PHASE_NONE;
static float         s_backoffOriginX = 0.0f, s_backoffOriginY = 0.0f;
static float         s_planStartX = 0.0f, s_planStartY = 0.0f, s_planStartHeadingDeg = 0.0f;
static uint8_t       s_planStartCol = 0U, s_planStartRow = 0U, s_inflateNextRow = 0U;

static PlanCandidate s_candidates[MAX_PLAN_CANDIDATES];
static uint8_t       s_numCandidates = 0U, s_curCandidateIdx = 0U, s_bestCandCount = 0U;
static bool          s_curSearchInitialized = false;
static AutoWaypoint  s_bestRoute[AUTO_MAX_WAYPOINTS];
static float         s_bestCandCost = 1e9f;

static uint8_t       s_inflated[MAP_ROWS][MAP_COLS];
static uint16_t      s_gScore[MAP_TOTAL_CELLS];
static uint16_t      s_cameFrom[MAP_TOTAL_CELLS];
static uint8_t       s_closed[MAP_TOTAL_CELLS / 8U];
static uint16_t      s_heapNodes[HEAP_MAX_SIZE];
static uint16_t      s_heapKeys[HEAP_MAX_SIZE];
static uint16_t      s_heapSize = 0U;
static uint16_t      s_rawGridPath[256];

/* ================= TIỆN ÍCH CƠ BẢN & HÌNH HỌC AN TOÀN ================= */
static inline float GridCenterMm(uint8_t idx) { return ((float)idx + 0.5f) * MAP_CELL_SIZE_MM; }
static inline float DistSq(float x0, float y0, float x1, float y1) {
    float dx = x1 - x0, dy = y1 - y0;
    return dx * dx + dy * dy;
}
static inline float DistMm(float x0, float y0, float x1, float y1) { return sqrtf(DistSq(x0, y0, x1, y1)); }

static inline bool Auto_IsBusyState(void) {
    return s_autoState == AUTO_STATE_STARTING || s_autoState == AUTO_STATE_RUNNING || s_autoState == AUTO_STATE_AVOIDING;
}
static inline bool Auto_IsLastWaypoint(uint8_t idx) {
    return idx >= ((s_numActiveWaypoints > 1U) ? (s_numActiveWaypoints - 1U) : 0U);
}
static void Auto_SetAllLeds(uint16_t count, uint16_t onMs) {
    for (uint8_t i = 1U; i <= 3U; i++) Led_SetPattern(i, count, onMs, 0U, 0U);
}

static void Auto_ResetPlannerState(void) {
    s_inflateNextRow = 0U;
    s_numCandidates = 0U;
    s_curCandidateIdx = 0U;
    s_curSearchInitialized = false;
    s_bestCandCount = 0U;
    s_bestCandCost = 1e9f;
}

static void Auto_ResetSession(void) {
    s_autoState = AUTO_STATE_IDLE;
    s_numActiveWaypoints = 0U;
    s_numCorners = 0U;
    s_activeWaypointIdx = 0U;
    s_avoidStartTick = 0U;
    s_backoffStartTick = 0U;
    s_avoidPhase = AVOID_PHASE_NONE;
    s_hitSampleCount = 0U;
    s_sensorsFresh = false;
    Auto_ResetPlannerState();
}

static float Auto_NormalizeAngleDiff(float target_deg, float current_deg) {
    float d = target_deg - current_deg;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static void Auto_ClampToGrid(float x_mm, float y_mm, uint8_t *col, uint8_t *row) {
    int c = (int)(fminf(fmaxf(x_mm, 0.0f), MAP_WIDTH_MM - 1.0f) / MAP_CELL_SIZE_MM);
    int r = (int)(fminf(fmaxf(y_mm, 0.0f), MAP_HEIGHT_MM - 1.0f) / MAP_CELL_SIZE_MM);
    *col = (uint8_t)((c < 0) ? 0 : ((c >= (int)MAP_COLS) ? ((int)MAP_COLS - 1) : c));
    *row = (uint8_t)((r < 0) ? 0 : ((r >= (int)MAP_ROWS) ? ((int)MAP_ROWS - 1) : r));
}

static float Auto_DistSqToBox(float px, float py, float xmin, float xmax, float ymin, float ymax) {
    float dx = (px < xmin) ? (xmin - px) : ((px > xmax) ? (px - xmax) : 0.0f);
    float dy = (py < ymin) ? (ymin - py) : ((py > ymax) ? (py - ymax) : 0.0f);
    return dx * dx + dy * dy;
}

static float Auto_MinPillarDistSq(float px, float py) {
    float best = 1e12f;
    for (uint8_t i = 0U; i < STATIC_OBSTACLE_COUNT; i++) {
        float cx = g_static_obstacles[i].x_mm, cy = g_static_obstacles[i].y_mm;
        float dSq = Auto_DistSqToBox(px, py, cx - 50.0f, cx + 50.0f, cy - 50.0f, cy + 50.0f);
        if (dSq < best) best = dSq;
    }
    return best;
}

static bool Auto_IsKnownStaticObstacle(float x_mm, float y_mm, float extraMarginMm) {
    float wallMargin = 95.0f + extraMarginMm * 0.5f;
    if (x_mm <= wallMargin || x_mm >= (MAP_WIDTH_MM - wallMargin) ||
        y_mm <= -60.0f || y_mm >= (MAP_HEIGHT_MM - wallMargin)) {
        return true;
    }
    if (x_mm >= (540.0f - extraMarginMm) && x_mm <= (1262.0f + extraMarginMm * 0.3f) &&
        y_mm >= -20.0f && y_mm <= (652.0f + extraMarginMm * 0.4f)) {
        return true;
    }
    float pm = 57.0f + extraMarginMm * 0.5f;
    return Auto_MinPillarDistSq(x_mm, y_mm) <= (pm * pm);
}

static bool Auto_RecordWallPoint(float wx_mm, float wy_mm, bool *outIsNew) {
    uint8_t wc = 0U, wr = 0U;
    if (!Map_MmToGrid(wx_mm, wy_mm, &wc, &wr)) return false;
    g_map_grid[wr][wc] = MAP_VAL_OBSTACLE;

    for (uint8_t i = 0U; i < s_numDynWallPts; i++) {
        if (DistSq(s_dynWallPts[i].x_mm, s_dynWallPts[i].y_mm, wx_mm, wy_mm) <= (15.0f * 15.0f)) {
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

static void Auto_ComputeClearanceSqMm(float px, float py, float *outObsClearSqMm, float *outWallClearMm) {
    float wallMin = fminf(px, fminf(MAP_WIDTH_MM - px, MAP_HEIGHT_MM - py));
    if (px < 600.0f && (px < 210.0f || px > 390.0f) && py < wallMin) wallMin = py;

    float minObsSq = fminf(Auto_DistSqToBox(px, py, 600.0f, 1200.0f, -150.0f, 600.0f),
                           Auto_MinPillarDistSq(px, py));
    for (uint8_t i = 0U; i < s_numDynWallPts; i++) {
        float wx = s_dynWallPts[i].x_mm, wy = s_dynWallPts[i].y_mm;
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
    float dx = x1 - x0, dy = y1 - y0, segLen = sqrtf(dx * dx + dy * dy);
    float startObs = 0.0f, startWall = 0.0f;
    Auto_ComputeClearanceMm(x0, y0, &startObs, &startWall);

    int steps = (int)ceilf(segLen / 12.0f);
    if (steps < 1) steps = 1;

    for (int s = 1; s <= steps; s++) {
        float t = (float)s / (float)steps, obsC = 0.0f, wallC = 0.0f;
        Auto_ComputeClearanceMm(x0 + t * dx, y0 + t * dy, &obsC, &wallC);
        if ((t * segLen) < 65.0f && (startObs < reqObsClearMm || startWall < reqWallClearMm)) {
            if (obsC  < fminf(reqObsClearMm,  fmaxf(45.0f, startObs  - 1.0f)) ||
                wallC < fminf(reqWallClearMm, fmaxf(48.0f, startWall - 1.0f))) return false;
        } else if (obsC < reqObsClearMm || wallC < reqWallClearMm) {
            return false;
        }
    }
    return true;
}

static bool Auto_IsWallPtBlockingSeg(float wx, float wy, float x0, float y0, float x1, float y1,
                                     float clearMm, float maxCheckLenMm, bool firstSeg, float headingDeg) {
    float dx = x1 - x0, dy = y1 - y0, segLen = sqrtf(dx * dx + dy * dy);
    if (segLen < 1.0f || maxCheckLenMm <= 0.0f) return false;

    float ux = dx / segLen, uy = dy / segLen;
    if (firstSeg && ((wx - x0) * ux + (wy - y0) * uy) <= 0.0f) return false;

    float fwdX = firstSeg ? cosf(headingDeg * DEG_TO_RAD_F) : 0.0f;
    float fwdY = firstSeg ? sinf(headingDeg * DEG_TO_RAD_F) : 0.0f;
    float xMin = wx - WALL_PT_HALF_SIZE_MM, xMax = wx + WALL_PT_HALF_SIZE_MM;
    float yMin = wy - WALL_PT_HALF_SIZE_MM, yMax = wy + WALL_PT_HALF_SIZE_MM;

    float checkLen = fminf(segLen, maxCheckLenMm);
    int steps = (int)ceilf(checkLen / 12.0f);
    if (steps < 1) steps = 1;

    float threshSq = clearMm * clearMm;
    float startDist = sqrtf(Auto_DistSqToBox(x0, y0, xMin, xMax, yMin, yMax));

    for (int s = 1; s <= steps; s++) {
        float sDist = checkLen * ((float)s / (float)steps);
        float px = x0 + sDist * ux, py = y0 + sDist * uy;
        if (firstSeg && ((px - x0) * fwdX + (py - y0) * fwdY) <= 0.0f) continue;

        float dSq = Auto_DistSqToBox(px, py, xMin, xMax, yMin, yMax);
        if (sDist < 65.0f && startDist < clearMm) {
            float minAllow = (startDist > 2.0f) ? (startDist - 2.0f) : 0.0f;
            if (dSq < (minAllow * minAllow)) return true;
        } else if (dSq < threshSq) {
            return true;
        }
    }
    return false;
}

static bool Auto_IsWallPointBlockingRoute(float wx, float wy, float startX, float startY, float headingDeg,
                                          const AutoWaypoint *wps, uint8_t firstIdx, uint8_t count, float clearMm) {
    float rem = ROUTE_BLOCK_LOOKAHEAD_MM, x0 = startX, y0 = startY;
    for (uint8_t k = firstIdx; k < count && rem > 0.0f; k++) {
        float x1 = wps[k].x_mm, y1 = wps[k].y_mm;
        if (Auto_IsWallPtBlockingSeg(wx, wy, x0, y0, x1, y1, clearMm, rem, k == firstIdx, headingDeg)) {
            return true;
        }
        rem -= DistMm(x0, y0, x1, y1);
        x0 = x1;
        y0 = y1;
    }
    return false;
}

/* ================= LẬP CUNG TRÒN TIẾP TUYẾN ICR ================= */
static bool Auto_CheckArcSafe(AutoWaypoint Tin, AutoWaypoint Tout, AutoWaypoint C, float R, int8_t dir) {
    float aIn = atan2f(Tin.y_mm - C.y_mm, Tin.x_mm - C.x_mm);
    float sweep = atan2f(Tout.y_mm - C.y_mm, Tout.x_mm - C.x_mm) - aIn;
    if (dir > 0 && sweep < 0.0f) sweep += 2.0f * 3.141592653589793f;
    if (dir < 0 && sweep > 0.0f) sweep -= 2.0f * 3.141592653589793f;

    const float reqDynArcSq = (ARC_SAFE_CLEAR_MM + WALL_PT_HALF_SIZE_MM + 18.0f) *
                              (ARC_SAFE_CLEAR_MM + WALL_PT_HALF_SIZE_MM + 18.0f);
    for (int i = 0; i <= 16; i++) {
        float a = aIn + sweep * ((float)i / 16.0f);
        float qx = C.x_mm + R * cosf(a), qy = C.y_mm + R * sinf(a);
        if (qy < -80.0f) return false;

        float obsClear = 0.0f, wallClear = 0.0f;
        Auto_ComputeClearanceMm(qx, qy, &obsClear, &wallClear);
        if (obsClear < ARC_SAFE_CLEAR_MM || wallClear < 65.0f) return false;

        for (uint8_t w = 0U; w < s_numDynWallPts; w++) {
            if (DistSq(qx, qy, s_dynWallPts[w].x_mm, s_dynWallPts[w].y_mm) < reqDynArcSq) return false;
        }
    }
    return true;
}

static void Auto_BuildPlan(const AutoWaypoint *wps, uint8_t numWps, float startX, float startY) {
    s_numCorners = 0U;
    if (numWps == 0U) return;

    AutoWaypoint pts[AUTO_MAX_WAYPOINTS + 1U];
    pts[0] = (AutoWaypoint){ startX, startY };
    uint8_t capped = (numWps > AUTO_MAX_WAYPOINTS) ? AUTO_MAX_WAYPOINTS : numWps;
    memcpy(&pts[1], wps, sizeof(AutoWaypoint) * capped);
    uint8_t totalPts = capped + 1U;

    for (uint8_t i = 1U; i < (totalPts - 1U); i++) {
        AutoWaypoint pPrev = pts[i - 1U], p = pts[i], pNext = pts[i + 1U];
        float v1_x = p.x_mm - pPrev.x_mm, v1_y = p.y_mm - pPrev.y_mm, L1 = sqrtf(v1_x * v1_x + v1_y * v1_y);
        float v2_x = pNext.x_mm - p.x_mm, v2_y = pNext.y_mm - p.y_mm, L2 = sqrtf(v2_x * v2_x + v2_y * v2_y);
        float u1_x = (L1 > 0.0f) ? (v1_x / L1) : 1.0f, u1_y = (L1 > 0.0f) ? (v1_y / L1) : 0.0f;
        float u2_x = (L2 > 0.0f) ? (v2_x / L2) : 1.0f, u2_y = (L2 > 0.0f) ? (v2_y / L2) : 0.0f;

        float th1 = atan2f(u1_y, u1_x) * RAD_TO_DEG_F;
        float th2 = atan2f(u2_y, u2_x) * RAD_TO_DEG_F;
        float dth = Auto_NormalizeAngleDiff(th2, th1);

        AutoCorner *cn = &s_corners[s_numCorners++];
        *cn = (AutoCorner){ .type = CORNER_NONE, .wp = p, .dth_deg = dth, .th1_deg = th1, .th2_deg = th2,
                            .u1_x = u1_x, .u1_y = u1_y, .u2_x = u2_x, .u2_y = u2_y };
        if (fabsf(dth) < 18.0f) continue;

        float tanHalf = tanf(fabsf(dth) * DEG_TO_RAD_F * 0.5f);
        float dMax = fminf((i == 1U) ? (L1 * 0.85f) : (L1 * 0.48f),
                           fminf((i == (totalPts - 2U)) ? (L2 * 0.85f) : (L2 * 0.48f), 120.0f));
        int8_t dir = (dth > 0.0f) ? 1 : -1;
        float n1_x = (dir > 0) ? -u1_y : u1_y, n1_y = (dir > 0) ? u1_x : -u1_x;

        bool arcFound = false;
        for (float R_try = NAV_ARC_RADIUS_NOMINAL_MM; R_try >= 25.0f; R_try -= 5.0f) {
            float R = R_try, d = R * tanHalf;
            if (d > dMax && tanHalf > 0.001f) { R = dMax / tanHalf; d = dMax; }
            if (R < 25.0f || d < 12.0f) break;

            AutoWaypoint Tin  = { p.x_mm - d * u1_x, p.y_mm - d * u1_y };
            AutoWaypoint Tout = { p.x_mm + d * u2_x, p.y_mm + d * u2_y };
            AutoWaypoint C    = { Tin.x_mm + R * n1_x, Tin.y_mm + R * n1_y };

            if (Auto_CheckArcSafe(Tin, Tout, C, R, dir)) {
                cn->type = CORNER_ARC;
                cn->Tin = Tin; cn->Tout = Tout; cn->R = R; cn->dir = dir;
                arcFound = true;
                break;
            }
        }
        if (!arcFound) cn->type = (fabsf(dth) < 30.0f) ? CORNER_NONE : CORNER_SPIN;
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

static void Auto_NavToIdx(uint8_t idx, bool forceSpin) {
    float x = 0.0f, y = 0.0f;
    Auto_GetAimTarget(idx, &x, &y);
    if (forceSpin) Motion_NavToPointEx(x, y, Auto_IsLastWaypoint(idx), true);
    else           Motion_NavToPoint(x, y, Auto_IsLastWaypoint(idx));
}

/* ================= MIN-HEAP, LOS & TÌM ĐƯỜNG A* CHIA LÁT CẮT ================= */
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
    if (--s_heapSize > 0U) {
        uint16_t lastNode = s_heapNodes[s_heapSize], lastKey = s_heapKeys[s_heapSize], i = 0U;
        while ((2U * i + 1U) < s_heapSize) {
            uint16_t l = 2U * i + 1U, r = l + 1U;
            uint16_t sm = (r < s_heapSize && s_heapKeys[r] < s_heapKeys[l]) ? r : l;
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
    int16_t dc = 0, dr = r;
    if (isFinishLine) {
        if (c < (int16_t)FINISH_LINE_COL_MIN)      dc = (int16_t)FINISH_LINE_COL_MIN - c;
        else if (c > (int16_t)FINISH_LINE_COL_MAX) dc = c - (int16_t)FINISH_LINE_COL_MAX;
    } else {
        dc = (goalC >= c) ? (goalC - c) : (c - goalC);
        dr = (goalR >= r) ? (goalR - r) : (r - goalR);
    }
    return (uint16_t)((dc > dr) ? (14 * dr + 10 * (dc - dr)) : (14 * dc + 10 * (dr - dc)));
}

static bool Auto_LineOfSight(int16_t c0, int16_t r0, int16_t c1, int16_t r1) {
    int16_t dc = (c1 > c0) ? (c1 - c0) : (c0 - c1), dr = (r1 > r0) ? (r1 - r0) : (r0 - r1);
    int16_t sc = (c0 < c1) ? 1 : -1, sr = (r0 < r1) ? 1 : -1, err = dc - dr;
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
        if (x != prevX && y != prevY &&
            (s_inflated[prevY][x] == MAP_VAL_OBSTACLE || s_inflated[y][prevX] == MAP_VAL_OBSTACLE)) {
            return false;
        }
    }
    return true;
}

static void Auto_PruneRouteByLOS(float startX, float startY, AutoWaypoint *wps, uint8_t *pCount, bool aggressiveLos) {
    uint8_t count = *pCount, i = 0U;
    while (count >= 2U && i < (count - 1U)) {
        AutoWaypoint pPrev = (i == 0U) ? (AutoWaypoint){ startX, startY } : wps[i - 1U];
        AutoWaypoint pCurr = wps[i], pNext = wps[i + 1U];

        float v1_x = pCurr.x_mm - pPrev.x_mm, v1_y = pCurr.y_mm - pPrev.y_mm;
        float d1 = sqrtf(v1_x * v1_x + v1_y * v1_y);
        float th1 = atan2f(v1_y, v1_x) * RAD_TO_DEG_F;
        float th2 = atan2f(pNext.y_mm - pCurr.y_mm, pNext.x_mm - pCurr.x_mm) * RAD_TO_DEG_F;
        float dth = fabsf(Auto_NormalizeAngleDiff(th2, th1));

        bool crossesFinishAngled = (pNext.y_mm < 0.0f && fabsf(pNext.x_mm - pPrev.x_mm) > 35.0f);
        if (!crossesFinishAngled && (aggressiveLos || dth < 10.0f || d1 < 60.0f) &&
            Auto_IsSegmentSafeMm(pPrev.x_mm, pPrev.y_mm, pNext.x_mm, pNext.y_mm, LOS_SAFE_CLEAR_MM, 74.0f)) {
            memmove(&wps[i], &wps[i + 1U], sizeof(AutoWaypoint) * (count - 1U - i));
            count--;
        } else {
            i++;
        }
    }
    *pCount = count;
}

static float Auto_EvaluateRouteCost(float startX, float startY, float startHeadingDeg,
                                    const AutoWaypoint *wps, uint8_t count) {
    if (count == 0U) return 1e9f;
    float cost = 0.0f, prevX = startX, prevY = startY, prevHeading = startHeadingDeg;
    for (uint8_t i = 0U; i < count; i++) {
        float dx = wps[i].x_mm - prevX, dy = wps[i].y_mm - prevY, segLen = sqrtf(dx * dx + dy * dy);
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
                                             const PlanCandidate *cand, AutoWaypoint *outWp, uint8_t maxCount) {
    uint16_t rawLen = 0U, curr = reachedGoalNode;
    while (curr != 0xFFFFU && rawLen < 256U) {
        s_rawGridPath[rawLen++] = curr;
        if (curr == startNode) break;
        curr = s_cameFrom[curr];
    }
    for (uint16_t i = 0U; i < (rawLen / 2U); i++) {
        uint16_t tmp = s_rawGridPath[i];
        s_rawGridPath[i] = s_rawGridPath[rawLen - 1U - i];
        s_rawGridPath[rawLen - 1U - i] = tmp;
    }

    uint8_t count = 0U;
    for (uint16_t anchor = 0U; rawLen > 1U && anchor < (rawLen - 1U); ) {
        uint16_t furthest = anchor + 1U;
        uint8_t ac = (uint8_t)(s_rawGridPath[anchor] % MAP_COLS), ar = (uint8_t)(s_rawGridPath[anchor] / MAP_COLS);
        float ax = (anchor == 0U) ? s_planStartX : GridCenterMm(ac);
        float ay = (anchor == 0U) ? s_planStartY : GridCenterMm(ar);

        for (uint16_t i = rawLen - 1U; i > (anchor + 1U); i--) {
            uint8_t tc = (uint8_t)(s_rawGridPath[i] % MAP_COLS), tr = (uint8_t)(s_rawGridPath[i] / MAP_COLS);
            if (Auto_LineOfSight(ac, ar, tc, tr) &&
                Auto_IsSegmentSafeMm(ax, ay, GridCenterMm(tc), GridCenterMm(tr), LOS_SAFE_CLEAR_MM, 74.0f)) {
                furthest = i;
                break;
            }
        }
        if (count < maxCount) {
            outWp[count++] = (AutoWaypoint){
                GridCenterMm((uint8_t)(s_rawGridPath[furthest] % MAP_COLS)),
                GridCenterMm((uint8_t)(s_rawGridPath[furthest] / MAP_COLS))
            };
        }
        anchor = furthest;
    }

    if (count == 0U && rawLen == 1U && maxCount > 0U) {
        outWp[count++] = (AutoWaypoint){ cand->goalX, cand->goalY };
    }

    if (count > 0U) {
        if (cand->isFinishLine) {
            float finalX = GridCenterMm((uint8_t)(reachedGoalNode % MAP_COLS));
            float prevX  = (count >= 2U) ? outWp[count - 2U].x_mm : s_planStartX;
            float prevY  = (count >= 2U) ? outWp[count - 2U].y_mm : s_planStartY;

            if (fabsf(finalX - prevX) <= 35.0f &&
                Auto_IsSegmentSafeMm(prevX, prevY, finalX, GOAL_FINISH_Y_MM, HARD_OBS_CLEAR_MM, HARD_WALL_CLEAR_MM)) {
                outWp[count - 1U] = (AutoWaypoint){ finalX, GOAL_FINISH_Y_MM };
            } else {
                outWp[count - 1U] = (AutoWaypoint){ finalX, 45.0f };
                if (count < maxCount) outWp[count++] = (AutoWaypoint){ finalX, GOAL_FINISH_Y_MM };
                else                  outWp[count - 1U].y_mm = GOAL_FINISH_Y_MM;
            }
        } else {
            outWp[count - 1U] = (AutoWaypoint){ cand->goalX, cand->goalY };
        }
    }

    Auto_PruneRouteByLOS(s_planStartX, s_planStartY, outWp, &count, false);
    return count;
}

static void Auto_PreparePlannerCandidates(void) {
    Auto_ResetPlannerState();

    if (s_planStartX < 1200.0f &&
        s_activeWaypointIdx < ((s_numActiveWaypoints > 2U) ? (s_numActiveWaypoints - 2U) : 0U)) {
        uint8_t maxEnd = s_activeWaypointIdx + 3U;
        if (maxEnd > s_numActiveWaypoints) maxEnd = s_numActiveWaypoints;

        for (uint8_t idx = s_activeWaypointIdx; idx < maxEnd && s_numCandidates < (MAX_PLAN_CANDIDATES - 1U); idx++) {
            float tx = s_activeWaypoints[idx].x_mm, ty = s_activeWaypoints[idx].y_mm;
            if (idx >= (s_numActiveWaypoints - 1U) || (ty <= 0.0f && tx >= 1200.0f)) continue;

            uint8_t gc = 0U, gr = 0U;
            Auto_ClampToGrid(tx, ty, &gc, &gr);
            if (s_inflated[gr][gc] >= 60U) continue;

            uint8_t tailLen = (uint8_t)(s_numActiveWaypoints - (idx + 1U));
            s_candidates[s_numCandidates++] = (PlanCandidate){
                .goalX = tx, .goalY = ty, .goalCol = gc, .goalRow = gr, .rejoinWpIdx = idx,
                .maxDetour = (tailLen < AUTO_MAX_WAYPOINTS) ? (uint8_t)(AUTO_MAX_WAYPOINTS - tailLen) : 4U,
                .isFinishLine = false
            };
        }
    }

    uint8_t finCol = 43U, finRow = 0U;
    Auto_ClampToGrid(1305.0f, GOAL_FINISH_Y_MM, &finCol, &finRow);
    s_candidates[s_numCandidates++] = (PlanCandidate){
        .goalX = 1305.0f, .goalY = GOAL_FINISH_Y_MM, .goalCol = finCol, .goalRow = finRow,
        .rejoinWpIdx = 0U, .maxDetour = AUTO_MAX_WAYPOINTS, .isFinishLine = true
    };
}

static void Auto_AdvanceCandidate(void) {
    s_curCandidateIdx++;
    s_curSearchInitialized = false;
    if (s_curCandidateIdx >= s_numCandidates) s_avoidPhase = AVOID_PHASE_APPLY;
}

bool Auto_StepPlannerSlice(void) {
    if (s_autoState != AUTO_STATE_AVOIDING || s_avoidPhase == AVOID_PHASE_NONE) return false;

    switch (s_avoidPhase) {
    case AVOID_PHASE_SETTLE:
    case AVOID_PHASE_POST_BACKOFF_SETTLE: {
        Odometry_State odom;
        if (Odometry_GetState(&odom) != HAL_OK) return false;
        uint32_t now = HAL_GetTick();

        if (s_avoidPhase == AVOID_PHASE_SETTLE) {
            float curObsC = 0.0f, curWallC = 0.0f;
            Auto_ComputeClearanceMm(odom.x_mm, odom.y_mm, &curObsC, &curWallC);
            if (curObsC < 64.0f || (s_tofValid[RAY_MID] && s_tofDist[RAY_MID] <= 75.0f)) {
                s_backoffStartTick = now;
                s_backoffOriginX = odom.x_mm;
                s_backoffOriginY = odom.y_mm;
                RunMotorLeft(-42);
                RunMotorRight(-42);
                s_avoidPhase = AVOID_PHASE_BACKOFF;
                return true;
            }
        }

        uint32_t maxWaitMs = (s_avoidPhase == AVOID_PHASE_SETTLE) ? 180U : 120U;
        bool settled = (fabsf(odom.linear_mm_s) <= 40.0f && fabsf(odom.angular_rad_s) <= 0.35f);
        if (!settled && (now - s_avoidStartTick) < maxWaitMs) return false;

        s_planStartX = odom.x_mm;
        s_planStartY = odom.y_mm;
        s_planStartHeadingDeg = odom.heading_deg;
        Auto_ClampToGrid(s_planStartX, s_planStartY, &s_planStartCol, &s_planStartRow);
        s_inflateNextRow = 0U;
        s_avoidPhase = AVOID_PHASE_INFLATE;
        return true;
    }

    case AVOID_PHASE_BACKOFF: {
        Odometry_State odom;
        if (Odometry_GetState(&odom) != HAL_OK) return false;
        uint32_t now = HAL_GetTick();

        float backedSq = DistSq(s_backoffOriginX, s_backoffOriginY, odom.x_mm, odom.y_mm);
        float curObsC = 0.0f, curWallC = 0.0f, rearObsC = 0.0f, rearWallC = 0.0f;
        Auto_ComputeClearanceMm(odom.x_mm, odom.y_mm, &curObsC, &curWallC);
        float headRad = odom.heading_deg * DEG_TO_RAD_F;
        Auto_ComputeClearanceMm(odom.x_mm - 45.0f * cosf(headRad),
                                odom.y_mm - 45.0f * sinf(headRad), &rearObsC, &rearWallC);

        if (!((curObsC >= 66.0f && backedSq >= (42.0f * 42.0f)) || backedSq >= (60.0f * 60.0f) ||
              rearObsC < 20.0f || rearWallC < 22.0f || (now - s_backoffStartTick) >= 340U)) {
            return false;
        }
        Motion_Stop();
        s_avoidStartTick = now;
        s_avoidPhase = AVOID_PHASE_POST_BACKOFF_SETTLE;
        return true;
    }

    case AVOID_PHASE_INFLATE: {
        const float hardObsSq = HARD_OBS_CLEAR_MM * HARD_OBS_CLEAR_MM;
        const float startRad = s_planStartHeadingDeg * DEG_TO_RAD_F;
        const float fwdX = cosf(startRad), fwdY = sinf(startRad);

        uint8_t endRow = (uint8_t)(s_inflateNextRow + INFLATE_ROWS_PER_SLICE);
        if (endRow > MAP_ROWS) endRow = MAP_ROWS;

        for (uint8_t r = s_inflateNextRow; r < endRow; r++) {
            float cy = GridCenterMm(r), dyStart = cy - s_planStartY;
            for (uint8_t c = 0U; c < MAP_COLS; c++) {
                if (g_map_grid[r][c] == MAP_VAL_OBSTACLE) {
                    s_inflated[r][c] = MAP_VAL_OBSTACLE;
                    continue;
                }
                float cx = GridCenterMm(c), obsSq = 0.0f, wallC = 0.0f;
                Auto_ComputeClearanceSqMm(cx, cy, &obsSq, &wallC);

                uint8_t cost = 0U;
                if (wallC < HARD_WALL_CLEAR_MM || obsSq < hardObsSq)         cost = MAP_VAL_OBSTACLE;
                else if (wallC < 95.0f || obsSq < (82.0f * 82.0f))           cost = 60U;
                else if (wallC < 115.0f || obsSq < (105.0f * 105.0f))        cost = 25U;

                float dxStart = cx - s_planStartX;
                if (cost < 95U && (dxStart * dxStart + dyStart * dyStart) <= (360.0f * 360.0f) &&
                    (dxStart * fwdX + dyStart * fwdY) < -5.0f) {
                    cost = 95U;
                }
                s_inflated[r][c] = cost;
            }
        }

        s_inflateNextRow = endRow;
        if (s_inflateNextRow < MAP_ROWS) return true;

        s_inflated[s_planStartRow][s_planStartCol] = MAP_VAL_FREE;
        bool hasCardinalExit = false;
        for (uint8_t k = 0U; k < 4U && !hasCardinalExit; k++) {
            int nr = (int)s_planStartRow + kDir8[k][1], nc = (int)s_planStartCol + kDir8[k][0];
            hasCardinalExit = (nr >= 0 && nr < (int)MAP_ROWS && nc >= 0 && nc < (int)MAP_COLS &&
                               s_inflated[nr][nc] != MAP_VAL_OBSTACLE);
        }
        if (!hasCardinalExit) {
            float sObs = 0.0f, sWall = 0.0f;
            Auto_ComputeClearanceMm(s_planStartX, s_planStartY, &sObs, &sWall);
            float startMinC = fminf(sObs, sWall);
            float reqGain = (startMinC < 44.0f) ? 2.0f : 3.0f;

            for (int8_t dr = -2; dr <= 2; dr++) {
                for (int8_t dc = -2; dc <= 2; dc++) {
                    if (dr == 0 && dc == 0) continue;
                    int nr = (int)s_planStartRow + dr, nc = (int)s_planStartCol + dc;
                    if (nr >= 0 && nr < (int)MAP_ROWS && nc >= 0 && nc < (int)MAP_COLS &&
                        g_map_grid[nr][nc] != MAP_VAL_OBSTACLE && s_inflated[nr][nc] == MAP_VAL_OBSTACLE) {
                        float nObs = 0.0f, nWall = 0.0f;
                        Auto_ComputeClearanceMm(GridCenterMm((uint8_t)nc), GridCenterMm((uint8_t)nr), &nObs, &nWall);
                        if (fminf(nObs, nWall) > (startMinC + reqGain)) s_inflated[nr][nc] = 80U;
                    }
                }
            }
        }

        Auto_PreparePlannerCandidates();
        s_avoidPhase = (s_numCandidates > 0U) ? AVOID_PHASE_SEARCH : AVOID_PHASE_APPLY;
        return true;
    }

    case AVOID_PHASE_SEARCH: {
        if (s_curCandidateIdx >= s_numCandidates) { s_avoidPhase = AVOID_PHASE_APPLY; return true; }
        const PlanCandidate *cand = &s_candidates[s_curCandidateIdx];
        if (cand->isFinishLine && s_bestCandCount > 0U) { Auto_AdvanceCandidate(); return true; }

        uint16_t startNode = (uint16_t)s_planStartRow * MAP_COLS + s_planStartCol;
        uint16_t targetGoalNode = (uint16_t)cand->goalRow * MAP_COLS + cand->goalCol;

        if (!s_curSearchInitialized) {
            memset(s_gScore, 0xFF, sizeof(s_gScore));
            memset(s_cameFrom, 0xFF, sizeof(s_cameFrom));
            memset(s_closed, 0, sizeof(s_closed));
            s_heapSize = 0U;
            s_gScore[startNode] = 0U;
            Heap_Push(startNode, Auto_Heuristic((int16_t)s_planStartCol, (int16_t)s_planStartRow,
                                                (int16_t)cand->goalCol, (int16_t)cand->goalRow, cand->isFinishLine));
            s_curSearchInitialized = true;
        }

        uint16_t reachedNode = 0xFFFFU;
        bool searchDone = false, searchFound = false;

        for (uint16_t iter = 0U; s_heapSize > 0U && iter < ASTAR_NODES_PER_SLICE; iter++) {
            uint16_t current = 0U, curKey = 0U;
            if (!Heap_Pop(&current, &curKey)) break;
            uint8_t c = (uint8_t)(current % MAP_COLS), r = (uint8_t)(current / MAP_COLS);

            if ((cand->isFinishLine && r == 0U && c >= FINISH_LINE_COL_MIN && c <= FINISH_LINE_COL_MAX) ||
                (!cand->isFinishLine && current == targetGoalNode)) {
                reachedNode = current;
                searchDone = searchFound = true;
                break;
            }
            if (ClosedSet_Contains(current)) continue;
            ClosedSet_Set(current);

            int8_t prevDc = 0, prevDr = 0;
            bool hasPrev = (s_cameFrom[current] != 0xFFFFU);
            if (hasPrev) {
                prevDc = (int8_t)c - (int8_t)(s_cameFrom[current] % MAP_COLS);
                prevDr = (int8_t)r - (int8_t)(s_cameFrom[current] / MAP_COLS);
            }

            for (uint8_t d = 0U; d < 8U; d++) {
                int8_t dc = kDir8[d][0], dr = kDir8[d][1];
                int16_t nc = (int16_t)c + dc, nr = (int16_t)r + dr;
                if (nc < 0 || nc >= (int16_t)MAP_COLS || nr < 0 || nr >= (int16_t)MAP_ROWS) continue;
                if (s_inflated[nr][nc] == MAP_VAL_OBSTACLE) continue;

                if (dc != 0 && dr != 0) {
                    if (g_map_grid[r][nc] == MAP_VAL_OBSTACLE || g_map_grid[nr][c] == MAP_VAL_OBSTACLE) continue;
                    if (current != startNode &&
                        (s_inflated[r][nc] == MAP_VAL_OBSTACLE || s_inflated[nr][c] == MAP_VAL_OBSTACLE)) {
                        float midObsSq = 0.0f, midWall = 0.0f;
                        Auto_ComputeClearanceSqMm((float)(c + nc + 1) * (MAP_CELL_SIZE_MM * 0.5f),
                                                  (float)(r + nr + 1) * (MAP_CELL_SIZE_MM * 0.5f), &midObsSq, &midWall);
                        if (midWall < HARD_WALL_CLEAR_MM || midObsSq < (HARD_OBS_CLEAR_MM * HARD_OBS_CLEAR_MM)) continue;
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
                    turnPenalty = (uint16_t)(fabsf(Auto_NormalizeAngleDiff(stepDeg, s_planStartHeadingDeg)) * 0.85f);
                }

                uint16_t tentativeG = s_gScore[current] + ((d < 4U) ? 10U : 14U) + (uint16_t)s_inflated[nr][nc] + turnPenalty;
                if (tentativeG < s_gScore[neighbor]) {
                    s_cameFrom[neighbor] = current;
                    s_gScore[neighbor]   = tentativeG;
                    Heap_Push(neighbor, tentativeG + Auto_Heuristic(nc, nr, (int16_t)cand->goalCol,
                                                                    (int16_t)cand->goalRow, cand->isFinishLine));
                }
            }
        }

        if (!searchDone && s_heapSize == 0U) searchDone = true;
        if (searchDone) {
            if (searchFound) {
                AutoWaypoint candRoute[AUTO_MAX_WAYPOINTS];
                uint8_t candCount = Auto_ReconstructAndSmoothPath(startNode, reachedNode, cand, candRoute, cand->maxDetour);
                bool tailBlocked = false;

                if (candCount > 0U && !cand->isFinishLine) {
                    for (uint8_t i = cand->rejoinWpIdx + 1U; i < s_numActiveWaypoints && candCount < AUTO_MAX_WAYPOINTS; i++) {
                        AutoWaypoint prev = candRoute[candCount - 1U];
                        if (!Auto_IsSegmentSafeMm(prev.x_mm, prev.y_mm, s_activeWaypoints[i].x_mm, s_activeWaypoints[i].y_mm,
                                                  HARD_OBS_CLEAR_MM, HARD_WALL_CLEAR_MM)) {
                            tailBlocked = true;
                            break;
                        }
                        candRoute[candCount++] = s_activeWaypoints[i];
                    }
                    if (!tailBlocked) Auto_PruneRouteByLOS(s_planStartX, s_planStartY, candRoute, &candCount, true);
                }

                if (candCount > 0U && !tailBlocked) {
                    float candCost = Auto_EvaluateRouteCost(s_planStartX, s_planStartY, s_planStartHeadingDeg, candRoute, candCount);
                    if (candCost < s_bestCandCost) {
                        s_bestCandCost = candCost;
                        s_bestCandCount = candCount;
                        memcpy(s_bestRoute, candRoute, sizeof(AutoWaypoint) * candCount);
                    }
                }
            }
            Auto_AdvanceCandidate();
        }
        return true;
    }

    case AVOID_PHASE_APPLY: {
        Odometry_State curOdom;
        if (Odometry_GetState(&curOdom) != HAL_OK) {
            curOdom.x_mm = s_planStartX;
            curOdom.y_mm = s_planStartY;
        }
        s_avoidPhase = AVOID_PHASE_NONE;
        s_hitSampleCount = 0U;

        if (s_bestCandCount > 0U) {
            memcpy(s_activeWaypoints, s_bestRoute, sizeof(AutoWaypoint) * s_bestCandCount);
            s_numActiveWaypoints = s_bestCandCount;
            s_activeWaypointIdx = 0U;

            if (s_numActiveWaypoints > 1U &&
                DistSq(curOdom.x_mm, curOdom.y_mm, s_activeWaypoints[0].x_mm, s_activeWaypoints[0].y_mm) < (25.0f * 25.0f)) {
                memmove(&s_activeWaypoints[0], &s_activeWaypoints[1], sizeof(AutoWaypoint) * (s_numActiveWaypoints - 1U));
                s_numActiveWaypoints--;
            }

            Auto_BuildPlan(s_activeWaypoints, s_numActiveWaypoints, curOdom.x_mm, curOdom.y_mm);
            if (s_numCorners > 0U && s_corners[0].type == CORNER_ARC &&
                DistSq(curOdom.x_mm, curOdom.y_mm, s_corners[0].Tin.x_mm, s_corners[0].Tin.y_mm) < (35.0f * 35.0f)) {
                s_corners[0].type = (fabsf(s_corners[0].dth_deg) < 20.0f) ? CORNER_NONE : CORNER_SPIN;
            }
            s_autoState = AUTO_STATE_RUNNING;
            Auto_NavToIdx(0U, false);
            return true;
        }

        if (s_tofValid[RAY_MID] && s_tofDist[RAY_MID] <= 120.0f) {
            Motion_Stop();
            s_autoState = AUTO_STATE_ERROR;
            Led_SetPattern(1U, 1U, 80U, 80U, 80U);
            return true;
        }

        s_autoState = AUTO_STATE_RUNNING;
        Auto_NavToIdx(s_activeWaypointIdx, false);
        return true;
    }

    default:
        return false;
    }
}

/* ================= GOM CỤM ĐIỂM CHẠM 3 TOF & ĐIỀU HƯỚNG TỰ ĐỘNG ================= */
static bool Auto_ProcessSensorHit(float hx, float hy, float rx, float ry, float rayDistMm, uint32_t now,
                                  float *outHitX, float *outHitY, float *outUx, float *outUy) {
    uint8_t writeIdx = 0U;
    for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
        if ((now - s_hitSamples[i].tick) <= 350U) s_hitSamples[writeIdx++] = s_hitSamples[i];
    }
    s_hitSampleCount = writeIdx;

    if (s_hitSampleCount >= CLUSTER_MAX_HITS) {
        memmove(&s_hitSamples[0], &s_hitSamples[1], sizeof(HitSample) * (CLUSTER_MAX_HITS - 1U));
        s_hitSampleCount = CLUSTER_MAX_HITS - 1U;
    }
    s_hitSamples[s_hitSampleCount++] = (HitSample){ hx, hy, rx, ry, now };

    uint8_t clusterCount = 0U;
    float sumX = 0.0f, sumY = 0.0f, sumRx = 0.0f, sumRy = 0.0f;
    const float threshSq = 45.0f * 45.0f;
    for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
        if (DistSq(s_hitSamples[i].x_mm, s_hitSamples[i].y_mm, hx, hy) <= threshSq) {
            clusterCount++;
            sumX  += s_hitSamples[i].x_mm;
            sumY  += s_hitSamples[i].y_mm;
            sumRx += s_hitSamples[i].rx;
            sumRy += s_hitSamples[i].ry;
        }
    }

    uint8_t minConfirm = (rayDistMm <= 95.0f) ? 2U : ((rayDistMm <= 155.0f) ? 3U : 4U);
    if (clusterCount < minConfirm) return false;

    *outHitX = sumX / (float)clusterCount;
    *outHitY = sumY / (float)clusterCount;
    float rLen = sqrtf(sumRx * sumRx + sumRy * sumRy);
    *outUx = (rLen > 0.01f) ? (sumRx / rLen) : rx;
    *outUy = (rLen > 0.01f) ? (sumRy / rLen) : ry;

    uint8_t keepIdx = 0U;
    for (uint8_t i = 0U; i < s_hitSampleCount; i++) {
        if (DistSq(s_hitSamples[i].x_mm, s_hitSamples[i].y_mm, hx, hy) > threshSq) {
            s_hitSamples[keepIdx++] = s_hitSamples[i];
        }
    }
    s_hitSampleCount = keepIdx;
    return true;
}

static void Auto_BeginReroute(void) {
    Motion_Stop();
    Motion_SetTightSpace(true);
    s_avoidStartTick = HAL_GetTick();
    s_hitSampleCount = 0U;
    s_avoidPhase = AVOID_PHASE_SETTLE;
    Auto_ResetPlannerState();
    s_autoState = AUTO_STATE_AVOIDING;
}

void Auto_Init(void) {
    s_startTick = 0U;
    s_numDynWallPts = 0U;
    memset(s_tofDist, 0, sizeof(s_tofDist));
    memset(s_tofValid, 0, sizeof(s_tofValid));
    Auto_ResetSession();
}

void Auto_UpdateSensors(float leftMm, bool leftValid, float midMm, bool midValid, float rightMm, bool rightValid) {
    s_tofDist[RAY_MID]   = midMm;   s_tofValid[RAY_MID]   = midValid;
    s_tofDist[RAY_LEFT]  = leftMm;  s_tofValid[RAY_LEFT]  = leftValid;
    s_tofDist[RAY_RIGHT] = rightMm; s_tofValid[RAY_RIGHT] = rightValid;
    s_sensorsFresh = (leftValid || midValid || rightValid);
}

void Auto_Stop(void) {
    Auto_ResetSession();
    Motion_Stop();
}

void Auto_OnButton1(void) {
    if (Auto_IsBusyState()) return;
    Auto_Stop();
    Odometry_Reset();
    s_numDynWallPts = 0U;
    Map_Init();

    s_numActiveWaypoints = NUM_UNIVERSAL_WAYPOINTS;
    memcpy(s_activeWaypoints, s_universalWaypoints, sizeof(s_universalWaypoints));
    s_startTick = HAL_GetTick();
    s_autoState = AUTO_STATE_STARTING;
    Auto_SetAllLeds(0U, 0U);
    Led_SetPattern(2U, 1U, 100U, 100U, 100U);
}

void Auto_OnButton2(void) {
    Auto_Stop();
    Auto_SetAllLeds(0U, 0U);
}

void Auto_Update(float dt_s) {
    (void)dt_s;
    uint32_t now = HAL_GetTick();

    if (s_autoState == AUTO_STATE_STARTING) {
        if ((now - s_startTick) >= 1000U) {
            Led_SetPattern(2U, 1U, 1000U, 0U, 0U);
            Odometry_Reset();
            s_activeWaypointIdx = 0U;
            s_autoState = AUTO_STATE_RUNNING;
            Auto_BuildPlan(s_activeWaypoints, s_numActiveWaypoints, ODOMETRY_INITIAL_X_MM, ODOMETRY_INITIAL_Y_MM);
            Auto_NavToIdx(0U, false);
        }
        return;
    }

    if (s_autoState == AUTO_STATE_AVOIDING) {
        (void)Auto_StepPlannerSlice();
        return;
    }
    if (s_autoState != AUTO_STATE_RUNNING) return;

    Odometry_State odom;
    if (Odometry_GetState(&odom) != HAL_OK) return;

    if (odom.x_mm >= 1200.0f && odom.y_mm <= -80.0f && Auto_IsLastWaypoint(s_activeWaypointIdx)) {
        Motion_Stop();
        s_autoState = AUTO_STATE_COMPLETED;
        Auto_SetAllLeds(1U, 1000U);
        return;
    }

    if (Motion_IsTurningInPlace()) s_hitSampleCount = 0U;

    bool inFinishZone = (odom.x_mm >= 1200.0f && odom.y_mm <= 180.0f);
    if (!inFinishZone && odom.x_mm >= 900.0f &&
        odom.y_mm >= -60.0f && odom.y_mm <= 1150.0f &&
        s_activeWaypointIdx < s_numActiveWaypoints && s_sensorsFresh) {

        s_sensorsFresh = false;
        if (!Motion_IsTurningInPlace()) {
            bool calmAngular = (!Motion_IsArcActive() && fabsf(odom.angular_rad_s) <= 0.55f);
            bool needReroute = false;

            for (uint8_t r = 0U; r < 3U; r++) {
                float dist = s_tofDist[r];
                float maxRange = calmAngular ? kRays[r].maxRange : 145.0f;
                if (!s_tofValid[r] || dist < 25.0f || dist > maxRange) continue;

                float px = 0.0f, py = 0.0f;
                kRays[r].pose(dist, odom.x_mm, odom.y_mm, odom.heading_deg, &px, &py);
                float slackMm = (dist > 150.0f) ? ((dist - 150.0f) * 0.18f) : 0.0f;
                if (px < 1135.0f || Auto_IsKnownStaticObstacle(px, py, slackMm)) continue;

                float rayRad = (odom.heading_deg + kRays[r].offsetDeg) * DEG_TO_RAD_F;
                float rx = cosf(rayRad), ry = sinf(rayRad), hx = 0.0f, hy = 0.0f, ux = rx, uy = ry;
                if (!Auto_ProcessSensorHit(px, py, rx, ry, dist, now, &hx, &hy, &ux, &uy)) continue;
                if (hx < 1135.0f || Auto_IsKnownStaticObstacle(hx, hy, slackMm)) continue;

                const float offsets[3] = { 0.0f, -20.0f, 20.0f };
                for (uint8_t k = 0U; k < 3U; k++) {
                    float wx = hx - offsets[k] * uy, wy = hy + offsets[k] * ux;
                    if (wx < 1135.0f || Auto_IsKnownStaticObstacle(wx, wy, slackMm)) continue;

                    bool isNewPt = false;
                    if (!Auto_RecordWallPoint(wx, wy, &isNewPt) || (!isNewPt && dist > 150.0f)) continue;
                    if (Auto_IsWallPointBlockingRoute(wx, wy, odom.x_mm, odom.y_mm, odom.heading_deg,
                                                      s_activeWaypoints, s_activeWaypointIdx, s_numActiveWaypoints,
                                                      isNewPt ? 48.0f : 42.0f)) {
                        needReroute = true;
                    }
                }
            }
            if (needReroute) {
                Auto_BeginReroute();
                return;
            }
        }
    }

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
                float nextX = 0.0f, nextY = 0.0f;
                Auto_GetAimTarget(s_activeWaypointIdx, &nextX, &nextY);
                Motion_StartArcICR(cn->R, cn->dir, cn->th2_deg, cn->Tout.x_mm, cn->Tout.y_mm,
                                   cn->u2_x, cn->u2_y, nextX, nextY, Auto_IsLastWaypoint(s_activeWaypointIdx));
                return;
            }

            float projTout   = (odom.x_mm - cn->Tout.x_mm) * cn->u2_x + (odom.y_mm - cn->Tout.y_mm) * cn->u2_y;
            float dCrossTout = fabsf((odom.x_mm - cn->Tout.x_mm) * cn->u2_y - (odom.y_mm - cn->Tout.y_mm) * cn->u2_x);
            float projWp     = (odom.x_mm - cn->wp.x_mm) * cn->u1_x + (odom.y_mm - cn->wp.y_mm) * cn->u1_y;
            if (d_along > NAV_TANGENT_PASS_DIST_MM || projWp > 20.0f ||
                (d_along >= -20.0f && projTout >= 0.0f && dCrossTout <= 40.0f)) {
                Auto_NavToIdx(++s_activeWaypointIdx, false);
                return;
            }
        } else {
            float rem  = DistMm(odom.x_mm, odom.y_mm, cn->wp.x_mm, cn->wp.y_mm);
            float proj = (odom.x_mm - cn->wp.x_mm) * cn->u1_x + (odom.y_mm - cn->wp.y_mm) * cn->u1_y;
            if (rem <= ((cn->type == CORNER_SPIN) ? 18.0f : 25.0f) || proj >= NAV_WAYPOINT_PASS_DIST_MM) {
                Auto_NavToIdx(++s_activeWaypointIdx, cn->type == CORNER_SPIN);
                return;
            }
        }
    }

    int16_t statusWord = Motion_GetFeedbackStatusWord();
    uint8_t activeCmd  = (uint8_t)((statusWord >> 8U) & 0xFFU);
    uint8_t cmdStatus  = (uint8_t)(statusWord & 0xFFU);

    if (activeCmd == CMD_NAV_TO_POINT && cmdStatus == STATUS_COMPLETED) {
        if (++s_activeWaypointIdx < s_numActiveWaypoints) {
            Auto_NavToIdx(s_activeWaypointIdx, false);
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
