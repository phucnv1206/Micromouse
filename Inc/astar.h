#ifndef MICROMOUSE_ASTAR_H
#define MICROMOUSE_ASTAR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AUTO_STATE_IDLE = 0,   /* Xe đứng yên chờ lệnh */
    AUTO_STATE_STARTING,   /* Đếm ngược ~1s trước khi xuất phát */
    AUTO_STATE_RUNNING,    /* Đang bám lộ trình */
    AUTO_STATE_AVOIDING,   /* Dừng khẩn cấp và tính đường né A* */
    AUTO_STATE_COMPLETED,  /* Đã cán đích thành công */
    AUTO_STATE_ERROR       /* Lỗi điều hướng giữa đường */
} AutoState;

typedef struct {
    float x_mm;
    float y_mm;
} AutoWaypoint;

#define AUTO_MAX_WAYPOINTS 24U

void Auto_Init(void);
void Auto_UpdateSensors(float leftMm, bool leftValid,
                        float midMm, bool midValid,
                        float rightMm, bool rightValid);
void Auto_Update(float dt_s);
bool Auto_StepPlannerSlice(void);

void Auto_OnButton1(void);
void Auto_OnButton2(void);
void Auto_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MICROMOUSE_ASTAR_H */
