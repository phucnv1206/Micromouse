#ifndef NAVIGATION
#define NAVIGATION

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void leftpose(float dis, float x, float y, float theta, float *x_pos, float *y_pos);
void midpose(float dis, float x, float y, float theta, float *x_pos, float *y_pos);
void rightpose(float dis, float x, float y, float theta, float *x_pos, float *y_pos);

#ifdef __cplusplus
}
#endif

#endif

