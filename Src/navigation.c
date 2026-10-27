#include "navigation.h"
#include <math.h>

#define DEG_TO_RAD 0.01745329251994329577f  // PI / 180.0f

void leftpose(float dis, float x, float y, float theta, float *x_pos, float *y_pos) {
    float phi1 = (theta + 55.7f) * DEG_TO_RAD;
    float phi2 = (theta + 45.0f) * DEG_TO_RAD;

    *x_pos = x + 33.9f * cosf(phi1) + dis * cosf(phi2);
    *y_pos = y + 33.9f * sinf(phi1) + dis * sinf(phi2);
}

void midpose(float dis, float x, float y, float theta, float *x_pos, float *y_pos) {
    float rad = theta * DEG_TO_RAD;

    *x_pos = x + (33.9f + dis) * cosf(rad);
    *y_pos = y + (33.9f + dis) * sinf(rad);
}

void rightpose(float dis, float x, float y, float theta, float *x_pos, float *y_pos) {
    float phi1 = (theta - 55.7f) * DEG_TO_RAD;
    float phi2 = (theta - 45.0f) * DEG_TO_RAD;

    *x_pos = x + 33.9f * cosf(phi1) + dis * cosf(phi2);
    *y_pos = y + 33.9f * sinf(phi1) + dis * sinf(phi2);
}

