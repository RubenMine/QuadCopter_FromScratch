// Utils.cpp
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

float interpolate(float h_start, float h_end, float t_diff) {
    float T_trans = 4 * abs(h_end - h_start);
    float h;
    if (t_diff <= T_trans)
        h = h_start + (h_end - h_start) / 2 * (1 - cos(atan(1) * 4 * t_diff / T_trans));
    else
        h = h_end;
    
    //char msg[70];
    //snprintf(msg, sizeof(msg), "interpolate: h_start=%d, h_end=%d, t_diff=%f, result=%f", 				h_start, h_end, t_diff, h);
    //sendDebug(msg);
    return h;
}

