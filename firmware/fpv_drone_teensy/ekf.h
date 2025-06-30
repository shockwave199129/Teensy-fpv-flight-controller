#pragma once

#include "config.h"

namespace EKF {
    void init(float dt);
    void predict(float ax, float ay, float dt);
    void update_gps(float lat, float lon, float vel_n, float vel_e);
    void update_optflow(float vel_x, float vel_y);
    void get_state(float& lat, float& lon, float& vel_n, float& vel_e);
} 