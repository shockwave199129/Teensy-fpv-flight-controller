#include "navigation_helpers.h"
#include "config.h"
#include <math.h>

namespace {
    constexpr float DEG_TO_RAD_F = 3.14159265359f / 180.0f;
    constexpr float EARTH_RADIUS_M = 6371000.0f;
}

namespace NavigationHelpers {

float distance_between(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * DEG_TO_RAD_F;
    float dLon = (lon2 - lon1) * DEG_TO_RAD_F;
    float a = sinf(dLat/2)*sinf(dLat/2) + cosf(lat1*DEG_TO_RAD_F)*cosf(lat2*DEG_TO_RAD_F)*sinf(dLon/2)*sinf(dLon/2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    return EARTH_RADIUS_M * c;
}

float bearing_to(float lat1, float lon1, float lat2, float lon2) {
    float y = sinf((lon2-lon1)*DEG_TO_RAD_F) * cosf(lat2*DEG_TO_RAD_F);
    float x = cosf(lat1*DEG_TO_RAD_F)*sinf(lat2*DEG_TO_RAD_F) - sinf(lat1*DEG_TO_RAD_F)*cosf(lat2*DEG_TO_RAD_F)*cosf((lon2-lon1)*DEG_TO_RAD_F);
    float brng = atan2f(y,x) / DEG_TO_RAD_F; // to degrees
    if (brng < 0) brng += 360.0f;
    return brng;
}

void set_nav_heading(float bearing_deg) {
    extern SensorData sensor_data;
    extern RcData rc_data_global;

    float yaw_error = bearing_deg - sensor_data.yaw;
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    float yaw_correction = constrain(yaw_error * 2.5f, -500.0f, 500.0f);
    rc_data_global.yaw = 1500 + (int)yaw_correction;
}

} // namespace NavigationHelpers 