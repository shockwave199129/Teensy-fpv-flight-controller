#include "navigation_helpers.h"
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
    // Placeholder – in real firmware this would feed into a yaw/PID autopilot.
    Serial.print("Nav heading set to "); Serial.println(bearing_deg);
}

} // namespace NavigationHelpers 