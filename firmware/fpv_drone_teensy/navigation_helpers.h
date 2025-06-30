#pragma once

#include <Arduino.h>

namespace NavigationHelpers {
    // Returns distance in metres between two lat/lon points (WGS-84) using haversine.
    float distance_between(float lat1, float lon1, float lat2, float lon2);

    // Returns bearing in degrees (0-360) from point1 to point2.
    float bearing_to(float lat1, float lon1, float lat2, float lon2);

    // Set desired navigation heading – placeholder, ties into autopilot heading hold.
    void set_nav_heading(float bearing_deg);
} 