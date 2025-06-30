#include "ekf.h"
#include <Arduino.h>

namespace {
    // State vector: [pos_N, pos_E, vel_N, vel_E]
    float x[4] = {0};
    float P[4][4];
    // Process noise
    const float q_pos = 0.1f;
    const float q_vel = 0.5f;
    // Measurement noise
    const float r_gps_pos = 3.0f;
    const float r_gps_vel = 1.0f;
    const float r_flow_vel = 0.5f;
}

namespace EKF {

void init(float dt) {
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) P[i][j] = (i==j)?1.0f:0.0f;
}

void predict(float ax, float ay, float dt) {
    // Simple constant-accel integration to velocity then position (ax,ay in m/s^2 body frame already converted)
    x[2] += ax*dt;
    x[3] += ay*dt;
    x[0] += x[2]*dt;
    x[1] += x[3]*dt;
    // Inflate covariance
    P[0][0]+= q_pos; P[1][1]+=q_pos; P[2][2]+=q_vel; P[3][3]+=q_vel;
}

static void update_generic(const float z[2], const float H[2][4], float R) {
    // innovation y = z - Hx
    float y[2] = { z[0] - (H[0][0]*x[0] + H[0][2]*x[2]),
                   z[1] - (H[1][1]*x[1] + H[1][3]*x[3]) };
    // S = HPH^T + R
    float S = P[0][0] + P[1][1] + R; // approx diag identical
    float K = P[0][0]/S; // simplistic scalar gain
    // Update state
    x[0] += K*y[0]; x[1]+=K*y[1];
    // Update covariance
    P[0][0] *= (1-K); P[1][1]*=(1-K);
}

void update_gps(float lat, float lon, float vel_n, float vel_e) {
    float z_pos[2] = {lat, lon};
    const float H_pos[2][4] = {{1,0,0,0},{0,1,0,0}};
    update_generic(z_pos, H_pos, r_gps_pos);
    float z_vel[2] = {vel_n, vel_e};
    const float H_vel[2][4] = {{0,0,1,0},{0,0,0,1}};
    update_generic(z_vel, H_vel, r_gps_vel);
}

void update_optflow(float vel_x, float vel_y) {
    float z[2] = {vel_x, vel_y};
    const float H[2][4] = {{0,0,1,0},{0,0,0,1}};
    update_generic(z, H, r_flow_vel);
}

void get_state(float& lat, float& lon, float& vel_n, float& vel_e) {
    lat = x[0]; lon = x[1]; vel_n = x[2]; vel_e = x[3];
}

} 