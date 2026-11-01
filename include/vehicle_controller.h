#pragma once

#include "lqr/lqr.h"

typedef struct {
  float x_m;
  float y_m;
  float z_m;

  float vx_mps;
  float vy_mps;
  float vz_mps;

  // Euler angle representation of rocket angle
  float theta_rad;
  float theta_rate_radps;

  float phi_rad;
  float phi_rate_radps;

  float psi_rad;
  float psi_rate_radps;
} VehicleState;

typedef struct {
  float height_m;
  float vertical_velocity_mps;
} VehicleGoal;

typedef struct {
  float gimbal_x_rad; // original sims gx/gy
  float gimbal_y_rad;
  float thrust_reduction; // P
} VehicleCommand;

VehicleCommand vehicle_control_step(const VehicleState *state,
                                    const VehicleCommand *previous,
                                    const VehicleGoal *goal);
