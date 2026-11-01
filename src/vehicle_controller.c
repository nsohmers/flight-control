#include "vehicle_controller.h"
#include "lqr/lqr.h"

VehicleCommand vehicle_control_step(const VehicleState *state,
                                    const VehicleCommand *previous,
                                    const VehicleGoal *goal) {
  // Gimbal around x axis
  float yz_output[CONTROL_COUNT];

  // Gimbal around y axis
  float xz_output[CONTROL_COUNT];

  // K matrix expects this convention:
  // XZ: [ 1, 1,  1, 1, -1, -1]
  // YZ: [-1, 1, -1, 1, -1, -1]

  {
    float xz_state[STATE_COUNT] = {
        state->x_m,    state->z_m,        state->vx_mps,
        state->vz_mps, -state->theta_rad, -state->theta_rate_radps,
    };

    float xz_goal[STATE_COUNT] = {
        0.0, goal->height_m, 0.0, goal->vertical_velocity_mps, 0.0, 0.0,
    };

    float xz_prev_output[CONTROL_COUNT] = {previous->gimbal_y_rad,
                                           previous->thrust_reduction};

    compute_lqr_step(&lqr_config, xz_goal, xz_state, xz_prev_output, xz_output);
  }

  {
    float yz_state[STATE_COUNT] = {
        -state->y_m,   state->z_m,      -state->vy_mps,
        state->vz_mps, -state->phi_rad, -state->phi_rate_radps,
    };

    float yz_goal[STATE_COUNT] = {
        0.0, goal->height_m, 0.0, goal->vertical_velocity_mps, 0.0, 0.0,
    };

    float yz_prev_output[CONTROL_COUNT] = {previous->gimbal_x_rad,
                                           previous->thrust_reduction};

    compute_lqr_step(&lqr_config, yz_goal, yz_state, yz_prev_output, yz_output);
  }

  VehicleCommand output = {
      .gimbal_x_rad = yz_output[GAMMA],
      .gimbal_y_rad = xz_output[GAMMA],
      .thrust_reduction = .5f * (yz_output[P] + xz_output[P]),
  };

  return output;
}
