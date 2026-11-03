#include "roll_control/roll.h"
#include <math.h>

RollCommand roll_step(const RollConfig *config, float roll_angle_rad,
                      float roll_rate_radps, RollCommand previous) {
  float ratio = config->inertia_kgm2 / config->torque_nm;

  float scaled_rate = ratio * roll_rate_radps;

  float root =
      sqrtf(ratio * fabsf(roll_angle_rad) + .5f * scaled_rate * scaled_rate);

  int direction = (roll_angle_rad > 0.0f) - (roll_angle_rad < 0.0f);

  float switch_time_s = ratio * roll_rate_radps * direction + root;
  float finish_time_s = ratio * roll_rate_radps * direction + 2.f * root;

  RollCommand command = ROLL_OFF;

  float switching_error_rad =
      fabsf(roll_angle_rad + config->rate_weight_s * roll_rate_radps);

  if ((!previous && (switching_error_rad > config->on_threshold_rad)) ||
      (previous && (switching_error_rad >= config->off_threshold_rad))) {

    if (ceilf(switch_time_s / config->dt) * config->dt > 0) {
      command = -direction;
    } else if (ceilf(finish_time_s / config->dt) * config->dt > 0) {
      command = direction;
    }
  }

  return command;
}
