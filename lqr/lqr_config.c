#include "lqr.h"

const LQRConfig lqr_config = {

    // Gain matrix: X, Z, dX, dZ, theta, dTheta (x2)
    .K =
        {
            [GAMMA] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            [P] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        },

    // From python
    .dt = (1.f / 15.f),

    // I don't know what reasonable limits are
    // Aniket can help here
    .gamma_limits =
        {
            .min = -1.f,
            .max = 1.f,
            .max_rate_per_s = 1.f,
        },

    .p_limits = {
        .min = -1.f,
        .max = 1.f,
        .max_rate_per_s = 1.f,
    }};
