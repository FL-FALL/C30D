#ifndef BALL_ALIGN_CONFIG_H
#define BALL_ALIGN_CONFIG_H
/* Body +X forward, +Y left. Image +x right, +y down.
 * vx = X_SIGN * KP_X * normalized image-y error.
 * vy = Y_SIGN * KP_Y * normalized image-x error. No commanded yaw rotation. */
#define BALL_ALIGN_X_SIGN (-1.0f)
#define BALL_ALIGN_Y_SIGN (-1.0f)
#define BALL_ALIGN_KP_X 0.08f
#define BALL_ALIGN_KP_Y 0.08f
#define BALL_ALIGN_MAX_MPS 0.03f
/* Error normalized by half image width/height: 0.06 = 9.6/7.2 px at 320x240. */
#define BALL_ALIGN_TOL_X 0.08f
#define BALL_ALIGN_TOL_Y 0.20f
#define BALL_ALIGN_STABLE_FRAMES 5U
#define BALL_ALIGN_STABLE_MS 1000U /* Continuous centered AND wheels stopped. */
#define BALL_ALIGN_FRESH_MS 200U
#define BALL_ALIGN_REQUEST_MS 100U
#define BALL_ALIGN_TIMEOUT_MS 15000U /* Legacy: no timeout while waiting for XY. */
#define BALL_ALIGN_MAX_TRAVEL_M 0.25f
#endif
