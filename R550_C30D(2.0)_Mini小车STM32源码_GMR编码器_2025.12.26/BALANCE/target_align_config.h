#ifndef TARGET_ALIGN_CONFIG_H
#define TARGET_ALIGN_CONFIG_H
#define TARGET_SLOW_Y_M (-0.130f) /* 打靶启动位置累计提前0.3米，向右行驶时Y减小。 */
#define TARGET_SEARCH_END_Y_M (-1.400f)
#define TARGET_TRAVEL_MPS 0.30f
#define TARGET_SEARCH_MPS 0.15f
#define TARGET_ALIGN_SIGN (-1.0f)
#define TARGET_ALIGN_KP 0.10f
#define TARGET_ALIGN_MAX_MPS 0.03f
#define TARGET_ALIGN_TOL_X 0.10f /* 打靶死区：误差不超过半幅宽的10%，320宽时为正负16像素。 */
#define TARGET_ALIGN_STABLE_MS 2000U /* 靶心连续在死区内2秒后开始打靶。 */
#define TARGET_FRESH_MS 200U
#define TARGET_REQUEST_MS 100U
#define TARGET_MAX_ALIGN_TRAVEL_M 0.25f
#endif
