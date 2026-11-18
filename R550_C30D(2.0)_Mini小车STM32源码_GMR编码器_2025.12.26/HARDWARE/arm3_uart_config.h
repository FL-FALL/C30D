#ifndef ARM3_UART_CONFIG_H
#define ARM3_UART_CONFIG_H
/* ZP25S, dedicated UART4 TX on PC10. Move signal wire from PC9 to PC10.
 * Actual servo ID is 001, NOT the logical joint index 3.
 * Position mode 1/2 must already be configured; no persistent mode changes here.
 */
#define ARM3_UART_ID 1U
#define ARM3_UART_BAUD 115200U
#define ARM3_UART_MOVE_MS 1000U
#define ARM3_UART_MIN_P 500U
#define ARM3_UART_MAX_P 2500U
#define ARM3_UART_FULL_RANGE_DEG 270U
/* Finite polling bound if UART clock/peripheral fails. */
#define ARM3_UART_POLL_LIMIT 200000U
#endif
