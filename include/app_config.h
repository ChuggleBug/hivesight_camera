#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/**
 * @brief Application UART baud rate for UART to USB comm
 * 
 */
#define CONFIG_BAUD_RATE (PIO_CONFIG_BAUD_RATE)

/**
 * @brief Maximum allowable framerate for saving video
 * to on device SD card
 * 
 */
#define CONFIG_CAMERA_SAVE_FRAME_RATE (30)

/**
 * @brief Maximum allowable framerate for sending video
 * over to coordinator over HTTP
 * 
 */
#define CONFIG_CAMERA_STREAM_FRAME_RATE (5)

/**
 * @brief Root of camera frame buffers
 * 
 */
#define CAMERA_FB_ROOT "/camera"

/**
 * @brief Number of seconds to store *prior*
 * to an event happening
 * 
 */
#define CAMERA_FB_SECOND_RANGE (30)

#endif  // __APP_CONFIG_H