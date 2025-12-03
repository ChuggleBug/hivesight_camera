#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/**
 * @brief Application UART baud rate for UART to USB comm
 * 
 */
#define CONFIG_BAUD_RATE (PIO_CONFIG_BAUD_RATE)

/**
 * @brief Maximum allowable framerate for the camera
 * 
 */
#define CONFIG_VIDEO_FRAME_RATE (100)

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