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
#define CONFIG_CAMERA_FRAME_RATE (30)

/**
 * @brief Scalar value to transmit framebuffers over HTTP
 * Since sending a frame over HTTP is considerably slower
 * than saving it, only some frames should be sent over
 * @note behavior is undefined for scalar values < 1
 * @note It is recomended to set this to a multiple of
 * the frame rate
 * 
 */
#define CONFIG_CAMERA_STREAM_FRAME_DOWNSCALE ((int) 15)

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