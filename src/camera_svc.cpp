#include <Arduino.h>
#include <WiFi.h>

#include "board_config.h"
#include "esp32-hal-ledc.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "fb_gfx.h"
#include "img_converters.h"
#include "sdkconfig.h"

// ===========================
// Select camera model in board_config.h
// ===========================
#include "app_config.h"
#include "board_config.h"
#include "main.h"

// === Local Defines ===
#define CAMERA_FB_SAVE_SZ (CONFIG_CAMERA_SAVE_FRAME_RATE * 2)

void camera_svc_start();

void camera_svc_task(void *pvParameters);
void camera_svc_save_task(void *pvParameters);

static void save_fb_to_sd(const camera_fb_t *fb, int time_index,
                          int frame_index);

TaskHandle_t CameraServiceTask;
TaskHandle_t CameraServiceSaveTask;

QueueHandle_t CameraFBSaveQ;

typedef struct _app_camera_frame {
  camera_fb_t *fb;
  unsigned long time_index;
  unsigned long frame_index;
} camera_frame_t;

void camera_svc_start() {
  Serial.println("Starting Camera...");

  // Create dir for image to store image buffers
  if (SD_MMC.exists(CAMERA_FB_ROOT)) {
    SD_MMC.rmdir(CAMERA_FB_ROOT);
  }
  SD_MMC.mkdir(CAMERA_FB_ROOT);

  xQueueCreate(CAMERA_FB_SAVE_SZ, sizeof(camera_frame_t));

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_QQVGA;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  xTaskCreate(camera_svc_task, "CamSvcTask", 8192, NULL, 5, &CameraServiceTask);

  xTaskCreate(camera_svc_save_task, "CamSvcSaveTask", 8192, NULL, 2,
              &CameraServiceSaveTask);
}

void camera_svc_task(void *pvParameters) {
  camera_fb_t *fb = NULL;
  static char url_buf[256];
  String url;
  String path;
  int prev_time_index;
  int time_index;
  int frame_index = 0;
  int resp;
  camera_frame_t frame;
  
  // Create url to use for API call
  memset(url_buf, 0, 256);
  snprintf(url_buf, 256, "http://%s:%d/api/device/stream?device=%s",
           coordinatorIP.toString().c_str(), coordinatorPort,
           deviceName.c_str());
  // Arduino libaries like to use String instead of char*
  url = url_buf;
  TickType_t prevTick = xTaskGetTickCount();
  prev_time_index = timeClient.getEpochTime() % (CAMERA_FB_SECOND_RANGE * 2);
  for (;;) {
    time_index = timeClient.getEpochTime() % (CAMERA_FB_SECOND_RANGE * 2);
    if (prev_time_index != time_index) {
      Serial.printf("Current time index = %d", time_index);
      Serial.println();
      prev_time_index = time_index;
      frame_index = 0;
    }

    fb = esp_camera_fb_get();
    if (fb == NULL) {
      Serial.println("Error capturing video buffer!");
      continue;
    }

    frame.fb = fb;
    frame.time_index = time_index;
    frame.frame_index = frame_index;

    // Fail immediately if the save queue is full
    // in order to try and maintain framerate
    if (xQueueSend(CameraFBSaveQ, &frame, 0) != pdPASS) {
      Serial.println("Frame dropped...");
      esp_camera_fb_return(fb);
    }

    // unsigned long t_start = millis();

    // http.begin(url);
    // http.addHeader("Content-Type", "image/jpeg");
    // resp = http.PUT(fb->buf, fb->len);

    // unsigned long t_end = millis();
    // unsigned long elapsed = t_end - t_start;

    // Serial.printf("Upload time: %lu ms\n", elapsed);

    // if (resp != HTTP_CODE_NO_CONTENT) {
    //   String err_msg = http.errorToString(resp);
    //   Serial.printf("Unexpected response. Got %s (%d)\n", err_msg.c_str(),
    //                 resp);
    // }

    // http.end();

    vTaskDelayUntil(&prevTick, pdMS_TO_TICKS(1000 / CONFIG_CAMERA_SAVE_FRAME_RATE))
  }
}

void camera_svc_save_task(void *pvParameters) {
  camera_frame_t frame;

  for (;;) {
    xQueueReceive(CameraFBSaveQ, &frame, portMAX_DELAY);

    save_fb_to_sd(frame.fb, frame.time_index, frame.frame_index);

    // Because sending an image over HTTP is considerably slower,
    // only some frames will be sent over. In that case, the frame
    // to be sent will not be released (yet)
    // TODO: Pass frame over to http transmitter task

    esp_camera_fb_return(frame.fb);

    // Is anything else busy?
    portYIELD();
  }
}

static void ensure_empty_dir(const char *path) {
  if (!SD_MMC.exists(path)) {
    SD_MMC.mkdir(path);
    return;
  }

  // Directory exists → clear its contents
  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) {
    Serial.printf("Failed to open existing directory: %s\n", path);
    return;
  }

  File entry;
  while ((entry = dir.openNextFile())) {
    String entryPath = String(path) + "/" + entry.name();
    if (entry.isDirectory()) {
      // Optional: recursively delete subfolders (not expected here)
      SD_MMC.rmdir(entryPath.c_str());
    } else {
      SD_MMC.remove(entryPath.c_str());
    }
    entry.close();
  }
  dir.close();
}

static void save_fb_to_sd(const camera_fb_t *fb, int time_index,
                          int frame_index) {
  if (!fb) return;

  // Build directory path
  char dir_path[64];
  snprintf(dir_path, sizeof(dir_path), "%s/%d", CAMERA_FB_ROOT, time_index);

  // Create or reset directory
  static int last_time_index = -1;
  if (time_index != last_time_index) {
    ensure_empty_dir(dir_path);
    last_time_index = time_index;
  }

  // File path for current frame
  char file_path[96];
  snprintf(file_path, sizeof(file_path), "%s/%d.jpg", dir_path, frame_index);

  // Open file
  File file = SD_MMC.open(file_path, FILE_WRITE);
  if (!file) {
    Serial.printf("Failed to open file for writing: %s\n", file_path);
    return;
  }

  // Write frame buffer
  file.write(fb->buf, fb->len);
  file.close();

  Serial.printf("Saved frame: %s (%d bytes)\n", file_path, fb->len);
}