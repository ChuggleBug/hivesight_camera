#include <Arduino.h>
#include <WiFi.h>

#include "app_config.h"
#include "board_config.h"
#include "esp32-hal-ledc.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "fb_gfx.h"
#include "img_converters.h"
#include "main.h"
#include "sdkconfig.h"

// === Enum Classes ===

enum class CAM_STATE {
  NORMAL,     // normal operations, passively saving + straming
  RECORDING,  // Notified that it will upload its rolling buffer soon
  UPLOADING,  // Actively uploading its video
};

// === Variables ===

CAM_STATE camera_state;
/**
 * @brief Time since the event event occured.
 * @note When uploading, the camera will look
 * from send from times [start_time - CAMERA_FB_SECOND_RANGE,
 * start_time + CAMERA_FB_SECOND_RANGE] (with
 * wraparound)
 */
int record_start_time;
/**
 * @brief Timestamp of event sent over by the sensor
 * This will be sent over to the coordinator
 * when uploading the saved recording
 *
 */
uint32_t timestamp;

// === Local Defines ===

#define CAMERA_FB_SAVE_SZ (CONFIG_CAMERA_FRAME_RATE * 2)

// === Local Functions ===

void camera_svc_start();
static void _ensure_empty_dir(const char *path);
static void save_fb_to_sd(const camera_fb_t *fb, int time_index,
                          int frame_index);
static void upload_frames(int start_index, int end_index);

// === Task Functions ===

void camera_svc_task(void *pvParameters);
void camera_svc_save_task(void *pvParameters);
void camera_svc_http_task(void *pvParameters);
void camera_svc_event_task(void *pvParameters);

// === FreeRTOS Objects ===

TaskHandle_t CameraServiceTask;
TaskHandle_t CameraServiceSaveTask;
TaskHandle_t CameraServiceHTTPTask;
TaskHandle_t CameraServiceEventTask;

QueueHandle_t CameraFBSaveQ;  // <camera_frame_t*>
QueueHandle_t CameraFBHTTPQ;  // <camera_frame_t*>

// Reference-counted frame wrapper
typedef struct _app_camera_frame {
  camera_fb_t *fb;
  int time_index;
  int frame_index;
  int refs;
} camera_frame_t;

/**
 * @note The refs counter to ensure free only once
 * was generated using AI. The logic of passing
 * framebuffers to svc -> save -> http was human design
 *
 */

static portMUX_TYPE frame_mux = portMUX_INITIALIZER_UNLOCKED;

static camera_frame_t *frame_alloc(camera_fb_t *fb, int time_index,
                                   int frame_index) {
  camera_frame_t *f = (camera_frame_t *)pvPortMalloc(sizeof(camera_frame_t));
  if (!f) return NULL;
  f->fb = fb;
  f->time_index = time_index;
  f->frame_index = frame_index;
  f->refs = 1;
  return f;
}

// increment reference count (thread-safe)
static void frame_ref(camera_frame_t *f) {
  if (!f) return;
  portENTER_CRITICAL(&frame_mux);
  f->refs++;
  portEXIT_CRITICAL(&frame_mux);
}

// decrement reference count and free when 0 (returns fb exactly once)
static void frame_release(camera_frame_t *f) {
  if (!f) return;
  bool do_free = false;
  portENTER_CRITICAL(&frame_mux);
  f->refs--;
  if (f->refs <= 0) {
    do_free = true;
  }
  portEXIT_CRITICAL(&frame_mux);

  if (do_free) {
    if (f->fb) {
      // return framebuffer to driver
      esp_camera_fb_return(f->fb);
    }
    vPortFree(f);
  }
}

void camera_svc_start() {
  Serial.println("Starting Camera...");

  // Create dir for image to store image buffers
  if (SD_MMC.exists(CAMERA_FB_ROOT)) {
    SD_MMC.rmdir(CAMERA_FB_ROOT);
  }
  SD_MMC.mkdir(CAMERA_FB_ROOT);

  // Queues hold pointers to camera_frame_t
  CameraFBSaveQ = xQueueCreate(CAMERA_FB_SAVE_SZ, sizeof(camera_frame_t *));
  CameraFBHTTPQ =
      xQueueCreate(CAMERA_FB_SAVE_SZ / CONFIG_CAMERA_STREAM_FRAME_DOWNSCALE,
                   sizeof(camera_frame_t *));

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
  // config.frame_size = FRAMESIZE_QVGA;
  config.frame_size = FRAMESIZE_UXGA;
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
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
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

  camera_state = CAM_STATE::NORMAL;
  xTaskCreate(camera_svc_task, "CamSvcTask", 8192, NULL, 3, &CameraServiceTask);

  xTaskCreate(camera_svc_save_task, "CamSvcSaveTask", 8192, NULL, 5,
              &CameraServiceSaveTask);

  xTaskCreate(camera_svc_http_task, "CamSvcHTTPTask", 16384, NULL, 8,
              &CameraServiceHTTPTask);

  xTaskCreate(camera_svc_event_task, "CamSvcEventTask", 2048, NULL, 8,
              &CameraServiceEventTask);
}

void camera_svc_task(void *pvParameters) {
  camera_fb_t *fb = NULL;
  int prev_time_index;
  int time_index;
  int frame_index = 0;

  camera_frame_t *frame_ptr = NULL;

  TickType_t prevTick = xTaskGetTickCount();
  prev_time_index = timeClient.getEpochTime() % (CAMERA_FB_SECOND_RANGE * 2);
  record_start_time = -1;
  for (;;) {
    time_index = timeClient.getEpochTime() % (CAMERA_FB_SECOND_RANGE * 2);
    if (prev_time_index != time_index) {
      prev_time_index = time_index;
      frame_index = 0;
    }

    if (record_start_time != -1) {
      // Handle wrap around times
      int dt = (time_index - record_start_time + (CAMERA_FB_SECOND_RANGE * 2)) %
               (CAMERA_FB_SECOND_RANGE * 2);

      if (dt > CAMERA_FB_SECOND_RANGE) {
        camera_state = CAM_STATE::UPLOADING;
        int size = 2 * CAMERA_FB_SECOND_RANGE;

        int record_start_index =
            ((record_start_time - CAMERA_FB_SECOND_RANGE) % size + size) % size;

        int record_end_index =
            ((record_start_time + CAMERA_FB_SECOND_RANGE - 1) % size + size) %
            size;

        Serial.printf("Sending Frames from %d to %d (center: %d)",
                      record_start_index, record_end_index, record_start_time);
        Serial.println();
        
        // Wait for the save buffer to be emptied before uploading
        while (uxQueueMessagesWaiting(CameraFBHTTPQ) != 0) {
          portYIELD();
        }
        upload_frames(record_start_index, record_end_index);

        camera_state = CAM_STATE::NORMAL;
        record_start_time = -1;
      }
    }

    fb = esp_camera_fb_get();
    if (fb == NULL) {
      Serial.println("Error capturing video buffer!");
    } else {
      frame_ptr = frame_alloc(fb, time_index, frame_index++);
      if (!frame_ptr) {
        Serial.println("Failed to allocate frame wrapper; returning fb");
        esp_camera_fb_return(fb);
      } else {
        if (xQueueSend(CameraFBSaveQ, &frame_ptr, 0) != pdPASS) {
          Serial.println("Frame dropped when passing it to save routine...");
          frame_release(frame_ptr);
        }
      }
    }

    if (camera_state == CAM_STATE::RECORDING && record_start_time == -1) {
      Serial.println("Begining Capture...");
      record_start_time = time_index;
    }

    vTaskDelayUntil(&prevTick, pdMS_TO_TICKS(1000 / CONFIG_CAMERA_FRAME_RATE));
  }
}

void camera_svc_save_task(void *pvParameters) {
  camera_frame_t *frame_ptr = NULL;
  int frame_count = 0;

  for (;;) {
    if (xQueueReceive(CameraFBSaveQ, &frame_ptr, portMAX_DELAY) == pdPASS) {
      if (!frame_ptr) continue;

      save_fb_to_sd(frame_ptr->fb, frame_ptr->time_index,
                    frame_ptr->frame_index);
      frame_count++;

      // Only send some frames to HTTP Task (since it is slower)
      if (frame_count >=
          (CONFIG_CAMERA_FRAME_RATE / CONFIG_CAMERA_STREAM_FRAME_DOWNSCALE)) {
        frame_count = 0;
        frame_ref(frame_ptr);
        if (xQueueSend(CameraFBHTTPQ, &frame_ptr, 0) != pdPASS) {
          Serial.println(
              "Frame dropped when passing it to http routine... undoing ref");
          frame_release(frame_ptr);
        }
        frame_release(frame_ptr);
      } else {
        frame_release(frame_ptr);
      }
    }
    portYIELD();
  }
}

void camera_svc_http_task(void *pvParameters) {
  static char url_buf[256];
  camera_frame_t *frame_ptr = NULL;
  String url;
  int resp;

  // Create url to use for API call
  memset(url_buf, 0, sizeof(url_buf));
  snprintf(url_buf, sizeof(url_buf), "http://%s:%d/api/device/stream?device=%s",
           coordinatorIP.toString().c_str(), coordinatorPort,
           deviceName.c_str());
  url = url_buf;

  for (;;) {
    if (xQueueReceive(CameraFBHTTPQ, &frame_ptr, portMAX_DELAY) == pdPASS) {
      if (!frame_ptr) continue;

      camera_fb_t *fb = frame_ptr->fb;

      http.begin(url);
      http.addHeader("Content-Type", "image/jpeg");

      http.setTimeout(CONFIG_HTTP_UPLOAD_TIMEOUT_MS);

      resp = http.PUT(fb->buf, fb->len);

      // Dont bother printing timeout errors
      if ((resp != HTTP_CODE_NO_CONTENT) &&
          (resp != HTTPC_ERROR_READ_TIMEOUT)) {
        String err = http.errorToString(resp);
        Serial.printf("HTTP error: %s (%d)\n", err.c_str(), resp);
      }

      http.end();

      frame_release(frame_ptr);
    }
  }
}

void camera_svc_event_task(void *pvParameters) {
  for (;;) {
    // Wait until notified
    timestamp = ulTaskNotifyTake(pdPASS, portMAX_DELAY);
    if (camera_state != CAM_STATE::NORMAL) {
      Serial.println("Camera is already busy. Blocking...");
      continue;
    }
    // Start recording
    Serial.printf("Got timestamp of event: %lu\n", timestamp);
    Serial.println();
    camera_state = CAM_STATE::RECORDING;
  }
}

static void _ensure_empty_dir(const char *path) {
  if (!SD_MMC.exists(path)) {
    SD_MMC.mkdir(path);
    return;
  }

  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) {
    Serial.printf("Failed to open existing directory: %s\n", path);
    return;
  }

  File entry;
  while ((entry = dir.openNextFile())) {
    String entryPath = String(path) + "/" + entry.name();
    if (entry.isDirectory()) {
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

  char dir_path[64];
  snprintf(dir_path, sizeof(dir_path), "%s/%d", CAMERA_FB_ROOT, time_index);

  static int last_time_index = -1;
  if (time_index != last_time_index) {
    _ensure_empty_dir(dir_path);
    last_time_index = time_index;
  }

  char file_path[96];
  snprintf(file_path, sizeof(file_path), "%s/%d.jpg", dir_path, frame_index);

  File file = SD_MMC.open(file_path, FILE_WRITE);
  if (!file) {
    Serial.printf("Failed to open file for writing: %s\n", file_path);
    return;
  }

  file.write(fb->buf, fb->len);
  file.close();
}


static void upload_frames(int start_index, int end_index) {
  
}