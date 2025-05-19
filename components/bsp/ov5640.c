#include <stdio.h>
#include "esp_timer.h"
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "lvgl.h"
#include "esp_camera.h"
#include "ov5640.h"

#define PWDN_GPIO_NUM 17  // power down is not used
#define RESET_GPIO_NUM -1 // software reset will be performed
#define XCLK_GPIO_NUM 8
#define SIOD_GPIO_NUM 21
#define SIOC_GPIO_NUM 16

#define Y9_GPIO_NUM 2
#define Y8_GPIO_NUM 7
#define Y7_GPIO_NUM 10
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 11
#define Y4_GPIO_NUM 15
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 12
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 4
#define PCLK_GPIO_NUM 9


lv_obj_t *img_camera;

static SemaphoreHandle_t camera_mux = NULL;

extern bool lvgl_lock(int timeout_ms);
extern void lvgl_unlock(void);


//xTaskCreatePinnedToCore(camera_task, "camera_task_task", 1024 * 3, NULL, 1, NULL, 0);

void lvgl_camera_ui_init(lv_obj_t *parent)
{
    // lv_obj_t *obj = lv_obj_create(parent);
    // lv_obj_set_size(obj, lv_pct(100), lv_pct(100));
    img_camera = lv_img_create(parent);
    // lv_obj_set_size(img_camera, 240,);
    // lv_img_set_angle(img_camera, 900);
    lv_obj_align(img_camera, LV_ALIGN_CENTER, 0, 0); // 居中显示
    lv_obj_set_pos(img_camera, -1, 0);
    lv_obj_set_scroll_dir(parent, LV_DIR_NONE);

    lv_obj_set_style_pad_top(img_camera, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(img_camera, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(img_camera, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_right(img_camera, 0, LV_PART_MAIN);
}

void init_rgb_lut();
void camera_init(void)
{
    camera_mux = xSemaphoreCreateRecursiveMutex();
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_1;
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
    config.frame_size = FRAMESIZE_HVGA;
    config.pixel_format = PIXFORMAT_RGB565; // for streaming
    // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        printf("Camera init failed with error 0x%x", err);
        vTaskDelete(NULL);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_hmirror(s, 1);
    init_rgb_lut();
}

// 预计算5位/6位转8位的查表（LUT）
static uint8_t rgb5_to_8[32];  // 5位转8位 LUT
static uint8_t rgb6_to_8[64];   // 6位转8位 LUT

// 初始化LUT（只需初始化一次）
void init_rgb_lut() {
    for (int i = 0; i < 32; i++) {
        // 5位转8位：i * 255 / 31 ≈ (i << 3) | (i >> 2)
        rgb5_to_8[i] = (i << 3) | (i >> 2); 
    }
    for (int i = 0; i < 64; i++) {
        // 6位转8位：i * 255 / 63 ≈ (i << 2) | (i >> 4)
        rgb6_to_8[i] = (i << 2) | (i >> 4);
    }
}

void camera_task(void *param)
{

    camera_fb_t *pic;
    lv_img_dsc_t img_dsc;
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = 480;
    img_dsc.header.h = 320;
    img_dsc.data_size = 320 * 480 * 2;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data = NULL;
    // lv_img_set_src(img_camera, &pic);
    while (1)
    {
        if (xSemaphoreTakeRecursive(camera_mux, portMAX_DELAY) == pdTRUE)
        {
            pic = esp_camera_fb_get();

            if (NULL != pic)
            {
                img_dsc.data = pic->buf;
                if (lvgl_lock(-1))
                {
                    lv_img_set_src(img_camera, &img_dsc);
                    lvgl_unlock();
                }
            }
            esp_camera_fb_return(pic);
            xSemaphoreGiveRecursive(camera_mux);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    vTaskDelete(NULL);
}

void resize_bilinear(uint8_t* src, uint8_t* dst);



void rgb565_to_rgb888(uint8_t* src, uint8_t* dst, int pixel_count) {
    for (int i = 0; i < pixel_count; i++) {
        // 小端序处理：低字节在前，高字节在后
        uint16_t pixel = (src[2*i+1] << 8) | src[2*i]; // 修正字节序

        // 提取颜色分量并使用LUT转换
        uint8_t r = (pixel >> 11) & 0x1F;  // 5位红色
        uint8_t g = (pixel >> 5)  & 0x3F;  // 6位绿色
        uint8_t b = pixel & 0x1F;           // 5位蓝色

        // 使用查表法转换
        dst[3*i]   = rgb5_to_8[r];  // R
        dst[3*i+1] = rgb6_to_8[g];  // G
        dst[3*i+2] = rgb5_to_8[b];  // B
    }
}


void img_get_float(uint8_t image_width, uint8_t image_height, float *data_buf)
{
     uint8_t *rgb888_buf = (uint8_t *)heap_caps_malloc(480 * 320 * 3,MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *resize_buf = (uint8_t *)heap_caps_malloc(100 * 100 * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (xSemaphoreTakeRecursive(camera_mux, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGE("camera", "img_get_float");
        camera_fb_t *fb = esp_camera_fb_get();
        rgb565_to_rgb888(fb->buf, rgb888_buf, 480*320);
        resize_bilinear(rgb888_buf, resize_buf);
        for(int i = 0; i < 100 * 100 * 3; i++)
        {
            data_buf[i] = resize_buf[i] / 255.0f; // 归一化到[0,1]
        }
        esp_camera_fb_return(fb);
        xSemaphoreGiveRecursive(camera_mux);
    }
    free(rgb888_buf);
    free(resize_buf);
}

void img_get_int(uint8_t image_width, uint8_t image_height, int8_t *data_buf)
{
    uint8_t *rgb888_buf = (uint8_t *)heap_caps_malloc(480 * 320 * 3,MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *resize_buf = (uint8_t *)heap_caps_malloc(100 * 100 * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (xSemaphoreTakeRecursive(camera_mux, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGE("camera", "img_get_int");
        camera_fb_t *fb = esp_camera_fb_get();
        rgb565_to_rgb888(fb->buf, rgb888_buf, 480*320);
        resize_bilinear(rgb888_buf, resize_buf);
        for(int i = 0; i < 100 * 100 * 3; i++)
        {
            data_buf[i] = (int8_t)((resize_buf[i] - 127)); 
        }
        esp_camera_fb_return(fb);
        xSemaphoreGiveRecursive(camera_mux);
    }
    free(rgb888_buf);
    free(resize_buf);
}


// 输入：src (480x320 RGB888 数组)
// 输出：dst (100x100 RGB888 数组)
void resize_bilinear(uint8_t* src, uint8_t* dst) {
    const int src_width = 480;
    const int src_height = 320;
    const int dst_width = 100;
    const int dst_height = 100;

    float x_ratio = (float)(src_width - 1) / dst_width;
    float y_ratio = (float)(src_height - 1) / dst_height;

    for (int y = 0; y < dst_height; y++) {
        for (int x = 0; x < dst_width; x++) {
            // 计算浮点坐标
            float src_x = x * x_ratio;
            float src_y = y * y_ratio;

            // 取四个相邻像素
            int x1 = floor(src_x);
            int y1 = floor(src_y);
            int x2 = x1 + 1;
            int y2 = y1 + 1;

            // 边界检查
            if (x2 >= src_width) x2 = src_width - 1;
            if (y2 >= src_height) y2 = src_height - 1;

            // 计算权重
            float wx = src_x - x1;
            float wy = src_y - y1;

            // 对每个颜色通道插值
            for (int c = 0; c < 3; c++) {
                float val = 
                    (1 - wx) * (1 - wy) * src[(y1 * src_width + x1) * 3 + c] +
                    wx * (1 - wy) * src[(y1 * src_width + x2) * 3 + c] +
                    (1 - wx) * wy * src[(y2 * src_width + x1) * 3 + c] +
                    wx * wy * src[(y2 * src_width + x2) * 3 + c];
                
                dst[(y * dst_width + x) * 3 + c] = (uint8_t)val;
            }
        }
    }
}
