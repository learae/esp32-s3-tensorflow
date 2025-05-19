#ifndef _ov5640_h_
#define _ov5640_h_

#include "esp_camera.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

void camera_init(void);
void lvgl_camera_ui_init(lv_obj_t *parent);
void camera_task(void *param);
void img_get_float(uint8_t image_width, uint8_t image_height, float *data_buf);
void img_get_int(uint8_t image_width, uint8_t image_height, int8_t *data_buf);
#ifdef __cplusplus
}
#endif

#endif