#ifndef _CST816T_DRIVER_H_
#define _CST816T_DRIVER_H_
#include "driver/gpio.h"
#include "esp_err.h"


//CST816T 触摸IC驱动

typedef struct 
{
    gpio_num_t  scl;     //SCL管脚
    gpio_num_t  sda;     //SDA管脚
    uint32_t    fre;       //I2C速率
    uint16_t    x_limit;    //X方向触摸边界
    uint16_t    y_limit;    //y方向触摸边界
}cst816t_cfg_t;


/** CST816T初始化
 * @param cfg 配置
 * @return err
*/
esp_err_t   cst816t_init(cst816t_cfg_t* cfg);

/** 读取坐标值
 * @param  x x坐标
 * @param  y y坐标
 * @param state 松手状态 0,松手 1按下
 * @return 无
*/
void cst816t_read(int16_t *x,int16_t *y,int *state);

#endif
