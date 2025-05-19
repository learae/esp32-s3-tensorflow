#include "cst816t_driver.h"
#include "driver/i2c.h"
//#include "driver/i2c_master.h"
#include "esp_log.h"

#define TOUCH_I2C_PORT      I2C_NUM_0

#define CST816T_ADDR    0x15

static const char *TAG = "cst816t";

//边界值
static uint16_t s_usLimitX = 0;
static uint16_t s_usLimitY = 0;

static esp_err_t i2c_read(uint8_t slave_addr, uint8_t register_addr, uint8_t read_len,uint8_t *data_buf);

/** CST816T初始化
 * @param cfg 配置
 * @return err
*/
esp_err_t cst816t_init(cst816t_cfg_t* cfg)
{
    i2c_port_t i2c_master_port = TOUCH_I2C_PORT;
    i2c_config_t conf = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = cfg->sda,
        .scl_io_num         = cfg->scl,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = cfg->fre,
    };

    s_usLimitX = cfg->x_limit;
    s_usLimitY = cfg->y_limit;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));
    
    uint8_t data_buf;
    i2c_read(CST816T_ADDR, 0xA7, 1,&data_buf);
    ESP_LOGI(TAG, "\tChip ID: 0x%02x", data_buf);

    i2c_read(CST816T_ADDR, 0xA9, 1,&data_buf);
    ESP_LOGI(TAG, "\tFirmware version: 0x%02x", data_buf);

    i2c_read(CST816T_ADDR, 0xAA, 1,&data_buf);
    ESP_LOGI(TAG, "\tFactory ID: 0x%02x", data_buf);

    return ESP_OK;
}

/** 读取坐标值
 * @param  x x坐标
 * @param  y y坐标
 * @param state 松手状态 0,松手 1按下
 * @return 无
*/
void cst816t_read(int16_t *x,int16_t *y,int *state)
{
    uint8_t data_x[2];        // 2 bytesX
    uint8_t data_y[2];        // 2 bytesY
    uint8_t touch_pnt_cnt = 0;        // Number of detected touch points
    static int16_t last_x = 0;  // 12bit pixel value
    static int16_t last_y = 0;  // 12bit pixel value
    i2c_read(CST816T_ADDR, 0x02,1, &touch_pnt_cnt);
    if (touch_pnt_cnt != 1) {    // ignore no touch & multi touch
        *x = last_x;
        *y = last_y;
        *state = 0;
        return;
    }

    //读取X坐标
    i2c_read(CST816T_ADDR,0x03,2,data_x);

    //读取Y坐标
    i2c_read(CST816T_ADDR,0x05,2,data_y);

    int16_t current_x = ((data_x[0] & 0x0F) << 8) | (data_x[1] & 0xFF);
    int16_t current_y = ((data_y[0] & 0x0F) << 8) | (data_y[1] & 0xFF);

    if(last_x != current_x || current_y != last_y)
    {
        last_x = current_x;
        last_y = current_y;
        //ESP_LOGI(TAG,"touch x:%d,y:%d",last_x,last_y);
    }


    if(last_x >= s_usLimitX)
        last_x = s_usLimitX - 1;
    if(last_y >= s_usLimitY)
        last_y = s_usLimitY - 1;

    *x = last_x;
    *y = last_y;
    *state = 1;
}




/** 根据寄存器地址读取N字节
 * @param slave_addr 器件地址
 * @param register_addr 寄存器地址
 * @param read_len  要读取的数据长度
 * @param data_buf 数据
 * @return err
*/
static esp_err_t i2c_read(uint8_t slave_addr, uint8_t register_addr, uint8_t read_len,uint8_t *data_buf) 
{
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    if(!i2c_cmd)
    {
        ESP_LOGE(TAG, "Error i2c_cmd creat fail!");
        return ESP_FAIL;
    }
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, register_addr, I2C_MASTER_ACK);

    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (slave_addr << 1) | I2C_MASTER_READ, true);
    for(int i = 0;i < read_len;i++)
    {
        if(i == read_len - 1)
            i2c_master_read_byte(i2c_cmd, &data_buf[i], I2C_MASTER_NACK);
        else
            i2c_master_read_byte(i2c_cmd, &data_buf[i], I2C_MASTER_ACK);
    }
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, i2c_cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(i2c_cmd);
    return ret;
}
