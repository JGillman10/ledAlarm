// #include <stdio.h>

// void app_main(void)
// {

// }

/* RMT example -- RGB LED Strip

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"
#include "driver/i2c.h"

static const char *TAG = "example";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define INPUT_GPIO CONFIG_EXAMPLE_INPUT_GPIO

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define RTC_SENSOR_ADDR                     0x68            /*!< Slave address of the real time clock */
#define RTC_SET_TIME_REGISTER               0X00
#define RTC_ALARM1_REGISTER                 0X07
#define RTC_STATUS_REGISTER                 0X0F

#define int_to_hex(x) (((x/10)%10) << 4) | (x%10)

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    // setup input gpio
    gpio_reset_pin(INPUT_GPIO);
    gpio_set_direction(INPUT_GPIO, GPIO_MODE_INPUT);

    // set up I2C communication
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    uint8_t set_alarm_buf[] = {RTC_ALARM1_REGISTER, 0x00, int_to_hex(CONFIG_ALARM1_MINUTES), int_to_hex(CONFIG_ALARM1_HOURS),  0x81};

    // set alarm
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, RTC_SENSOR_ADDR, set_alarm_buf, sizeof(set_alarm_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

    uint8_t clear_status_buf[] = {RTC_STATUS_REGISTER, 0x00};

    // clear status flags to let alarm trip
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, RTC_SENSOR_ADDR, clear_status_buf, sizeof(clear_status_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    



    // check gpio if alarm has tripped
    ESP_LOGI(TAG, "Alarm main loop start");
    while(true)
    {
        if(!gpio_get_level(INPUT_GPIO)) // if alarm tripped, gpio low
        {
            ESP_LOGI(TAG, "Alarm tripped!");
            for(int i = 0; i < CONFIG_EXAMPLE_STRIP_LED_NUMBER; i++)
            {
                ESP_ERROR_CHECK(strip->set_pixel(strip, i, 255, 255, 255));
                ESP_ERROR_CHECK(strip->refresh(strip, 25));
            }
        }

        // check alarm every minute
        for(int i = 0; i < 60; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }


    }

}
