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

static const char *TAG = "example";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define INPUT_GPIO CONFIG_EXAMPLE_INPUT_GPIO


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

    // test to see if lights turnign on 
    // for(int i = 0; i < CONFIG_EXAMPLE_STRIP_LED_NUMBER; i++)
    //     {
    //         ESP_ERROR_CHECK(strip->set_pixel(strip, i, 255, 255, 255));
    //         ESP_ERROR_CHECK(strip->refresh(strip, 25));
    //     }

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
