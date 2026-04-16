#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

// micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

static const char *TAG = "MICRO_ROS";

// Пин светодиода (GPIO2 на большинстве ESP32)
#define LED_PIN GPIO_NUM_2

// Глобальные переменные
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 sensor_msg;
std_msgs__msg__Int32 led_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int counter = 0;

// ==============================================
// ТАЙМЕР: публикует счётчик каждые 500 мс
// ==============================================
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        sensor_msg.data = counter++;
        rcl_publish(&publisher, &sensor_msg, NULL);
        ESP_LOGI(TAG, "Published: %d", sensor_msg.data);
    }
}

// ==============================================
// ПОДПИСЧИК: управляет светодиодом по команде
// ==============================================
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    ESP_LOGI(TAG, "Received command: %d", msg->data);
    
    if (msg->data == 1) {
        gpio_set_level(LED_PIN, 1);
        ESP_LOGI(TAG, "LED ON");
    } else if (msg->data == 0) {
        gpio_set_level(LED_PIN, 0);
        ESP_LOGI(TAG, "LED OFF");
    }
}

// ==============================================
// ОСНОВНАЯ ЗАДАЧА micro-ROS
// ==============================================
void micro_ros_task(void *pvParameters) {
    // 1. Инициализация NVS (нужна для сетевых настроек)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. Настройка светодиода
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PIN),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);
    
    // 3. Инициализация micro-ROS
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    
    // 4. Создание узла
    rclc_node_init_default(&node, "esp32_node", "", &support);
    
    // 5. Издатель (топик /sensor_data)
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/sensor_data");
    
    // 6. Подписчик (топик /led_command)
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/led_command");
    
    // 7. Таймер (500 мс)
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(500),
        timer_callback);
    
    // 8. Executor (обработчик событий)
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &led_msg, &subscription_callback, ON_NEW_DATA);
    
    ESP_LOGI(TAG, "micro-ROS node started");
    ESP_LOGI(TAG, "Publishing to /sensor_data, subscribed to /led_command");
    
    // 9. Основной цикл
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==============================================
// ТОЧКА ВХОДА
// ==============================================
void app_main(void) {
    ESP_LOGI(TAG, "Starting micro-ROS on ESP32");
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 5, NULL);
}