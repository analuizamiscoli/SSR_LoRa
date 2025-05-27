#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "math.h"

#define UART_PORT UART_NUM_0
#define UART_PORT_LORA UART_NUM_2
#define BUF_SIZE 4096

typedef struct __attribute__((packed, aligned(1))) {
    int32_t time;
    uint32_t status;
    float pressure;
    float temperature;
    float bmp_altitude;
    float max_altitude;
    float accel_x;
    float accel_y;
    float accel_z;
    float rotation_x;
    float rotation_y;
    float rotation_z;
    float latitude;
    float longitude;
    float gps_altitude;
    float voltage;
} data_t;

void init_uart() {
    printf("Configurando UART...\n");

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_LORA, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_LORA, BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI("UART", "UART configurada com sucesso.");
    printf("UART inicializada com sucesso!\n");
}

void lora_receive_task(void *pvParameters) {
    data_t received_data;
    uint8_t rx_buffer[sizeof(data_t)];
    int len;

    printf("Aguardando dados do LoRa...\n");

    while (1) {
        len = uart_read_bytes(UART_PORT_LORA, rx_buffer, sizeof(data_t), pdMS_TO_TICKS(1000));
        if (len == sizeof(data_t)) {
            memcpy(&received_data, rx_buffer, sizeof(data_t));
            printf("Recebido: Time: %ld, Status: %lu, V: %.2f, Alt: %.2f, Lat: %.5f, Lon: %.5f\n",
                received_data.time, received_data.status, received_data.voltage,
                received_data.bmp_altitude, received_data.latitude, received_data.longitude);

            // Envia os dados recebidos para o supervisório via UART
            uart_write_bytes(UART_PORT, (const char*)&received_data, sizeof(data_t));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main() {
    printf("Inicializando armazenamento NVS...\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("NVS cheio ou com versão incompatível. Apagando...\n");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("NVS inicializado com sucesso.\n");

    init_uart();

    printf("Criando tarefa de recepção LoRa...\n");
    xTaskCreate(lora_receive_task, "lora_receive_task", 4096, NULL, 5, NULL);
}