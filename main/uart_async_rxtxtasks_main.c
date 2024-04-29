#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "math.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_43)
#define RXD_PIN (GPIO_NUM_44)

typedef struct {
    float roll, pitch, yaw; // 각
    float ax, ay, az; // 가속도
    float rx, ry, rz; // 각속도
} Imu;

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int tx_sendData(uart_port_t uart_num, const char* logName, const unsigned char* data, int len) {
    const int txBytes = uart_write_bytes(uart_num, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void imu_data_receiver(void *arg) {
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    const unsigned char Tx_ang[] = {0x61, 0x6E, 0x67, 0x0D, 0x0A};  // Assuming ANG is the command to request IMU data
    const unsigned char Tx_help[] = {0x68, 0x65, 0x6C, 0x70, 0x0D, 0x0A}; // help
    const unsigned char Tx_ss7[] = {0x73, 0x73, 0x3D, 0x37, 0x0D, 0x0A}; // ss=7 가속도, 각속도, 각도 데이터
    const unsigned char Tx_sp10[] = {0x73, 0x70, 0x3D, 0x31, 0x30, 0x0D, 0x0A}; // sp=10 10ms 의 주기로 데이터를 송신하겠다는 말

    tx_sendData(UART_NUM_1, TX_TASK_TAG, Tx_ss7, sizeof(Tx_ss7)); // 통신 속도 설정

    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    Imu imu_data;

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        ESP_LOGI(RX_TASK_TAG, "Received %d bytes: '%s'", rxBytes, data);
        if (rxBytes > 0) {
            data[rxBytes] = 0; // Null-terminate whatever we received and parse it

            if (data[0] == 'a' && data[1] == 'n' && data[2] == 'g') {
                char *ptr = strtok((char *)data, " ");
                int ang_count = 0;

                while (ptr != NULL) {
                    ang_count++;
                    ptr = strtok(NULL, " ");

                    if (ang_count == 1) {
                        imu_data.roll = atof(ptr);
                    } else if (ang_count == 2) {
                        imu_data.pitch = atof(ptr);
                    } else if (ang_count == 3) {
                        imu_data.yaw = atof(ptr);
                        break;
                    }
                }
                ESP_LOGI(RX_TASK_TAG, "roll = %f, pitch = %f, yaw = %f", imu_data.roll, imu_data.pitch, imu_data.yaw);
            }
        }
    }
    free(data);
}

void app_main(void) {
    init();
    xTaskCreate(imu_data_receiver, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}
