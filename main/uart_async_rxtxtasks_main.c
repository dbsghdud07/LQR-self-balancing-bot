#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "math.h"

static const int RX_BUF_SIZE = 4096;
// static int64_t last_call_time = 0;

#define IMU_UART (UART_NUM_1)
#define IMU_TX (GPIO_NUM_43)
#define IMU_RX (GPIO_NUM_44)

#define PATTERN_CHR_NUM (2)

typedef struct {
    float roll, pitch, yaw; // 각
    float ax, ay, az; // 가속도
    float rx, ry, rz; // 각속도
} Imu;

Imu imu_data;

QueueHandle_t uart0_queue;
int tx_sendData(uart_port_t uart_num, const char* logName, const unsigned char* data, int len);
static void imu_configuration(void *arg);
static void imu_data_receiver(void *arg);
void process_data(uint8_t* data);

void init(void) {
    const uart_config_t imu_uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(IMU_UART, RX_BUF_SIZE, RX_BUF_SIZE, 10, &uart0_queue, 0);
    uart_param_config(IMU_UART, &imu_uart_config);
    uart_set_pin(IMU_UART, IMU_TX, IMU_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_enable_pattern_det_baud_intr(IMU_UART, '\n', 1, 15, 0, 0);
    // uart_enable_pattern_det_intr(IMU_UART, 0x0a, 1, 10000, 10, 10);
}

int tx_sendData(uart_port_t uart_num, const char* logName, const unsigned char* data, int len) {
    const int txBytes = uart_write_bytes(IMU_UART, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void imu_configuration(void *arg) {
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    // const unsigned char Tx_ang[] = {0x61, 0x6E, 0x67, 0x0D, 0x0A};  // Assuming ANG is the command to request IMU data
    // const unsigned char Tx_help[] = {0x68, 0x65, 0x6C, 0x70, 0x0D, 0x0A}; // help
    const unsigned char Tx_ss7[] = {0x73, 0x73, 0x3D, 0x37, 0x0D, 0x0A}; // ss=7 가속도, 각속도, 각도 데이터
    const unsigned char Tx_sp10[] = {0x73, 0x70, 0x3D, 0x32, 0x30, 0x0D, 0x0A}; // sp=10 10ms 의 주기로 데이터를 송신하겠다는 말
    const unsigned char Tx_baud_921600[] = {0x73, 0x62, 0x3D, 0x39, 0x32, 0x31, 0x36, 0x30, 0x30, 0x0D, 0x0A}; // baudrate를 921600으로 설정 
    const unsigned char Tx_baud_fw[] = {0x66, 0x77, 0x0D, 0x0A}; // 데이터를 끄기 전에 플래시에 저장

    int cnt = 0;
    while (cnt < 5) {
        tx_sendData(IMU_UART, TX_TASK_TAG, Tx_ss7, sizeof(Tx_ss7)); // 데이터 형태 설정
        vTaskDelay(500 / portTICK_PERIOD_MS);
        tx_sendData(IMU_UART, TX_TASK_TAG, Tx_sp10, sizeof(Tx_sp10)); // 데이터 형태 설정
        vTaskDelay(500 / portTICK_PERIOD_MS);
        tx_sendData(IMU_UART, TX_TASK_TAG, Tx_baud_921600, sizeof(Tx_baud_921600)); // 데이터 형태 설정
        vTaskDelay(500 / portTICK_PERIOD_MS);
        tx_sendData(IMU_UART, TX_TASK_TAG, Tx_baud_fw, sizeof(Tx_baud_fw)); // 데이터 형태 설정
        vTaskDelay(500 / portTICK_PERIOD_MS);
        cnt++;
    }
    vTaskDelete(NULL);
}

static void imu_data_receiver(void *arg) {
    uart_event_t event;
    size_t buffered_size;
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    for (;;) {
        if (xQueueReceive(uart0_queue, (void*)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_PATTERN_DET:
                    // 패턴 감지 시 데이터 읽기
                    uart_read_bytes(UART_NUM_1, data, 82, portMAX_DELAY);
                    data[82] = '\0';  // NULL 종료 문자 추가
                    process_data(data);  // 데이터 처리 함수
                    break;
                default:
                    break;
            }
        }
    }
    free(data);
}

void process_data(uint8_t* data) {
    // int64_t current_time = esp_timer_get_time();
    // if (last_call_time != 0) {
    //     int64_t time_diff = current_time - last_call_time;
    //     ESP_LOGI("process_data", "Time since last call: %lld microseconds", time_diff);
    // }
    // last_call_time = current_time;
    // 데이터 파싱 및 처리 로직
    char temp[9]; // 8 바이트 데이터 + NULL
    double values[9]; // 9개의 수치 데이터 저장
    char *token = strtok((char *)data, " "); // 첫 번째 토큰
    int idx = 0;

    while (token != NULL && idx < 9) {
        strncpy(temp, token, 8); // 데이터 복사
        temp[8] = '\0'; // NULL 종료
        values[idx++] = atof(temp); // 변환 후 저장
        token = strtok(NULL, " "); // 다음 토큰
        // printf("data: %f", values[idx]);
    }
}

void app_main(void) {
    init();
    // xTaskCreate(imu_configuration, "imu_configuration", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL); // 이 task 는 한번씩 필요할때만 configuration 하고싶을 때 실행시킨다.
    xTaskCreate(imu_data_receiver, "imu_data_receiver", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
    while(1)
    {
        vTaskDelay(1000);
    }
}
