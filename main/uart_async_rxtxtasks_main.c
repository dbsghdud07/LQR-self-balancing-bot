#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "math.h"

// static int64_t last_call_time = 0;

#define IMU_UART (UART_NUM_1)
#define IMU_TX (GPIO_NUM_43)
#define IMU_RX (GPIO_NUM_44)
#define RX_BUF_SIZE 4096
#define BUFFER_SIZE 100

#define PATTERN_CHR_NUM (1)

typedef struct {
    float roll, pitch, yaw; // 각
    float ax, ay, az; // 가속도
    float rx, ry, rz; // 각속도
} Imu;

Imu imu_data;
volatile char dataBuffer[BUFFER_SIZE];
volatile int bufferIndex = 0;

float values[9];
int count = 0;

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
    // uart_enable_pattern_det_baud_intr(IMU_UART, '\n', 1, 15, 0, 0);
    uart_enable_pattern_det_baud_intr(IMU_UART, '\n', PATTERN_CHR_NUM, 10000,0,0);  // '\n' 패턴 감지 활성화
    uart_pattern_queue_reset(IMU_UART, 20);  // 패턴 큐 사이즈 설정
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


// IMU 데이터를 UART에서 읽고 처리
void imu_data_receiver(void *pvParameters) {
    uart_event_t event;
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE);
    while (true) {
        // 이벤트 큐에서 이벤트 기다리기
        if (xQueueReceive(uart0_queue, (void*)&event, portMAX_DELAY)) {
            bzero(data, RX_BUF_SIZE);
            switch (event.type) {
                // 패턴 감지 이벤트 처리
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(IMU_UART, &event.size);
                    int pos = uart_pattern_pop_pos(IMU_UART);
                    uart_read_bytes(IMU_UART, data, pos + 1, 100 / portTICK_PERIOD_MS);
                    data[pos] = '\0';  // NULL 종료 문자 추가
                    // ESP_LOGI("UART", "Received: %s, %d", data, pos);
                    process_data(data);
                    break;
                default:
                    ESP_LOGI("UART", "Unhandled event type: %d", event.type);
                    break;
            }
        }
    }
    free(data);
}

void process_data(uint8_t* data) {
    count = sscanf((char*)data, "%8f%8f%8f%8f%8f%8f%8f%8f%8f",
                   &values[0], &values[1], &values[2], &values[3], &values[4],
                   &values[5], &values[6], &values[7], &values[8]);

    printf("Parsed data (%d elements):\n", count);
    for(int i = 0; i < count; i++) {
        printf("%.3f ", values[i]);
    }
    printf("\n");
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