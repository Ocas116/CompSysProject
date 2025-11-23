#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <pico/stdlib.h>
#include <task.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "tkjhat/sdk.h"
#include "imu.h"
#include "led.h"
#include "morse_utils.h"
// memory stack size per task
#define TASK_STACK 512



#define BUFFER_SIZE 256 

static char tx_buffer[BUFFER_SIZE];
static int tx_buffer_index = 0;
static char rx_buffer[BUFFER_SIZE];
static int rx_buffer_index = 0;
// Possible Event Types
typedef enum {
    IMU_MOVED,
    SEND_MSG,
    SYMBOL_DETECTED,
    RECEIVE,
    CALIBRATING,
    POLL
} EventType;

// Event Structure
typedef struct {
    EventType event;
    char data;   // any kind of data that we might want to pass with the event if any
} ProgramEvent;


// tasks handles
TaskHandle_t readIMU_handle = NULL;
TaskHandle_t write_serial_handle = NULL;
TaskHandle_t stateMachine_handle = NULL;
TaskHandle_t receive_serial_handle = NULL;
TaskHandle_t calibration_handle = NULL;
TaskHandle_t poll_handle = NULL;

// events queue handle
QueueHandle_t eventQueue;

/* Prototypes */
int check_for_eom();
int counter1 = 0;
void task_poll(void *pvParameters) {
    int counter = 0;
    for(;;){
        //read IMU input
        char input = read_IMU();
        //check if input is valid
        if(input != 'X' && input != '\0'){
            tx_buffer[tx_buffer_index++] = input;
            printf("received char: %c\n", input);
        }
        
        if(check_for_eom(tx_buffer, tx_buffer_index)){
            printf("Final %s", tx_buffer);
            xTaskNotifyGive(write_serial_handle);
            memset(tx_buffer, 0, sizeof(tx_buffer));
            tx_buffer_index = 0;
        }
        if(counter % 10 == 0) xTaskNotifyGive(receive_serial_handle);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
void task_calibrate(void *pvParameters) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //printf("in task calibrate\n");
        gpio_set_irq_enabled(SW2_PIN, GPIO_IRQ_EDGE_RISE, false);
        set_calib_IMU();
        gpio_set_irq_enabled(SW2_PIN, GPIO_IRQ_EDGE_RISE, true);
    }
}
void calib_handler(uint gpio, uint32_t event_mask){
    xTaskNotifyGive(calibration_handle);
}


//receive data task 
void task_receive_serial(void *pvParameters){
    for(;;){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //printf("In task_receive\n");
        // printf("rx_buffer[%d] %s\n",rx_buffer_index ,  rx_buffer);
        char input;
            do {
                input = (char)getchar_timeout_us(0);
                if(input == ' ' || input == '.' || input == '-'){
                rx_buffer[rx_buffer_index++] = input;
                printf("Message %s", rx_buffer);
                led_blink_char(input);
                printf("rx_buffer[%d] %s\n", rx_buffer_index, rx_buffer);
                }
            } while(input == ' ' || input == '.' || input == '-');
        if(check_for_eom(rx_buffer, rx_buffer_index)){
            printf("Final buffer: %s\n in text %s", rx_buffer, morse_to_text(rx_buffer));
            for(int i = 0;i < rx_buffer_index;i++){
                delay_ms(500);
                led_blink_char(rx_buffer[i]);
            }
            xTaskNotifyGive(write_serial_handle);
            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_buffer_index = 0;
        } 
    }
}

// writes message to serial
void task_writeSerial(void *pvParameters){
    
    for(;;){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //printf("In task_write\n");
        int timeout = 0;
        while (!stdio_usb_connected() && (timeout < 50)) {
        timeout++;
        sleep_ms(100);
        }
        printf("%s", tx_buffer);
        memset(tx_buffer, 0, sizeof(tx_buffer));
        tx_buffer_index = 0;
    }
}

// reads IMU data
void task_readIMU(void *pvParameters){

    for(;;){
    // tasks waits until notified
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(check_for_eom(tx_buffer, tx_buffer_index)){
        printf("Final %s", tx_buffer);
        memset(tx_buffer, 0, sizeof(tx_buffer));
        tx_buffer_index = 0;
    }
    vTaskDelay(200);
    char symbol = read_IMU();
    if(symbol == 'X' || symbol == '\0') continue;
    printf("char read: %c\n", symbol);
    // creates event for symbol being read from IMU
    ProgramEvent programEvent;
    programEvent.event = SYMBOL_DETECTED;
    tx_buffer[tx_buffer_index++] = symbol;
    // sends created event to queue
    vTaskDelay(200);
    xQueueSend(eventQueue, &programEvent, portMAX_DELAY);
    }
}

// processes events from queue
void task_stateMachine(void *pvParameters){
    ProgramEvent event;
    
    for(;;){
        // printf("In task_state\n");
        if(xQueueReceive(eventQueue, &event, portMAX_DELAY)){
            // printf("eventnum: %d\n", event.event);
            switch(event.event){
                case IMU_MOVED:
                    xTaskNotifyGive(readIMU_handle);
                    break;
                case SYMBOL_DETECTED:
                    printf("tx_buffer[%d] %s\n", tx_buffer_index, tx_buffer);
                    // append symbol (event.data) to buffer
                    // check for End of Message (3 spaces)
                    if(check_for_eom(tx_buffer, tx_buffer_index))
                    {

                            ProgramEvent programEvent;
                            programEvent.event = SEND_MSG;
                            // sends symbol as .data
                            programEvent.data = '\0';

                            // sends created event to queue
                            xQueueSend(eventQueue, &programEvent, portMAX_DELAY);
                    }
                    break;
                case SEND_MSG:
                    // initiates the serial writing task
                    xTaskNotifyGive(write_serial_handle);
                    break;
            }
        }
    }
}

// checks if received End Of Message sequence from user input
int check_for_eom(char *buffer, int len){
    if(len < 4) return 0;
    if((buffer[len-1] == ' ') && (buffer[len-2] == ' ') && (buffer[len-3] == ' ')) return 1;
    return 0;
}

// setup function
int main() {
    stdio_init_all();
    init_hat_sdk();
    init_display();
    IMU_init();
    init_red_led();
    led_blink_eom();
    // might need to enable pullup resistor
    gpio_init(SW2_PIN);
    gpio_set_dir(SW2_PIN, GPIO_IN); // set pin as input
    // ties pin interrupt to a callback function
    gpio_set_irq_enabled_with_callback(SW2_PIN, 
        GPIO_IRQ_EDGE_RISE , // which events trigger the interrupt
        true,
        calib_handler // pointer to isr function
    );

    eventQueue = xQueueCreate(10, sizeof(ProgramEvent));

    BaseType_t res_a = xTaskCreate(task_readIMU, "Read IMU Task", TASK_STACK, NULL, 2, &readIMU_handle);
    BaseType_t res_b = xTaskCreate(task_writeSerial, "Write Serial Task", TASK_STACK, NULL, 1, &write_serial_handle);
    BaseType_t res_c = xTaskCreate(task_stateMachine, "State Machine Task", TASK_STACK, NULL, 2, &stateMachine_handle);
    BaseType_t res_d = xTaskCreate(task_receive_serial, "Receive Task", TASK_STACK, NULL, 2, &receive_serial_handle);
    BaseType_t res_e = xTaskCreate(task_calibrate, "Calibration Task", TASK_STACK, NULL, 2, &calibration_handle);
    BaseType_t res_f = xTaskCreate(task_poll, "Poll Task", TASK_STACK, NULL, 2, &poll_handle);
    vTaskStartScheduler();
}
