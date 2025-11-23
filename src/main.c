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
 
 
// State Machine
typedef enum {
    STATE_IDLE, 
    STATE_RECEIVING_IMU,
    STATE_RECEIVING_SERIAL,
    STATE_SENDING,
    STATE_CALIBRATING
} ProgramState;
 
// Possible Event Types
typedef enum {
    EVENT_SYMBOL_IMU_RECEIVED,
    EVENT_SYMBOL_SERIAL_RECEIVED,
    EVENT_EOM,
    EVENT_INITIATE_CALIBRATION,
    EVENT_SEND_MSG
} EventType;
 
// Event Structure
typedef struct {
    EventType event;
    char data;   // any kind of data that we might want to pass with the event if any
} ProgramEvent;
 
 
// tasks handles
TaskHandle_t poll_IMU_handle = NULL;
TaskHandle_t write_serial_handle = NULL;
TaskHandle_t state_machine_handle = NULL;
TaskHandle_t poll_serial_handle = NULL;
 
// events queue handle
QueueHandle_t eventQueue;
 
/* Prototypes */
int check_for_eom();
 
 
void task_poll_imu(void *pvParameters){
    for(;;){
        char imu_char = read_IMU();
        if(imu_char != 'X' && imu_char != '\0'){
            ProgramEvent event = {EVENT_SYMBOL_IMU_RECEIVED, imu_char};
            xQueueSend(eventQueue, &event, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
 
void task_poll_serial(void *pvParameters){
    for(;;){
        char serial_char = (char)getchar_timeout_us(0);
        if(serial_char == ' ' || serial_char == '.' || serial_char == '-'){
            ProgramEvent event = {EVENT_SYMBOL_SERIAL_RECEIVED, serial_char};
            xQueueSend(eventQueue, &event, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
 
void calibrate_imu() {
    taskENTER_CRITICAL();
    gpio_set_irq_enabled(SW2_PIN, GPIO_IRQ_EDGE_RISE, false);
    set_calib_IMU();
    gpio_set_irq_enabled(SW2_PIN, GPIO_IRQ_EDGE_RISE, true);
    taskEXIT_CRITICAL();
}
 
void calib_handler(uint gpio, uint32_t event_mask){
    ProgramEvent ev = { EVENT_INITIATE_CALIBRATION, '\0' };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
 
    // Send event to queue from ISR
    xQueueSendFromISR(eventQueue, &ev, &xHigherPriorityTaskWoken);
 
    // Yield if a higher-priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
 
 
// writes message to serial
void task_write_serial(void *pvParameters){
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
 
 
// processes events from queue
void task_stateMachine(void *pvParameters){
    ProgramEvent event;
    ProgramState state = STATE_IDLE;
 
    for(;;){
        if(xQueueReceive(eventQueue, &event, portMAX_DELAY)){
            switch(state){
                case STATE_IDLE:
                    if(event.event == EVENT_SYMBOL_IMU_RECEIVED){
                        state = STATE_RECEIVING_IMU;
                        // buffering input from imu
                        tx_buffer[tx_buffer_index++] = event.data;
                    }
                    else if (event.event == EVENT_SYMBOL_SERIAL_RECEIVED){
                        state = STATE_RECEIVING_SERIAL;
                        // buffering input from serial
                        rx_buffer[rx_buffer_index++] = event.data;
                    }
                    else if (event.event == EVENT_INITIATE_CALIBRATION){
                        state = STATE_CALIBRATING;
                        calibrate_imu();
                        state = STATE_IDLE;
                    }
                    break;
 
                case STATE_RECEIVING_IMU:
                    if(event.event == EVENT_SYMBOL_IMU_RECEIVED){
                        // append imu symbol to buffer
                        tx_buffer[tx_buffer_index++] = event.data;
                        
                        // checks for eom
                        if(check_for_eom(tx_buffer, tx_buffer_index)){
                            state = STATE_SENDING;
                            // notifies the write to serial task
                            xTaskNotifyGive(write_serial_handle);
 

 
                            state = STATE_IDLE;
                        }
                    }
                    if(event.event == EVENT_SYMBOL_SERIAL_RECEIVED){
                        // append serial symbol to buffer
                        char serial_char = event.data;
                        do{
                            led_blink_char(serial_char);
                            // clear_display();
                            // write_text(&serial_char);
                            rx_buffer[rx_buffer_index++] = serial_char;
                            serial_char = (char)getchar_timeout_us(0);
                        } while(serial_char == ' ' || serial_char == '.' || serial_char == '-');
                        
                        // checks   for eom
                        if(check_for_eom(rx_buffer, rx_buffer_index)){
                            // whatever we need to do after eom for serial
                            state = STATE_IDLE;
                            printf("%s ---  %s\n", rx_buffer, morse_to_text(rx_buffer));
                            // resets rx buffer
                            memset(rx_buffer, 0, sizeof(rx_buffer));
                            rx_buffer_index = 0;
                        }
                    }
                    break;
 
                case STATE_RECEIVING_SERIAL:
                    if(event.event == EVENT_SYMBOL_SERIAL_RECEIVED){
                        // append serial symbol to buffer
                        char serial_char = event.data;
                        do{
                            led_blink_char(serial_char);
                            // clear_display();
                            // write_text(&serial_char);
                            rx_buffer[rx_buffer_index++] = serial_char;
                            serial_char = (char)getchar_timeout_us(0);
                        } while(serial_char == ' ' || serial_char == '.' || serial_char == '-');
                    
                        // checks   for eom
                        if(check_for_eom(rx_buffer, rx_buffer_index)){
                            // whatever we need to do after eom for serial
                            state = STATE_IDLE;
 
                            // resets rx buffer
                            memset(rx_buffer, 0, sizeof(rx_buffer));
                            rx_buffer_index = 0;
                        }
                    }
                    break;
 
                case STATE_SENDING:
                    if(event.event == EVENT_SEND_MSG){
                        // send message
                        state = STATE_IDLE;
                    }
                    break;
                case STATE_CALIBRATING:
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
 
    xTaskCreate(task_poll_imu, "Poll IMU Task", TASK_STACK, NULL, 1, &poll_IMU_handle);
    xTaskCreate(task_poll_serial, "Poll Serial Task", TASK_STACK, NULL, 1, &poll_serial_handle);
    xTaskCreate(task_stateMachine, "State Machine Task", TASK_STACK, NULL, 2, &state_machine_handle);
    xTaskCreate(task_write_serial, "write serial Task", TASK_STACK, NULL, 2, &write_serial_handle);

 
    vTaskStartScheduler();
}