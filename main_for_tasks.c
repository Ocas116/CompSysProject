#include <stdio.h>

#include <FreeRTOS.h>
#include <pico/stdlib.h>
#include <task.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

// memory stack size per task
#define TASK_STACK 512

// pin used for IMU interrupt
#define IMU_INTERRUPT_PIN -1

// Possible Event Types
typedef enum {
    IMU_MOVED,
    SEND_MSG,
    SYMBOL_DETECTED,
} EventType;

// Event Structure
typedef struct {
    EventType event;
    char data;   // any kind of data that we might want to pass with the event if any
} ProgramEvent;


// tasks handles
TaskHandle_t readIMU_handle = NULL;
TaskHandle_t writeSerial_handle = NULL;
TaskHandle_t stateMachine_handle = NULL;

// events queue handle
xQueueHandle_t eventQueue;

// triggered by voltage change on IMU pin
// notifies task_readIMU to read the IMU data
// DO NOT PROCESS STUFF HERE
void interrupt_IMU(uint gpio, uint32_t events){
    // flag for telling freeRTOS if there is a higher priority (urgent) task
    // that has been unblocked and needs to be switched to right after current interrupt
    // flag value is set by freertos itself
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // creates event for imu being moved
    ProgramEvent programEvent;
    programEvent.event = IMU_MOVED;
    programEvent.data = 1;

    // sends created event to queue
    xQueueSendFromISR(eventQueue, &programEvent, &xHigherPriorityTaskWoken);

    // passes the flag to freertos and tells freertos
    // whether to switch to high priority task
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// writes message to serial
void task_writeSerial(void *pvParameters){
    for(;;){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // code for writing to serial
    }
}

// reads IMU data
void task_readIMU(void *pvParameters){
    for(;;){
    // tasks waits until notified
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // IMU Reading + Decoding
    // ...
    // get the symbol var

    // creates event for symbol being read from IMU
    ProgramEvent programEvent;
    programEvent.event = SYMBOL_DETECTED;
    // sends symbol as .data
    programEvent.data = symbol;

    // sends created event to queue
    xQueueSend(eventQueue, &programEvent, portMAX_DELAY);
    }
}

// processes events from queue
void task_stateMachine(void *pvParameters){
    ProgramEvent event;

    for(;;){
        if(xQueueReceive(eventQueue, &event, portMAX_DELAY)){
            switch(event.event){
                case IMU_MOVED:
                    xTaskNotifyGive(readIMU_handle);
                    break;
                case SYMBOL_DETECTED:
                    // append symbol (event.data) to buffer
                    // check for End of Message (3 spaces)
                    if(check_for_eom())
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
                    xTaskNotifyGive(writeSerial_handle);
                    break;
            }
        }
    }
}

// checks if received End Of Message sequence from user input
bool check_for_eom(){
    // reads last 3 chars from buffer
    // ...
}

// setup function
int main() {
    stdio_init_all();

    // interrupt pin configuration
    gpio_init(IMU_INTERRUPT_PIN);
    gpio_set_dir(IMU_INTERRUPT_PIN, GPIO_IN); // set pin as input
    // might need to enable pullup resistor

    // ties pin interrupt to a callback function
    gpio_set_irq_with_callback(IMU_INTERRUPT_PIN, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, // which events trigger the interrupt
        true,
        &interrupt_IMU // pointer to isr function
    );

    eventQueue = xQueueCreate(10, sizeof(ProgramEvent));

    BaseType_t res_a = xTaskCreate(task_readIMU, "Read IMU Task", TASK_STACK, NULL, 2, &readIMU_handle);
    BaseType_t res_b = xTaskCreate(task_writeSerial, "Write Serial Task", TASK_STACK, NULL, 1, &writeSerial_handle);
    BaseType_t res_c = xTaskCreate(task_stateMachine, "State Machine Task", TASK_STACK, NULL, 2, &stateMachine_handle);
    
    vTaskStartScheduler();
}
