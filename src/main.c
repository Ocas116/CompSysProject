#include <stdio.h>
#include <string.h>
#include "tkjhat/sdk.h"
#include <FreeRTOS.h>
#include <pico/stdlib.h>
#include <queue.h>
#include <task.h>
#include <pico/cyw43_arch.h>
#include "imu.h"
#include "wifi.h"
#include <time.h>

#define BUFFER_SIZE 256 

static char tx_buffer[BUFFER_SIZE];
static char rx_buffer[BUFFER_SIZE];

enum state { 
    idle,
    reading,
    transmitting,
    receiving,
    connecting,
};
enum state programState = idle;


int flag = 0;


void reading_input(char first){
    int flag = 0;
    int i = 0;
    tx_buffer[i++] = first;
    while(flag < 3){
        char input = read_IMU();
        if(input) tx_buffer[i++] = input;
        else continue;
        if(input == ' ') flag++;
        else flag = 0;
    }
    //if sending to state machine
    printf("%s", tx_buffer);
}

void idle_state(){
    // if(wifi_connect()) printf("connection failed \n");
    while(1){

        if(gpio_get(SW2_PIN)) set_calib_IMU();

        char input = read_IMU();
        if(input){
            programState = reading;
            reading_input(input);
        }
        input = (char)getchar_timeout_us(0);
        if(input){
            programState = receiving;
            int i = 0;
            while(input){
                rx_buffer[i] = input;
                i++;
                input = (char)getchar_timeout_us(0);
            }
        }
    }
    }

int pico_led_init(void) {
    return cyw43_arch_init();
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}
int main(){
    stdio_init_all();
    init_hat_sdk();
    pico_led_init();
    int flagss = 0;
    int inputs = 2;
    float ax = 0, ay = 0, az = 0 , gx, gy, gz, t;
    int res = init_ICM42670();
    if(res < 0){
        printf("Error intializing error: %d", res);
        return 1;
    }
     if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    while(1) 
    {
    tx_buffer[0] = 'M';
    tx_buffer[1] = 'C';
        int inputs = 2;
        int flagss = 0;
    while(flagss < 3){
    if(gpio_get(SW2_PIN)){
        set_calib_IMU();
    }
    /*
    if(flagss % 10 == 0){
        if(0 > ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       printf("Error reading sensor data");
    }
    printf("x: %d,y: %d,z: %d\n", (int)(ax*1000), (int)(ay*1000), (int)(az*1000));
    }
    */
    //  printf("x: %d,y: %d,z: %d\n", (int)(ax*1000), (int)(ay*1000), (int)(az*1000));
    char chart = read_IMU();
    if(chart != 'X') {
        tx_buffer[inputs++] = chart;
        printf("%c", chart);
        cyw43_delay_ms(200);
        if(chart == '_') flagss++;
        else flagss = 0;

    }
    //printf("readIMU() val: %c, %d\n", tx_buffer[inputs-1], gpio_get(SW2_PIN));
    //printf("Buffer[%d] %s\n", inputs, tx_buffer);
    cyw43_delay_ms(10);
}
    printf("\nBuffer[%d] %s\n", inputs, tx_buffer);
    for(int i=0; i< BUFFER_SIZE; i++) tx_buffer[i] = '\0';
    delay_ms(5000);
}
    idle_state();
    return 0;
}