#include <stdio.h>
#include <string.h>
#include "tkjhat/sdk.h"
#include <FreeRTOS.h>
#include <pico/stdlib.h>
#include <queue.h>
#include <task.h>

#include "imu.h"
#include "wifi.h"

#define BUFFER_SIZE 256 

static char tx_buffer[BUFFER_SIZE];
static char rx_buffer[BUFFER_SIZE];

enum state { 
    idle,
    reading,
    transmitting,
    receiving,
    connecting
};
enum state programState = idle;





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
}

void idle_state(){
    // if(wifi_connect()) printf("connection failed \n");
    while(1){
        char input = read_IMU();
        if(input){
            programState = reading;
            reading_input(input);
        } 
    }
    
}

int main(){
    stdio_init_all();

    init_hat_sdk();
    load_calib_IMU();
    idle_state();
    return 0;
}