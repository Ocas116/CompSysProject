#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include "hardware/flash.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tkjhat/sdk.h>
#include "imu.h"


#define CALIB_VALS_OFFSET (1024 * 1024)

static float ax_threshold = 0, ay_threshold = 0, az_threshold = 0;


typedef struct {
    float a, b, c;
} calib_vals;

void init_IMU_int(){
    printf("test");
}
void load_calib_IMU(){
    const calib_vals *data = (const calib_vals *)(XIP_BASE + CALIB_VALS_OFFSET);
    ax_threshold = data->a;
    ay_threshold = data->b;
    az_threshold = data->c;
}

int set_calib_IMU(){
    int flag = 0;
    float ax, ay, az, gx, gy, gz, t;
    if(!init_ICM42670()){
        printf("Error intializing");
        return 1;
    } 
    while(!flag){
        if(!ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       printf("Error reading sensor data");
       return 1;
    }
    if(ax_threshold < ax) ax_threshold = ax;
    if(ay_threshold < ay) ay_threshold = ay;
    if(az_threshold < az) az_threshold = az;
    flag = gpio_get(SW2_PIN);
    }
    calib_vals data = {ax_threshold, ay_threshold, az_threshold};

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CALIB_VALS_OFFSET, 4096);
    flash_range_program(CALIB_VALS_OFFSET, (const uint8_t*)&data, sizeof(data));
    restore_interrupts(ints);
    return 0;
}
char read_IMU(){
    float ax, ay, az, gx, gy, gz, t;
    if(!init_ICM42670()){
        printf("Error intializing");
        return '\0';
    } 
    printf("Successful initialization\n");
    if(!ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       printf("Error reading sensor data");
        return '\0'; 
    }
    if(ax > ax_threshold || ay > ay_threshold || az > az_threshold)
        return parseIMU(&ax, &ay, &az);
    return '\0';
    }

char parseIMU(float *ax,float *ay,float  *az){
    // acceleration in x = .
    if(*ax > *ay && *ax > *az) return '.';
    // acceleration in y = -
    else if(*ay > *az) return '-';
    // acceleration in z = space
    else return ' ';
}