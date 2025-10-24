#include <stdio.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tkjhat/sdk.h>
#include "imu.h"

#define THRESHOLD 100

char read_IMU(){
    char out;
    float ax, ay, az, gx, gy, gz, t;
    if(!init_ICM42670()){
        printf("Error intializing")
        return NULL;
    } 
    printf("Successful initialization\n");
    if(!ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       printf("Error reading sensor data")
        return NULL; 
    }
    if(ax > THRESHOLD || ay > THRESHOLD || az > THRESHOLD)
        return parseIMU(&ax, &ay, &az);
    return NULL;
    }

char parseIMU(float *ax,float *ay,float  *az){
    // acceleration in x = .
    if(*ax > *ay && *ax > *az) return '.';
    // acceleration in y = -
    else if(*ay > *az) return '-';
    // acceleration in z = space
    else return ' ';
}