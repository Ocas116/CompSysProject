#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <pico/cyw43_arch.h>
#include <pico/flash.h>

#include <tkjhat/sdk.h>
#include "imu.h"


#define CALIB_VALS_OFFSET (1560 * 1024)

static float ax_low_threshold = -1.5, ax_high_threshold = 1.5, az_threshold = -0.5;


typedef struct {
    float a, b, c;
} calib_vals;

void __not_in_flash_func(write_calibration_to_flash)(calib_vals *data);

void load_calib_IMU(){
    const calib_vals *data = (const calib_vals *)(XIP_BASE + CALIB_VALS_OFFSET);
    ax_low_threshold = data->a;
    ax_high_threshold = data->b;
    az_threshold = data->c;
}
void test_write(){
    char text[256] = "Hello, Flash!";
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CALIB_VALS_OFFSET, 4096);
    flash_range_program(
        CALIB_VALS_OFFSET,
        (const uint8_t *)&text,
        sizeof(text)
    );
    restore_interrupts(ints);
}

int set_calib_IMU(){
    int flag = 0;
    float ax, ay, az, gx, gy, gz, t;
    delay_ms(1000);
    while(!flag){
        if(0 > ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       //printf("Error reading sensor data");
       return 1;
    }
    cyw43_delay_ms(100);
    printf("calibrating... \n max vals min %d,max %d,az %d\n", (int)(ax_low_threshold*1000), (int)(ax_high_threshold*1000), (int)(az_threshold*1000));
    if(ax_low_threshold > ax) ax_low_threshold = ax;
    if(ax_high_threshold < ax) ax_high_threshold = ax;
    if(az_threshold > az) az_threshold = az;
    if(gpio_get(SW2_PIN))
    flag = 1;
    }
}

char read_IMU(){
    float ax, ay, az, gx, gy, gz, t;
    //printf("Successful initialization\n");
    if(0 > ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       printf("Error reading sensor data");
        return 'X'; 
    }
    if(ax < ax_low_threshold || ax > ax_high_threshold || az < az_threshold)
        return parseIMU(ax, az);
    return 'X';
    }

char parseIMU(float ax,float  az){
    if(az < -abs(ax)) return '_';
    if(ax < 0) return '.';
    return '-';
    
}
void IMU_init(){
    if(!init_ICM42670()){
        printf("Error intializing");
        return;
    } 
    load_calib_IMU();
    IMU_LP_init();
    gpio_set_dir(ICM42670_INT, GPIO_IN);
    gpio_pull_down(ICM42670_INT);
    gpio_set_irq_enabled_with_callback(
    ICM42670_INT,
    GPIO_IRQ_EDGE_RISE,
    true,
    &motion_handler);
}
void IMU_LP_init(){

    /**
     * Find control registers from this pdf 
     * https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf
     */
    icm_i2c_write_byte(ICM42670_PWR_MGMT0_REG, 0x00);

    icm_i2c_write_byte(IMU_WOM_CONFIG, 0b00000001);

    icm_i2c_write_byte(IMU_WOM_X_THR, ax_low_threshold);
    icm_i2c_write_byte(IMU_WOM_Y_THR, 0xFF);
    icm_i2c_write_byte(IMU_WOM_Z_THR, az_threshold);

    icm_i2c_write_byte(IMU_SMD_CONFIG, 0b00000111);

    icm_i2c_write_byte(ICM42670_PWR_MGMT0_REG, 0x0F);
    
    icm_i2c_write_byte(IMU_ACCEL_CONFIG, 0x03);
}
void motion_handler(uint gpio, uint32_t events){
    read_IMU();
}

void delay_ms(uint32_t ms){
    cyw43_delay_ms(ms);
}