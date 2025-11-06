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

    icm_i2c_write_byte(IMU_WOM_X_THR, ax_threshold);
    icm_i2c_write_byte(IMU_WOM_Y_THR, ay_threshold);
    icm_i2c_write_byte(IMU_WOM_Z_THR, az_threshold);

    icm_i2c_write_byte(IMU_SMD_CONFIG, 0b00000111);

    icm_i2c_write_byte(ICM42670_PWR_MGMT0_REG, 0x0F);
    
    icm_i2c_write_byte(IMU_ACCEL_CONFIG, 0x03);
}
void motion_handler(uint gpio, uint32_t events){
    read_IMU();
}
