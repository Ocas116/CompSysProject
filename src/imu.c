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
#include "led.h"
#include <tkjhat/sdk.h>
#include "imu.h"


#define CALIB_VALS_OFFSET (1560 * 1024)

static float ax_threshold = 0.5, ay_threshold = 0.5, az_threshold = 1.5;


typedef struct {
    float a, b, c;
} calib_vals;


void load_calib_IMU(){
    const calib_vals *data = (const calib_vals *)(XIP_BASE + CALIB_VALS_OFFSET);
    ay_threshold = data->a;
    ax_threshold = data->b;
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
    ax_threshold = 0.5, ay_threshold = 0.5, az_threshold = 1.5;
    delay_ms(1000);
    while(!flag){
        if(0 > ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       printf("Error reading sensor data");
       return 1;
    }
    led_blink_dot();
    if(ay_threshold < ax) ay_threshold = ay;
    if(ax_threshold < ax) ax_threshold = ax;
    if(az_threshold < az) az_threshold = az;
    delay_ms(500);
    if(gpio_get(SW2_PIN)) flag = 1;
    }
    delay_ms(1000);
    return 0;
}

char read_IMU(){
    float ax, ay, az, gx, gy, gz, t;
    //printf("Successful initialization\n");
    if(0 > ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t)){
       printf("Error reading sensor data");
        return 'X'; 
    }
    if(ay > ay_threshold) return '.';
    if(ax > ax_threshold) return '-';
    if(az > az_threshold) return ' ';
    return 'X';
    }

void IMU_init(){
    printf("Initializing IMU\n");
    // load_calib_IMU();
    gpio_set_dir(ICM42670_INT, GPIO_IN);
    int ret = init_ICM42670();
    if(0 > ret){
        printf("Error intializing %d", ret);
    } 
    ICM42670_start_with_default_values();
    /*
    IMU_LP_init();
    
    printf("Setting up interrupt\n");
    gpio_pull_up(ICM42670_INT);
    gpio_set_irq_enabled_with_callback(
    ICM42670_INT,
    GPIO_IRQ_EDGE_RISE,
    true,
    motion_handler);

    */
}
void IMU_LP_init(){
    printf("Configuring\n");
    /**
     * Find control registers from this pdf 
     * https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf
     */
    int ret = 0;
    uint8_t value;
    // icm_i2c_write_byte(0x26, 0x03);
    if(ret < 0) printf("Error writing i2c\n");
    delay_ms(1000);
    ret = icm_i2c_write_byte(IMU_INT_CONFIG , 0x02); 
    if(ret < 0) printf("Error writing i2c int\n");
    delay_ms(500);
    ret = icm_i2c_write_byte(IMU_WOM_CONFIG, 0x03);
    if(ret < 0) printf("Error writing i2c config\n");
    ret = icm_i2c_write_byte(IMU_INT_SOURCE, 0x07);
    if(ret < 0) printf("Error writing i2c source\n");
    delay_ms(50);
    ret = icm_i2c_write_byte(IMU_WOM_X_THR, 0x50);
    if(ret < 0) printf("Error writing i2c x\n");
    busy_wait_ms(50);
    icm_i2c_read_byte(IMU_WOM_Y_THR, &value);
    ret = icm_i2c_write_byte(IMU_WOM_Y_THR, 0x50);
    if(ret < 0) printf("Error writing i2c y\n");
    busy_wait_ms(200);
    
    ret = icm_i2c_write_byte(IMU_WOM_Z_THR, 0x50); 
    if(ret < 0) printf("Error writing i2c z\n");   
    delay_ms(50);
    ret = icm_i2c_write_byte(0x24, 0x63); 
    if(ret < 0) printf("Error writing i2c z\n");   
    delay_ms(50);
    
    ICM42670_start_with_default_values();
    ICM42670_enable_ultra_low_power_mode();
    uint8_t status = 0;
    ret = icm_i2c_read_byte(IMU_INT_STATUS, &status);
    if(ret < 0) printf("Error reading i2c\n");
    printf("INT STATUS: 0x%02X\n", status);
    printf("Success?\n");
}
int count = 0;
void motion_handler(uint gpio, uint32_t event_Mask){
    uint8_t status = 0;
    int ret = icm_i2c_read_byte(IMU_INT_STATUS, &status);
    if(ret < 0) printf("Error reading i2c\n");
    printf("INT STATUS: 0x%02X\n", status);
    delay_ms(500);
    printf("Interrupt detected from IMU %d\n", count++);
    
    //printf("\ninterrupt input: %c\n", read_IMU());
}

void delay_ms(uint32_t ms){
    busy_wait_ms(ms);
}