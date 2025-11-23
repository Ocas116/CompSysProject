#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include "imu.h"
#include <tkjhat/sdk.h>
#include "led.h"

#define TIME_UNIT 200
void led_blink_char(char input){
    if(input == ' ') led_blink_space();
    if(input == '.') led_blink_dot();
    if(input == '-') led_blink_dash(); 
}
void led_blink_dot(){
    gpio_put(RED_LED_PIN, 1);
    delay_ms(TIME_UNIT);
    gpio_put(RED_LED_PIN, 0);
    delay_ms(TIME_UNIT);
}
void led_blink_dash(){
    gpio_put(RED_LED_PIN, 1);
    delay_ms(TIME_UNIT * 3);
    gpio_put(RED_LED_PIN, 0);
    delay_ms(TIME_UNIT);
}
void led_blink_space(){
    delay_ms(TIME_UNIT * 3);
    gpio_put(RED_LED_PIN, 1);
    delay_ms(30);
    gpio_put(RED_LED_PIN, 0);
    delay_ms(30);
    gpio_put(RED_LED_PIN, 1);
    delay_ms(30);
    gpio_put(RED_LED_PIN, 0);
    delay_ms(TIME_UNIT);
}
void led_blink_eom(){
    gpio_put(RED_LED_PIN, 1);
    delay_ms(120);
    gpio_put(RED_LED_PIN, 0);
    delay_ms(120);
    gpio_put(RED_LED_PIN, 1);
    delay_ms(120);
    gpio_put(RED_LED_PIN, 0);
    delay_ms(TIME_UNIT);
}
void led_blink_success(){
    for(int i=0;i<50;i++){
    gpio_put(RED_LED_PIN, 1);
    delay_ms(20);
    gpio_put(RED_LED_PIN, 0);
    delay_ms(20);
    }
    delay_ms(TIME_UNIT);
    
}
