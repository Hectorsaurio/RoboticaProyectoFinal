#include <stdio.h>
#include <math.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// Pines
#define MOTOR_A_ENA 6
#define MOTOR_A_IN1 7
#define MOTOR_A_IN2 8
#define MOTOR_B_ENB 11
#define MOTOR_B_IN3 9
#define MOTOR_B_IN4 10

#define ENCODER_LEFT_ADC 0   // GPIO26
#define ENCODER_RIGHT_ADC 1  // GPIO27

// Parámetros físicos
const float MAX_SPEED = 0.3f;

rcl_subscription_t subscriber;
rcl_publisher_t encoder_pub;
geometry_msgs__msg__Twist msg;

float read_angle_from_adc(uint adc_input) {
    adc_select_input(adc_input);
    uint16_t raw = adc_read();
    float voltage = (raw / 4095.0f) * 3.3f;
    return (voltage / 3.3f) * 2.0f * M_PI;  // Escalado de 0 a 2π
}

void pwm_set_speed(int pin, float speed) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);

    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;

    int pwm_value = (int)(fabs(speed) / MAX_SPEED * 255.0f);
    pwm_set_chan_level(slice, channel, pwm_value);
}

void set_motor(int ena, int in1, int in2, float vel) {
    gpio_put(in1, vel > 0);
    gpio_put(in2, vel < 0);
    pwm_set_speed(ena, vel);
}

void subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float v = msg->linear.x;
    float w = msg->angular.z;
    const float L = 0.20f;

    float v_r = v + (L / 2.0f) * w;
    float v_l = v - (L / 2.0f) * w;

    set_motor(MOTOR_A_ENA, MOTOR_A_IN1, MOTOR_A_IN2, v_l);
    set_motor(MOTOR_B_ENB, MOTOR_B_IN3, MOTOR_B_IN4, v_r);
}

void pwm_init_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
    pwm_set_enabled(slice, true);
}

void publish_encoders(rcl_publisher_t * publisher) {
    float angle_left = read_angle_from_adc(ENCODER_LEFT_ADC);
    float angle_right = read_angle_from_adc(ENCODER_RIGHT_ADC);

    geometry_msgs__msg__Vector3 enc_msg;
    geometry_msgs__msg__Vector3__init(&enc_msg);
    enc_msg.x = angle_left;
    enc_msg.y = angle_right;
    enc_msg.z = 0.0;

    rcl_publish(publisher, &enc_msg, NULL);
}

int main() {
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    stdio_init_all();
    adc_init();

    gpio_init(MOTOR_A_IN1); gpio_set_dir(MOTOR_A_IN1, GPIO_OUT);
    gpio_init(MOTOR_A_IN2); gpio_set_dir(MOTOR_A_IN2, GPIO_OUT);
    gpio_init(MOTOR_B_IN3); gpio_set_dir(MOTOR_B_IN3, GPIO_OUT);
    gpio_init(MOTOR_B_IN4); gpio_set_dir(MOTOR_B_IN4, GPIO_OUT);

    pwm_init_pin(MOTOR_A_ENA);
    pwm_init_pin(MOTOR_B_ENB);

    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) return 1;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // cmd_vel
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    // encoder publisher
    rclc_publisher_init_default(
        &encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "encoders"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
        publish_encoders(&encoder_pub);
        sleep_ms(100);
    }

    return 0;
}