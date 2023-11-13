#include <stdio.h>
#include <rcl/rcl.h> //ROS Client Library (RCL) providing functions and data structures for ROS 2 application development.
#include <rcl/error_handling.h> //RCL error handling library.
#include <rclc/rclc.h> //ROS Client Library for C (RCLC) to simplify using RCL in C applications.
#include <rclc/executor.h> //xecutor library for RCLC to manage multiple tasks in a program.
#include <std_msgs/msg/int32.h> //Library containing the definition of the std_msgs::msg::Int32 message type.
#include <rmw_microros/rmw_microros.h> //Micro-ROS library containing functions and definitions related to Micro-ROS.
#include "pico/stdlib.h"
#include "pico_uart_transports.h" //Library containing functions related to UART communication.

const uint LED_PIN = 25;

rcl_publisher_t publisher; 
std_msgs__msg__Int32 msg; 
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL); 
    msg.data++;
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	 );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    
    if (ret != RCL_RET_OK)
    {
        return ret;
    }
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);

    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    msg.data = 0;


    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    }
    return 0;
}

// ---command lines---
// cd micro_ros_raspberrypi_pico_sdk
// mkdir build
// cd build
// cmake ..
// make
// cp pico_micro_ros_example.uf2 /media/$USER/RPI-RP2
// export ROS_DOMAIN_ID=0
// snap interface serial-port
// snap connect micro-ros-agent:serial-port snapd:pico
// sudo micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200