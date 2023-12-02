#ifdef SUBCRIBER_CMD_VEL
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>

// Define a structure to hold the publisher and subscriber objects
typedef struct
{
    rcl_publisher_t publisher;
    rcl_subscription_t subscription;
    geometry_msgs__msg__Twist msg;
} NodeComponents;
   // Initialize NodeComponents structure
    NodeComponents node_components;

void subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    rcl_publish(&node_components.publisher, msg, NULL);
}

int main()
{
    stdio_init_all();

    // Set up Micro-ROS serial transport
    rmw_uros_set_custom_transport(
        true, NULL, pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read);

    // Initialize the Node and Micro-ROS support
    rclc_support_t support;
    rcl_allocator_t allocator;
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // Initialize Node
    rcl_node_t node;
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Initialize Publisher
    rclc_publisher_init_default(&node_components.publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "pico_publisher_topic_test");

    // Initialize Subscriber
    rclc_subscription_init_default(&node_components.subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &node_components.subscription, &node_components.msg, &subscription_callback, ON_NEW_DATA);

    // Main loop
    while (true)
    {
   
        // Spin the executor to handle subscriptions
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Clean up
    rcl_subscription_fini(&node_components.subscription, &node);
    rcl_publisher_fini(&node_components.publisher, &node);
    rcl_node_fini(&node);

    return 0;
}
#endif