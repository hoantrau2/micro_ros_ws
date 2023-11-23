#ifdef BOTH_PUB_SUB
#include <stdio.h>
#include <rcl/rcl.h> 
#include <rcl/error_handling.h> 
#include <rclc/rclc.h> 
#include <rclc/executor.h> 
#include <std_msgs/msg/int32.h> 
#include <std_msgs/msg/float64.h> 
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h> 
#include "pico/stdlib.h"
#include "pico_uart_transports.h" 

// Define a structure to hold the publisher and subscriber objects
typedef struct
{
    rcl_publisher_t publisher;
    rcl_subscription_t subscription;
    std_msgs__msg__Float64 msg;
} NodeComponents;
   // Initialize NodeComponents structure
    NodeComponents node_components;
void subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
    //printf("Received: %f\n", msg->data);

    // Assign the received message data to the publisher message
   // node_components.msg.data = msg->data;
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
    rclc_publisher_init_default(&node_components.publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "pico_publisher_topic");

    // Initialize Subscriber
    rclc_subscription_init_default(&node_components.subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "transfer_to_pico_topic");

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &node_components.subscription, &node_components.msg, &subscription_callback, ON_NEW_DATA);

    // Main loop
    while (true)
    {
        // Publish the received message
        
        // Spin the executor to handle subscriptions
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // Add any other logic or delay here as needed
    }

    // Clean up
    rcl_subscription_fini(&node_components.subscription, &node);
    rcl_publisher_fini(&node_components.publisher, &node);
    rcl_node_fini(&node);

    return 0;
}
#endif