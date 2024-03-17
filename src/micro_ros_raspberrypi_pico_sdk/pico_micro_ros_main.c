#define PUBLISHER
// #define  SUBCRIBER_CMD_VEL

#ifdef PUBLISHER
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <stdio.h>
#define NUM_SENSOR 4


rcl_publisher_t publisher;
std_msgs__msg__Float64MultiArray data_sensor;

// Initialize timer_callback funtion
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  rcl_ret_t ret = rcl_publish(&publisher, &data_sensor, NULL);
}

// Cleanup function to free allocated memory
void cleanup() {
  if (data_sensor.data.data != NULL) {
    free(data_sensor.data.data);
    data_sensor.data.data = NULL;  // Set to NULL after freeing to avoid double-free
  }
}

int main() {
  stdio_init_all();

  data_sensor.data.capacity = NUM_SENSOR;
  data_sensor.data.size = NUM_SENSOR;
  data_sensor.data.data =
      (int64_t *)malloc(data_sensor.data.capacity * sizeof(double));

  double voltage = 10.0005;
  double angular_pich = 20.002;
  double encoder = 30.003;
  double error = 0.00;

  data_sensor.data.data[0] = voltage;
  data_sensor.data.data[1] = angular_pich;
  data_sensor.data.data[2] = encoder;
  data_sensor.data.data[3] = error;
 
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

  // Initialize Timer_Interrupt
  rcl_timer_t timer;
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;
  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);

  // Initialize Publisher
  rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "transfer_to_pico_topic");

  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
   // Cleanup before exiting
  cleanup();
  
  return 0;
}
#endif

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