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
#define PUBLISHER
// #define SUBCRIBER_SUB
#ifdef SUBCRIBER_SUB
// Define a structure to hold the publisher and subscriber objects
typedef struct {
  rcl_publisher_t publisher;
  rcl_subscription_t subscription;
  std_msgs__msg__Float64 msg;
} NodeComponents;
// Initialize NodeComponents structure
NodeComponents node_components;
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
  rcl_publish(&node_components.publisher, msg, NULL);
}

int main() {
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
  rclc_publisher_init_default(
      &node_components.publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "pico_publisher_topic");

  // Initialize Subscriber
  rclc_subscription_init_default(
      &node_components.subscription, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "transfer_to_pico_topic");

  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &node_components.subscription,
                                 &node_components.msg, &subscription_callback,
                                 ON_NEW_DATA);

  // Main loop
  while (true) {
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

#ifdef PUBLISHER
#define NUM_SENSOR 4
rcl_publisher_t publisher;
std_msgs__msg__Float64MultiArray data_sensor;

// Initialize timer_callback funtion
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  double voltage = 10.0005;
  double angular_pich = 20.002;
  double encoder = 30.003;
  double error = 0.00;

  data_sensor.data.data[0] = voltage;
  data_sensor.data.data[1] = angular_pich;
  data_sensor.data.data[2] = encoder;
  data_sensor.data.data[3] = error;

  rcl_ret_t ret = rcl_publish(&publisher, &data_sensor, NULL);
}

int main() {
  stdio_init_all();

  // double voltage = 10.0005;
  // double angular_pich = 20.002;
  // double encoder = 30.003;
  // double error = 0.00;

  // // Initialize data transfer
  // //   data_sensor.size = NUM_SENSOR;
  // //   data_sensor.capacity = NUM_SENSOR;
  // //   data_sensor.data[0].data = voltage;
  // //   data_sensor.data[1].data = angular_pich;
  // //   data_sensor.data[2].data = encoder;
  // //   data_sensor.data[3].data = error;

  data_sensor.data.capacity = NUM_SENSOR;
  data_sensor.data.size = NUM_SENSOR;
  data_sensor.data.data =
      (int64_t *)malloc(data_sensor.data.capacity * sizeof(double));

  // data_sensor.layout.dim.capacity = 1;
  // data_sensor.layout.dim.size = 1;
  // data_sensor.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(
  //     data_sensor.layout.dim.capacity *
  //     sizeof(std_msgs__msg__MultiArrayDimension));

  // for (size_t i = 0; i < data_sensor.layout.dim.capacity; i++) {
  //   data_sensor.layout.dim.data[i].label.capacity = 20;
  //   data_sensor.layout.dim.data[i].label.size = 0;
  //   data_sensor.layout.dim.data[i].label.data = (char *)malloc(
  //       data_sensor.layout.dim.data[i].label.capacity * sizeof(char));
  // }
  // data_sensor.data.data[0] = voltage;
  // data_sensor.data.data[1] = angular_pich;
  // data_sensor.data.data[2] = encoder;
  // data_sensor.data.data[3] = error;

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
  return 0;
}
#endif
