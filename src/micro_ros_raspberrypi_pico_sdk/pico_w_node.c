#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <stdio.h>
#define ANGULAR_VELOCITY 4

// Define a structure to hold the publisher and subscriber objects
typedef struct {
  rcl_publisher_t publisher;
  rcl_subscription_t subscription;
} NodeComponents;

NodeComponents node_components;

std_msgs__msg__Float64MultiArray angular_velocity_motor;

double motor1 = 10.0005;
double motor2 = 20.002;
double motor3 = 30.003;
double motor4 = 0.020;

// Initialize timer_callback funtion
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  angular_velocity_motor.data.capacity = ANGULAR_VELOCITY;
  angular_velocity_motor.data.size = ANGULAR_VELOCITY;
  angular_velocity_motor.layout.data_offset = 222;
  angular_velocity_motor.data.data =
      (double *)malloc(angular_velocity_motor.data.capacity * sizeof(double));
  angular_velocity_motor.data.data[0] = motor1;
  angular_velocity_motor.data.data[1] = motor2;
  angular_velocity_motor.data.data[2] = motor3;
  angular_velocity_motor.data.data[3] = motor4;
  rcl_ret_t ret =
      rcl_publish(&node_components.publisher, &angular_velocity_motor, NULL);
}

// Initialize subscription_callback funtion
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *msg =
      (const std_msgs__msg__Float64MultiArray *)msgin;
  if (msg->layout.data_offset == 333) {
    // angular_velocity_motor.data.data[0] = msg->data.data[0];
    // angular_velocity_motor.data.data[1] = msg->data.data[1];
    // angular_velocity_motor.data.data[2] = msg->data.data[2];
    // angular_velocity_motor.data.data[3] = msg->data.data[3];
    // Process data here
    printf("Received desired angle data\n");
  }
}

// Cleanup function to free allocated memory
// Set to NULL after freeing to avoid double-free
void cleanup() {
  if (angular_velocity_motor.data.data != NULL) {
    free(angular_velocity_motor.data.data);
    angular_velocity_motor.data.data = NULL;
  }
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
  rclc_node_init_default(&node, "pico_w_node", "", &support);

  // Initialize Timer_Interrupt
  // timeout for the ping function to the agent (Micro-ROS)
  // 120 is a number of reconnection attempts when ping agent failed
  rcl_timer_t timer;
  const int timeout_ms = 1000;
  const uint8_t attempts = 120;
  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
  if (ret != RCL_RET_OK) {
    return ret;
  }

  // RCL_MS_TO_NS(100) ms is the interupt timer
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);

  // Initialize Publisher
  rclc_publisher_init_default(
      &node_components.publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/actual_angle");
  // Initialize Subscriber
  rclc_subscription_init_default(
      &node_components.subscription, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/desired_angle");

  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &node_components.subscription,
                                 &angular_velocity_motor,
                                 &subscription_callback, ON_NEW_DATA);

  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    // 100 is the maximum time a function can run before returning
  }
  // Clean up
  rcl_subscription_fini(&node_components.subscription, &node);
  rcl_publisher_fini(&node_components.publisher, &node);
  rcl_node_fini(&node);
  // Cleanup before exiting
  cleanup();
  return 0;
}

// cd micro_ros_raspberrypi_pico_sdk
// mkdir build
// cd build
// cmake ..
// make
// cp pico_micro_ros_example.uf2 /media/$USER/RPI-RP2
// sudo snap set core experimental.hotplug=true
// sudo systemctl restart snapd
// snap interface serial-port
// snap connect micro-ros-agent:serial-port snapd:pico
// sudo micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200