#include <stdio.h>
#include <rcl/rcl.h> //ROS Client Library (RCL) providing functions and data structures for ROS 2 application development.
#include <rcl/error_handling.h> //RCL error handling library.
#include <rclc/rclc.h> //ROS Client Library for C (RCLC) to simplify using RCL in C applications.
#include <rclc/executor.h> //xecutor library for RCLC to manage multiple tasks in a program.
#include <std_msgs/msg/int32.h> //Library containing the definition of the std_msgs::msg::Int32 message type.
#include <std_msgs/msg/float64.h> 
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h> //Micro-ROS library containing functions and definitions related to Micro-ROS.
#include "pico/stdlib.h"
#include "pico_uart_transports.h" //Library containing functions related to UART communication.
#define SUBCRIBER

#ifdef SUBCRIBER

rcl_subscription_t subscription;
std_msgs__msg__Float64 msg;

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
  printf("%f\n", msg->data);
}

int main()
{
  stdio_init_all();

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_subscriber_node", "", &support);

  rclc_subscription_init_default(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "transfer_to_pico_topic");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscription, &msg, &subscription_callback, ON_NEW_DATA);
  while (true)
  {
//    rclc_spin_node_once(&node, 1000000);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    // sleep_us(100000);
  }

  return 0;
}

#endif

#ifdef PUBLISHER
#define NUM_SENSOR 4
const uint LED_PIN = 25;
rcl_publisher_t publisher;
std_msgs__msg__Float64 values[NUM_SENSOR];

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  for (size_t i = 0; i < NUM_SENSOR; ++i) 
  {
    rcl_ret_t ret = rcl_publish(&publisher, &values[i], NULL);
  }
//    printf("%f \n ", values[1].data );
}

int main() 
{
    stdio_init_all();
  double voltage = 10.0005;
  double angular_pich = 20.002;
  double encoder = 30.003;
  double error = 0.00;

  double sensor_values[] = {voltage, angular_pich, encoder, error};
  for (size_t i = 0; i < NUM_SENSOR; ++i)
  {
      values[i].data = sensor_values[i];
  }


  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

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

  if (ret != RCL_RET_OK) {
    return ret;
  }
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_node", "", &support);
  rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "pico_publisher_topic");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  gpio_put(LED_PIN, 1);

 
  while (true) 
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
  return 0;
}
#endif

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
// ros2 topic list
// ros2 topic echo /pico_publisher
// ros2 node list

// int main() {

//     const uint LED_PIN = 25;
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);
//     while (true) {
//         printf("hoan\n");
//         gpio_put(LED_PIN, 1);
//         sleep_ms(250);
//         gpio_put(LED_PIN, 0);
//         sleep_ms(250);
//     }
// }