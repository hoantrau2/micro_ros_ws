#ifdef PUB_OR_SUB
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
#define SUBCRIBER
// #define PUBLISHER

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
    //printf("%f \n ", values[i].data );
    // printf("ros2" );
  } 
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

#endif

#ifdef PUBLISHER_STRING
#define NUM_SENSOR 4
rcl_publisher_t publisher;
std_msgs__msg__Float64 values[NUM_SENSOR];
std_msgs__msg__String Hoan;

// Initialize timer_callback funtion
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    rcl_ret_t ret = rcl_publish(&publisher, &Hoan, NULL);
}

int main() {
    stdio_init_all();
    double voltage = 10.0005;
    double angular_pich = 20.002;
    double encoder = 30.003;
    double error = 0.00;

    // Initialize data transfer
    values[0].data = voltage;
    values[1].data = angular_pich;
    values[2].data = encoder;
    values[3].data = error;

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
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "pico_publisher_topic");

    // Initialize the string message
    Hoan.data.data = "ros2 hello";
    Hoan.data.size = strlen(Hoan.data.data);

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}


#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/float64.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()  : Node("read_data_picoW_node")
  {
    // Ensure that the necessary header is included
    subscription_ = this->create_subscription<tutorial_interfaces::msg::MyInterface>(
     "transfer_to_pico_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    if (subscription_ == nullptr) {
      // Check if subscription creation failed
      RCLCPP_ERROR(this->get_logger(), "Failed to create subscription.");
    }
  }

private:
  // Fix the type and add a default value for msg.data
  void topic_callback(const tutorial_interfaces::msg::MyInterface::SharedPtr msg) const
  {
    if (msg != nullptr) {
      // Use msg->data instead of msg.data
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->first_data.c_str());
      RCLCPP_INFO(this->get_logger(), "I heard: '%lf'", msg->data);

    } else {
      RCLCPP_ERROR(this->get_logger(), "Received null message");
    }
  }
  rclcpp::Subscription<tutorial_interfaces::msg::MyInterface>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalSubscriber>();

  // Check if the node is successfully created
  if (node != nullptr) {
    rclcpp::spin(node);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create node");
  }

  rclcpp::shutdown();
  return 0;
}
#endif


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
