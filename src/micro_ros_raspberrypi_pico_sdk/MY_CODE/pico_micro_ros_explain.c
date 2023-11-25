#ifdef EXPLAIN
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

const uint LED_PIN = 25;
rcl_publisher_t publisher; // used for publishing messages
std_msgs__msg__Float64 imu; //used to store the data of the message.

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) //last_call_time: the last time this callback was invoked.
{
    rcl_ret_t ret = rcl_publish(&publisher, &imu, NULL); //The rcl_publish function is used to publish a message via the publisher.
    //&publisher: The address of the publisher variable, representing the object publishing information to the ROS 2 network.
    // &msg: The address of the msg variable, which contains the data of the message to be published.
    // NULL: For simple cases, information about a service call can be set to NULL.
    // The return value of the rcl_publish function is stored in the ret variable of type rcl_ret_t, commonly used to check whether the publishing was successful.
    imu.data =10.0005;
    //After the message is published, the data value of the message (msg.data) is incremented each time the callback function is invoked. 
    //This is done to have the data value of the message increase by one with each publication, creating an increasing sequence of integers.
    // This can be useful for tracking time or events related to the message.

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
//     Purpose:
// The main purpose of this code snippet is to set up custom communication through the UART serial port for Micro-ROS. 
// This is crucial for connecting the embedded device to a computer or a ROS management system via the serial port.
// UART Communication:
// By using custom open, close, write, and read functions, the program can perform communication via the UART serial port with other devices, 
// such as a host computer running ROS Master or other embedded devices using Micro-ROS.

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    //The purpose of this code is trying to connect to the Micro-ROS Agent by sending a PING request and waiting for a response.
    //If there is no response after the waiting period and the maximum number of connection attempts has been reached, 
    //the program will continue with the next steps or exit with a coding error, optionally program logic.

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
//     rclc_support_init: Initializes a rclc_support_t structure to support the use of RCL.
// &support: Pointer to the initialized rclc_support_t structure.
// 0: The maximum number of nodes you want to support. In this case, there is no limit.
// NULL: No specific node is needed during initialization.
// &allocator: Pointer to an allocator, which can be NULL to use the default allocator.

    rclc_node_init_default(&node, "pico_node", "", &support);
//     rclc_node_init_default: Initializes a node with default parameters.
// &node: Pointer to the initialized rcl_node_t structure.
// "pico_node": The name of the node.
// "": The namespace of the node (in this case, no namespace is used).
// &support: Pointer to the initialized rclc_support_t structure.
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "pico_publisher_topic");
// rclc_publisher_init_default: Initializes a publisher with default parameters.
// &publisher: Pointer to the initialized rcl_publisher_t structure.
// &node: Pointer to the rcl_node_t structure that the publisher will belong to.
// ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32):   
//In this case, the message type is std_msgs::msg::Int32.
// "pico_publisher": The name of the publisher.
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

// rclc_timer_init_default: Initializes a timer with default parameters.
// &timer: Pointer to the initialized rcl_timer_t structure.
// &support: Pointer to the initialized rclc_support_t structure.
// RCL_MS_TO_NS(1000): The cycle time of the timer, converted from milliseconds to nanoseconds.
//  In this case, the timer will call the timer_callback function every 1000 milliseconds (1 second).
// timer_callback: The callback function to be called when the timer expires.

    rclc_executor_init(&executor, &support.context, 1, &allocator);
//     rclc_executor_init: Initializes an executor to manage tasks within a context.
// &executor: Pointer to the initialized rclc_executor_t structure.
// &support.context: Pointer to the context of the executor, obtained from the rclc_support_t structure.
// 1: The number of timers the executor can manage.
// &allocator: Pointer to an allocator, which can be NULL to use the default allocator.
    rclc_executor_add_timer(&executor, &timer);
// rclc_executor_add_timer: Adds a timer to the executor for management.
// &executor: Pointer to the rclc_executor_t structure.
// &timer: Pointer to the rcl_timer_t structure of the timer to be added to the executor.
    gpio_put(LED_PIN, 1);

    imu.data = 0;


    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//The time duration parameter (RCL_MS_TO_NS(100)) determines how much time the executor is allowed to spend on its tasks during each invocation. 
//if must be less than the timer interrupt 
// The executor will attempt to perform as much work as possible within the given time duration.
// This function is often used in a loop to repeatedly advance the execution of tasks managed by the executor.
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
#endif




#ifdef CODE_ANH_TRI_SP
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

#ifdef PUBLISHER
#define NUM_SENSOR 4
rcl_publisher_t publisher;
std_msgs__msg__Float64MultiArray data_sensor;

// Initialize timer_callback funtion
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
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

  double voltage = 10.0005;
  double angular_pich = 20.002;
  double encoder = 30.003;
  double error = 0.00;

  data_sensor.data.data[0] = voltage;
  data_sensor.data.data[1] = angular_pich;
  data_sensor.data.data[2] = encoder;
  data_sensor.data.data[3] = error;
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


// ben linux
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "transfer_to_pico_topic", 10,
        std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void
  topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {
     RCLCPP_INFO(this->get_logger(), "Data1: %lf",msg->data[0]);
     RCLCPP_INFO(this->get_logger(), "Data2: %lf",msg->data[1]);
     RCLCPP_INFO(this->get_logger(), "Data3: %lf",msg->data[2]);
     RCLCPP_INFO(this->get_logger(), "Data4: %lf",msg->data[3]);
     
    // std::cout << "Data 1: " << msg->data[0] << std::endl;
    // std::cout << "Data 2: " << msg->data[1] << std::endl;
    // std::cout << "Data 3: " << msg->data[2] << std::endl;
    // std::cout << "Data 4: " << msg->data[3] << std::endl;
  }
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

#endif