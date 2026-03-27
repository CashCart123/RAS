#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/bool.h>

#define MAX_AXES 8
#define MAX_BUTTONS 12

#define LED_PIN 13
#define TOGGLE_PIN 41

// -----------------------------
// micro-ROS globals
// -----------------------------
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

sensor_msgs__msg__Joy joy_msg;
std_msgs__msg__Bool pin_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// -----------------------------
// Toggle tracking
// -----------------------------
bool pin41_state = false;
bool rb_last_state = false;

// -----------------------------
// Error handling
// -----------------------------
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) {} \
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// -----------------------------
// Publish current pin state
// -----------------------------
void publish_pin41_state() {
  pin_state_msg.data = pin41_state;
  RCSOFTCHECK(rcl_publish(&publisher, &pin_state_msg, NULL));
}

// -----------------------------
// /joy callback
// RB = buttons.data[5]
// Toggles pin 41 on rising edge
// -----------------------------
void subscription_callback(const void * msgin) {
  const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;

  if (msg->buttons.size <= 5) {
    return;
  }

  bool rb_pressed = (msg->buttons.data[5] == 1);

  // Rising edge detect: only toggle once per press
  if (rb_pressed && !rb_last_state) {
    pin41_state = !pin41_state;

    digitalWrite(TOGGLE_PIN, pin41_state ? HIGH : LOW);
    digitalWrite(LED_PIN, pin41_state ? HIGH : LOW);

    publish_pin41_state();
  }

  rb_last_state = rb_pressed;
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  pinMode(TOGGLE_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(TOGGLE_PIN, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pin41_toggle_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"
  ));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/pin41_state"
  ));

  sensor_msgs__msg__Joy__init(&joy_msg);

  joy_msg.axes.data = (float *) allocator.allocate(sizeof(float) * MAX_AXES, allocator.state);
  joy_msg.axes.capacity = MAX_AXES;
  joy_msg.axes.size = 0;

  joy_msg.buttons.data = (int32_t *) allocator.allocate(sizeof(int32_t) * MAX_BUTTONS, allocator.state);
  joy_msg.buttons.capacity = MAX_BUTTONS;
  joy_msg.buttons.size = 0;

  std_msgs__msg__Bool__init(&pin_state_msg);

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &joy_msg,
    &subscription_callback,
    ON_NEW_DATA
  ));

  // Publish initial state
  publish_pin41_state();
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}