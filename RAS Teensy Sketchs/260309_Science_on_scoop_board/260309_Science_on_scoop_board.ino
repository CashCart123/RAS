#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/joy.h>

#define MAX_AXES 8
#define MAX_BUTTONS 12
#define LED_PIN 13

#define base1 1
#define base2 2   // keep definition, even though no special new purpose
#define top1 3
#define top2 4

// -----------------------------
// micro-ROS globals
// -----------------------------
rcl_subscription_t subscriber;
sensor_msgs__msg__Joy msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// -----------------------------
// top actuator toggle state
// -----------------------------
bool top_actuator_on = false;
bool prev_rb_pressed = false;
unsigned long last_rb_toggle_time = 0;
const unsigned long RB_DEBOUNCE_MS = 200;

// -----------------------------
// Error Handling Macros
// -----------------------------
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) {} \
}

// Function prototypes
void error_loop();
void subscription_callback(const void * msgin);
void move_actuators_base(int state);
void set_top_actuator(bool on);

// -----------------------------
// Error Loop
// -----------------------------
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// -----------------------------
// Callback function for /joy
// -----------------------------
void subscription_callback(const void * msgin)
{
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;

  // Safety check
  if (joy_msg->buttons.size < 6) {
    return;
  }

  // Xbox-style mapping usually:
  // A = 0, B = 1, X = 2, Y = 3, LB = 4, RB = 5
  int32_t b_pressed  = joy_msg->buttons.data[1];
  int32_t x_pressed  = joy_msg->buttons.data[2];
  int32_t rb_pressed = joy_msg->buttons.data[5];

  // -----------------------------
  // Base actuator control
  // X = up, B = down
  // -----------------------------
  if (x_pressed == 1) {
    move_actuators_base(0);   // up
  }
  else if (b_pressed == 1) {
    move_actuators_base(1);   // down
  }
  else {
    move_actuators_base(2);   // stop
  }

  // -----------------------------
  // Top actuator toggle on RB
  // -----------------------------
  unsigned long now = millis();

  if (rb_pressed && !prev_rb_pressed && (now - last_rb_toggle_time > RB_DEBOUNCE_MS)) {
    top_actuator_on = !top_actuator_on;
    set_top_actuator(top_actuator_on);
    last_rb_toggle_time = now;
  }

  prev_rb_pressed = rb_pressed;

  // LED shows top actuator state
  digitalWrite(LED_PIN, top_actuator_on ? HIGH : LOW);
}

// -----------------------------
// Base actuator control
// 0 = up
// 1 = down
// 2 = stop
// -----------------------------
void move_actuators_base(int state)
{
  if (state == 0) {
    // Move base actuator up
    digitalWrite(base1, HIGH);
    digitalWrite(base2, LOW);
  }
  else if (state == 1) {
    // Move base actuator down
    digitalWrite(base1, LOW);
    digitalWrite(base2, HIGH);
  }
  else {
    // Stop base actuator
    digitalWrite(base1, HIGH);
    digitalWrite(base2, HIGH);
  }
}

// -----------------------------
// Top actuator control
// on  = drive actuator
// off = stop actuator
// -----------------------------
void set_top_actuator(bool on)
{
  if (on) {
    // Drive top actuator in one direction
    digitalWrite(top1, HIGH);
    digitalWrite(top2, LOW);
  } else {
    // Stop top actuator
    digitalWrite(top1, HIGH);
    digitalWrite(top2, HIGH);
  }
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"
  ));

  sensor_msgs__msg__Joy__init(&msg);

  msg.axes.data = (float *) allocator.allocate(sizeof(float) * MAX_AXES, allocator.state);
  msg.axes.capacity = MAX_AXES;
  msg.axes.size = 0;

  msg.buttons.data = (int32_t *) allocator.allocate(sizeof(int32_t) * MAX_BUTTONS, allocator.state);
  msg.buttons.capacity = MAX_BUTTONS;
  msg.buttons.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Output pins
  pinMode(base1, OUTPUT);
  pinMode(base2, OUTPUT);
  pinMode(top1, OUTPUT);
  pinMode(top2, OUTPUT);

  // Default inactive state
  digitalWrite(base1, HIGH);
  digitalWrite(base2, HIGH);
  digitalWrite(top1, HIGH);
  digitalWrite(top2, HIGH);

  top_actuator_on = false;
  prev_rb_pressed = false;
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}