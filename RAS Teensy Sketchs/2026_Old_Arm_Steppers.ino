//-----------------------
//BEGIN DO NOT TOUCH ZONE
//-----------------------

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

rcl_subscription_t subscriber;
sensor_msgs__msg__Joy msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13

#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){error_loop();}}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

#define MAX_AXES 8
#define MAX_BUTTONS 12

//---------------------
//END DO NOT TOUCH ZONE
//---------------------

// -----------------------------
// Stepper Pin Config
// -----------------------------
#define dirPin1    26
#define stepPin1   27
#define engagePin1 1

#define dirPin2    23
#define stepPin2   24
#define engagePin2 2

bool stepper1_active = false;
bool stepper2_active = false;

// -----------------------------
// Stepper Pulse Functions
// -----------------------------
void move_stepper1() {
  digitalWrite(stepPin1, HIGH);
  delayMicroseconds(200);
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(200);
}

void move_stepper2() {
  digitalWrite(stepPin2, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(500);
}

// -----------------------------
// SUBSCRIPTION CALLBACK
// -----------------------------
void subscription_callback(const void * msgin)
{  
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;

  // Safety check
  if (joy_msg->buttons.size < 4) return;

  int a = joy_msg->buttons.data[0];
  int b = joy_msg->buttons.data[1];
  int x = joy_msg->buttons.data[2];
  int y = joy_msg->buttons.data[3];

  // -----------------------------
  // Horizontal (Stepper 1)
  // A = one direction
  // Y = opposite
  // -----------------------------
  if (a) {
    digitalWrite(engagePin1, LOW);
    digitalWrite(dirPin1, LOW);
    stepper1_active = true;
  }
  else if (y) {
    digitalWrite(engagePin1, LOW);
    digitalWrite(dirPin1, HIGH);
    stepper1_active = true;
  }
  else {
    digitalWrite(engagePin1, HIGH);
    stepper1_active = false;
  }

  // -----------------------------
  // Vertical (Stepper 2)
  // B = one direction
  // X = opposite
  // -----------------------------
  if (b) {
    digitalWrite(engagePin2, LOW);
    digitalWrite(dirPin2, LOW);
    stepper2_active = true;
  }
  else if (x) {
    digitalWrite(engagePin2, LOW);
    digitalWrite(dirPin2, HIGH);
    stepper2_active = true;
  }
  else {
    digitalWrite(engagePin2, HIGH);
    stepper2_active = false;
  }

  digitalWrite(LED_PIN, a ? HIGH : LOW);
}

//-----------------------
//BEGIN DO NOT TOUCH ZONE
//-----------------------

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(engagePin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(engagePin2, OUTPUT);

  // Disable both steppers initially
  digitalWrite(engagePin1, HIGH);
  digitalWrite(engagePin2, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"));

  sensor_msgs__msg__Joy__init(&msg);

  msg.axes.data = (float *) allocator.allocate(sizeof(float) * MAX_AXES, allocator.state);
  msg.axes.capacity = MAX_AXES;
  msg.axes.size = 0;

  msg.buttons.data = (int32_t *) allocator.allocate(sizeof(int32_t) * MAX_BUTTONS, allocator.state);
  msg.buttons.capacity = MAX_BUTTONS;
  msg.buttons.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

//---------------------
//END DO NOT TOUCH ZONE
//---------------------

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  if (stepper1_active) {
    move_stepper1();
  }

  if (stepper2_active) {
    move_stepper2();
  }
}