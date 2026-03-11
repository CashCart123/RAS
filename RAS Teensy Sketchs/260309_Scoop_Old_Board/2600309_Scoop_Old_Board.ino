
#include <micro_ros_arduino.h>



#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include Int32 message instead of Joy
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#define MAX_AXES 8
#define MAX_BUTTONS 12

// Global Variables
rcl_subscription_t subscriber;

#include <sensor_msgs/msg/joy.h>
sensor_msgs__msg__Joy msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// Timer is not used; remove if unnecessary
// rcl_timer_t timer;

#define LED_PIN 13
// #define base1 1
// #define base2 2
// #define top1 3
// #define top2 4

#define base1 1
#define base2 2
#define top1 3
#define top2 4


// Error Handling Macros
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){error_loop();}}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){}}

// Error Loop: Blink LED rapidly to indicate an error
void error_loop(){
  // Turn off all actuators for safety
  // digitalWrite(base1, LOW);
  // digitalWrite(base2, LOW);
  // digitalWrite(top1, LOW);
  // digitalWrite(top2, LOW);


  // Blink LED rapidly
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Callback function for /button_states
void subscription_callback(const void * msgin)
{  
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;
  // Extract the button bitmask


  // Decode button states from bitmask
  int32_t a_pressed = joy_msg->buttons.data[0];
  int32_t b_pressed = joy_msg->buttons.data[1];
  int32_t x_pressed = joy_msg->buttons.data[2]; 
  int32_t y_pressed = joy_msg->buttons.data[3];


  // Control base actuators based on A and B buttons
  if(a_pressed == 1)
  {
    move_actuators_base(0);
  }
  else if(b_pressed == 1)
  {
    move_actuators_base(1);
  }
  else
  {
    move_actuators_base(2);
  }

  // Control top actuator based on X and Y buttons
  if(x_pressed == 1)
  {
    move_actuator_top(0);
  }
  else if(y_pressed == 1)
  {
    move_actuator_top(1);
  }
  else 
  {
    move_actuator_top(2);
  }

  // Update LED state based on y button
  digitalWrite(LED_PIN, (y_pressed) ? HIGH : LOW); 
}

// Function to control base actuators
void move_actuators_base(int y)
{
  if (y == 0)
  {
    // Move base forward
    digitalWrite(base1, HIGH);
    digitalWrite(base2, LOW);


  }
  else if (y == 1) 
  {
    // Move base backward
    digitalWrite(base1, LOW);
    digitalWrite(base2, HIGH);
  }
  else
  {
    // Stop base actuators
    digitalWrite(base2, HIGH);
    digitalWrite(base1, HIGH);

  }
}

// Function to control top actuator
void move_actuator_top(int x)
{
  if (x == 0)
  {
    // Move top actuator up
    digitalWrite(top1, HIGH);
    digitalWrite(top2, LOW);
  }
  else if (x == 1) 
  {
    // Move top actuator down
    digitalWrite(top1, LOW);
    digitalWrite(top2, HIGH);
  }
  else
  {
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

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
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

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Base actuator 
  pinMode(base1, OUTPUT);
  pinMode(base2, OUTPUT);
  // Top actuator
  pinMode(top1, OUTPUT);
  pinMode(top2, OUTPUT);

  // Set all actuators to default HIGH (inactive state)
  digitalWrite(top1, HIGH);
  digitalWrite(top2, HIGH);

  digitalWrite(base1, HIGH);
  digitalWrite(base2, HIGH);

}

void loop() {

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}