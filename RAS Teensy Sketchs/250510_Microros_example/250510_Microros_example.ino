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
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){error_loop();}}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

  
#define MAX_AXES 8
#define MAX_BUTTONS 12


//---------------------
//END DO NOT TOUCH ZONE
//---------------------

//If you need to add global variables (Such as button state tracking do that between here and the subscription callback)

//This is the subscription call back. Run everytime a new message on the /joy topic comes through
//Within this void, put in the control logic. I.e. if a button pressed, calls a seperate function which commands the motor

//Button Definitions. If button is pressed, value will be 1. If not pressed, value will be 0:
// A = buttons.data[0]
// B = buttons.data[1]
// X = buttons.data[2]
// Y = buttons.data[3]

// LB = buttons.data[4]
// RB = buttons.data[5]
// LJ (Left Joystick) pressed = buttons.data[9]
// RJ (Right Joystick) pressed = buttons.data[10]

//Axes Definitions:
// RJ X-axis = axes.data[0] (-1 to 1, -1 is to the right, 1 is to the left)
// RJ Y-axis = axes.data[1] (-1 to 1, -1 is down, 1 is up)
// LJ X-axis = axes.data[3] (-1 to 1, -1 is to the right, 1 is to the left)
// LJ Y-axis = axes.data[4] (-1 to 1, -1 is down, 1 is up)

// LT = axes.data[1] (1 if not pressed, -1 fully depressed)
// RT = axes.data[5] (1 if not pressed, -1 fully depressed)

// D-Pad Right =  axes.data[6] will equal (-1)
// D-Pad Left =  axes.data[6] will equal (1)

// D-Pad Up =  axes.data[7] will equal (1)
// D-Pad Down =  axes.data[7] will equal (-1)



void subscription_callback(const void * msgin)
{  
  //Defining the message
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;

  // Example: read the y-button and switch the led on and off
  if (joy_msg->buttons.size > 0) {
    int32_t first_button = joy_msg->buttons.data[5];
    digitalWrite(LED_PIN, first_button ? HIGH : LOW);
  }

  // Example: read the top D-pad and switch the led on and off
  if (joy_msg->axes.size > 0) {
    float first_axes = joy_msg->axes.data[5];
    if(first_axes > 0) {
      digitalWrite(LED_PIN, HIGH);
    }
  }
}

//Between here and the DO NOT TOCUH ZONE, insert any new functions. This can be to control the stepper motors, myactuator CANBUS motors etc


//-----------------------
//BEGIN DO NOT TOUCH ZONE
//-----------------------

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
}

//---------------------
//END DO NOT TOUCH ZONE
//---------------------

void loop() {
  //Executer Spinning. This line checks for a new message on the /joy topic at 1000Hz. Reason for high speed is due to currently implemented stepper motor logic. 
  //Each time the executer is spun currently equals one step. This is a point for fixing at some point in the future 
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}


