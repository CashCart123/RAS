//-----------------------
//BEGIN DO NOT TOUCH ZONE
//-----------------------

#include <micro_ros_arduino.h>

#include <mcp_can.h>
#include <SPI.h>

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

MCP_CAN CAN0(10);     // Set CS to pin 10 (MISO, MOSI, and SCK automatically assigned)

byte data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Initialization command
byte data1[8] = {0xA1, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00}; // Motor "forward" torque control
byte data2[8] = {0xA1, 0x00, 0x00, 0x00, 0xCE, 0xFF, 0x00, 0x00}; // Motor "reverse" torque control
byte data3[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Software kill switch
byte data4[8] = {0xA2, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xF0}; // Test

unsigned long last_command_time = 0; // To track the time of the last command

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
// LJ X-axis = axes.data[0] (-1 to 1, -1 is to the right, 1 is to the left)
// LJ Y-axis = axes.data[1] (-1 to 1, -1 is down, 1 is up)
// RJ X-axis = axes.data[3] (-1 to 1, -1 is to the right, 1 is to the left)
// RJ Y-axis = axes.data[4] (-1 to 1, -1 is down, 1 is up)

// LT = axes.data[2] (1 if not pressed, -1 fully depressed)
// RT = axes.data[5] (1 if not pressed, -1 fully depressed)

// D-Pad Right =  axes.data[6] will equal (-1)
// D-Pad Left =  axes.data[6] will equal (1)

// D-Pad Up =  axes.data[7] will equal (1)
// D-Pad Down =  axes.data[7] will equal (-1)



void subscription_callback(const void * msgin)
{  
  //Defining the message
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;
  // Update the last command time
  last_command_time = millis();

  // linear_x is the linear speed of the rover and angular_z is the rate of turn
  float linear_x = ((-1 * (joy_msg->axes.data[5]) + 1) / 2) - ((-1 * (joy_msg->axes.data[2]) + 1) / 2);
  float angular_z = -1 * joy_msg->axes.data[0];

  if (joy_msg->buttons.size > 0) {
  int32_t first_button = joy_msg->buttons.data[0];
  digitalWrite(LED_PIN, first_button ? HIGH : LOW);
}

  
  // Differential Drive Equation. Value ranges between 0 and 1
  float velocity_r =  (linear_x + (angular_z * 0.5) / 2);
  float velocity_l = -1* (linear_x - (angular_z * 0.5) / 2);

  // Scale velocities to integer values (e.g., milli-units of RPM)
  int scaled_velocity_r = int(velocity_r * 40000); // Convert float to scaled int
  int scaled_velocity_l = int(velocity_l * 40000);

  // Handle conversion for negative numbers (two's complement)
  byte dir_r = 0x00;
  byte dir_l = 0x00; 

  if (scaled_velocity_r < 0) {
    scaled_velocity_r = 0xFFFF + scaled_velocity_r + 1;
    dir_r = 0xFF;
  }

  if (scaled_velocity_l < 0) {
    scaled_velocity_l = 0xFFFF + scaled_velocity_l + 1;
    dir_l = 0xFF;
  }

  // Extract low and high bytes
  byte low_byte_r = scaled_velocity_r & 0xFF;         // Extract lower 8 bits
  byte high_byte_r = (scaled_velocity_r >> 8) & 0xFF; // Extract upper 8 bits

  byte low_byte_l = scaled_velocity_l & 0xFF;         // Extract lower 8 bits
  byte high_byte_l = (scaled_velocity_l >> 8) & 0xFF; // Extract upper 8 bits

  // Create speed control data packet
  byte speed_command_r[8] = {0xA2, 0x00, 0x00, 0x00, low_byte_r, high_byte_r, dir_r, dir_r};
  byte speed_command_l[8] = {0xA2, 0x00, 0x00, 0x00, low_byte_l, high_byte_l, dir_l, dir_l};

  // Sending Message
  byte sndStat_r1 = CAN0.sendMsgBuf(0x142, 0, 8, speed_command_r); // Send left wheel command
  byte sndStat_r2 = CAN0.sendMsgBuf(0x144, 0, 8, speed_command_l); // Send right wheel command
  byte sndStat_l1 = CAN0.sendMsgBuf(0x143, 0, 8, speed_command_r); // Send right wheel command
  byte sndStat_l2 = CAN0.sendMsgBuf(0x141, 0, 8, speed_command_l); // Send left wheel command



}

//Between here and the DO NOT TOCUH ZONE, insert any new functions. This can be to control the stepper motors, myactuator CANBUS motors etc


//-----------------------
//BEGIN DO NOT TOUCH ZONE
//-----------------------

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  Serial.begin(9600);
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

  //CAN Initialisation
    while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {

    delay(100);
  }
  CAN0.setMode(MCP_NORMAL); // Set CAN mode
}

//---------------------
//END DO NOT TOUCH ZONE
//---------------------

void loop() {
  //Executer Spinning. This line checks for a new message on the /joy topic at 10
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Check for 2-second timeout
  if (millis() - last_command_time > 2000) {
    // Send the software kill switch message (data3)
    CAN0.sendMsgBuf(0x144, 0, 8, data3);
    CAN0.sendMsgBuf(0x142, 0, 8, data3);
    CAN0.sendMsgBuf(0x143, 0, 8, data3);
    CAN0.sendMsgBuf(0x141, 0, 8, data3);

    
    Serial.println("Sent software kill switch message");
    
    // Optionally turn off the LED as well when sending the kill switch message
    digitalWrite(LED_PIN, HIGH);
}
}
