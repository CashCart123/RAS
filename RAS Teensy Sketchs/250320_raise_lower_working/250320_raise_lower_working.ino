#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>

//Include CAN
#include <SPI.h>
#include <mcp_can.h>
MCP_CAN CAN0(10); 
byte data[8] = {0xA2, 0x00, 0x00, 0x00, 0x46, 0x50, 0xFF, 0xFF};
byte data3[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ------------------ ACCELSTEPPER ------------------
#include <AccelStepper.h>

// Drill Raise/Lower Pins
#define dirPin1    21
#define stepPin1   20
#define engagePin1  0

// Drill
#define drillPin    8

// Heat Cover
#define dirPin2     23
#define stepPin2    24
#define engagePin2   25

// Microscope
#define dirPin3     25
#define stepPin3    29
#define engagePin3   3

// Lightbulb
#define lightPin   100

// Heat Probe
#define probePin   101

// Vacuum pump
#define vacuumPin  102

int drill_tracker = 0;
int light_tracker = 0;
int probe_tracker = 0;
int vacuum_tracker = 0;

int prev_lb_pressed  = 0; 
int prev_rb_pressed  = 0; 
int prev_keypad_1    = 0; 
int prev_keypad_2    = 0; 

// We'll keep stepper2, stepper3 as they are (manual toggling).
// This example only updates stepper1 to use AccelStepper.
bool stepper2 = false;
bool stepper3 = false;

// ------------------ NEW: AccelStepper object for stepper1
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);

// Global micro-ROS variables
rcl_subscription_t subscriber;
std_msgs__msg__Int32 button_msg;

rcl_subscription_t keypad_subscriber;
std_msgs__msg__Int32 keypad_msg;

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
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// -----------------------------
// Callback for /button_states
// -----------------------------
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * button_msg_in = 
    (const std_msgs__msg__Int32 *)msgin;

  int32_t button_bitmask = button_msg_in->data;

  // Extract relevant buttons from bitmask
  int a_pressed  = (button_bitmask & (1 << 0)) ? 1 : 0; // A
  int y_pressed  = (button_bitmask & (1 << 2)) ? 1 : 0; // Y
  int b_pressed  = (button_bitmask & (1 << 1)) ? 1 : 0; // B
  int x_pressed  = (button_bitmask & (1 << 3)) ? 1 : 0; // X
  int rb_pressed = (button_bitmask & (1 << 5)) ? 1 : 0; // RB
  int lb_pressed = (button_bitmask & (1 << 4)) ? 1 : 0; // LB

  // -------------------------------
  // STEPPER #1 via AccelStepper
  // -------------------------------
  // If A pressed => move "down" (arbitrary direction)
  // If Y pressed => move "up"
  // Otherwise => stop
  if (a_pressed == 1) {
    // Positive speed
    stepper1.setSpeed(400.0);  // adjust to your desired steps/sec
  } else if (y_pressed == 1) {
    // Negative speed
    stepper1.setSpeed(-400.0);
  } else {
    // No movement
    stepper1.setSpeed(0.0);
  }

  // -------------------------------
  // STEPPER #2
  // -------------------------------
  if (b_pressed == 1)
  {
    digitalWrite(engagePin2, LOW);
    digitalWrite(dirPin2, LOW);
    stepper2 = true;
  }
  else if (x_pressed == 1)
  {
    digitalWrite(engagePin2, LOW);
    digitalWrite(dirPin2, HIGH);
    stepper2 = true;
  }
  else
  {
    digitalWrite(engagePin2, HIGH);
    stepper2 = false;
  }

  // -------------------------------
  // Light (toggle on LB press)
  // -------------------------------
  if (lb_pressed == 1 && prev_lb_pressed == 0)
  {
    if (light_tracker == 0) {
      digitalWrite(lightPin, HIGH);
      light_tracker = 1;
    } else {
      digitalWrite(lightPin, LOW);
      light_tracker = 0;
    }
  }
  prev_lb_pressed = lb_pressed;

  // -------------------------------
  // Drill (toggle on RB press)
  // -------------------------------
  if (rb_pressed == 1 && prev_rb_pressed == 0)
  {
    if (drill_tracker == 0) {
      drill_tracker = 1;
      byte sendStatus = CAN0.sendMsgBuf(0x141, 0, 8, data);
    } else {
      drill_tracker = 0;
      byte sendStatus = CAN0.sendMsgBuf(0x141, 0, 8, data3);
    }
  }
  prev_rb_pressed = rb_pressed;

  // Optional: LED shows RB press
  digitalWrite(LED_PIN, (rb_pressed) ? HIGH : LOW); 
}

// -----------------------------
// Callback for /keypad_states
// -----------------------------
void subscription_callback_keypad(const void * msgin)
{
  const std_msgs__msg__Int32 * keypad_msg_in = 
    (const std_msgs__msg__Int32 *) msgin;

  int32_t keypad_state = keypad_msg_in->data;

  // Microscope movement
  if (keypad_state == 3)
  {
    digitalWrite(engagePin3, LOW);
    digitalWrite(dirPin3, LOW);
    // or your existing step function:
    // move_stepper3();
    stepper3 = true; 
  }
  else if (keypad_state == 4)
  {
    digitalWrite(engagePin3, LOW);
    digitalWrite(dirPin3, HIGH);
    // move_stepper3();
    stepper3 = true; 
  }
  else
  {
    digitalWrite(engagePin3, HIGH);
    stepper3 = false;
  }
  
  // Vacuum toggle on keypad_state == 1
  int key1_pressed = (keypad_state == 1) ? 1 : 0;
  if (key1_pressed == 1 && prev_keypad_1 == 0)
  {
    if (vacuum_tracker == 0) {
      digitalWrite(vacuumPin, HIGH);
      vacuum_tracker = 1;
    } else {
      digitalWrite(vacuumPin, LOW);
      vacuum_tracker = 0;
    }
  }
  prev_keypad_1 = key1_pressed;

  // Probe toggle on keypad_state == 2
  int key2_pressed = (keypad_state == 2) ? 1 : 0;
  if (key2_pressed == 1 && prev_keypad_2 == 0)
  {
    if (probe_tracker == 0) {
      digitalWrite(probePin, HIGH);
      probe_tracker = 1;
    } else {
      digitalWrite(probePin, LOW);
      probe_tracker = 0;
    }
  }
  prev_keypad_2 = key2_pressed;
}

// ---------------------------------
// Setup for AccelStepper #1
// ---------------------------------
void setup_accelstepper1()
{
  // If your DRV8825 requires enable LOW for active, invert the enable pin:
  // The third param in setPinsInverted() inverts the enable pin
  stepper1.setEnablePin(engagePin1);
  stepper1.setPinsInverted(false, false, true);  // invert enable pin
  stepper1.setMaxSpeed(1000.0);      // steps/s
  stepper1.setAcceleration(300.0);   // steps/s^2
  stepper1.enableOutputs();
}

void setup() {
  set_microros_transports();
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  // Step/Dir/Enable pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1,  OUTPUT);
  pinMode(engagePin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2,  OUTPUT);
  pinMode(engagePin2, OUTPUT);

  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3,  OUTPUT);
  pinMode(engagePin3, OUTPUT);

  pinMode(lightPin,  OUTPUT);
  pinMode(probePin,  OUTPUT);
  pinMode(vacuumPin, OUTPUT);
  pinMode(drillPin,  OUTPUT);

  // Initialize stepper #1 with AccelStepper
  setup_accelstepper1();
  
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "scoop_control", "", &support));

  // Subscriber for /button_states
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/button_states"));
  std_msgs__msg__Int32__init(&button_msg);

  // Subscriber for /keypad_states
  RCCHECK(rclc_subscription_init_default(
    &keypad_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/keypad_states"));
  std_msgs__msg__Int32__init(&keypad_msg);

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &button_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &keypad_subscriber, &keypad_msg, &subscription_callback_keypad, ON_NEW_DATA));

  // Setup CAN
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("MCP2515 Initializing...");
    delay(100);
  }
  Serial.println("Initialized MCP2515");
  CAN0.setMode(MCP_NORMAL);
  delay(1000);
}

void move_stepper2() {
  // same as your original code
  for (int i = 0; i < 3; i++) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(200);
  }
}
void move_stepper3() {
  // same as your original code
  digitalWrite(stepPin3, HIGH);
  delayMicroseconds(200);
  digitalWrite(stepPin3, LOW);
  delayMicroseconds(200);
}

void loop() {
  // micro-ROS spin
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  // --- Run AccelStepper #1 at the speed set by our callback logic
  //     (If speed=0, it won't move; if speed>0 or <0, it goes in that direction)
  stepper1.runSpeed();

  // --- Stepper #2 (still manual toggling)
  if (stepper2) {
    move_stepper2();
  }

  // --- Stepper #3 (still manual toggling)
  if (stepper3) {
    move_stepper3();
  }
}
