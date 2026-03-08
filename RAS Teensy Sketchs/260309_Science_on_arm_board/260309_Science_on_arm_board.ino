//-----------------------
// BEGIN DO NOT TOUCH ZONE
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
// END DO NOT TOUCH ZONE
//---------------------

// =====================================================
// USER CONTROL / HARDWARE SECTION
// =====================================================

// ------------------ CAN ------------------
#include <SPI.h>
#include <mcp_can.h>
MCP_CAN CAN0(10);

// Your provided CAN command
byte can_cmd_down[8] = {0xA2, 0x00, 0x00, 0x00, 0x46, 0x50, 0xFF, 0xFF};
byte can_cmd_stop[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Opposite direction version of your A2 command.
// This assumes the last 4 bytes are a signed little-endian speed/current target.
// If your motor behaves backwards, just swap can_cmd_up and can_cmd_down.
byte can_cmd_up[8]   = {0xA2, 0x00, 0x00, 0x00, 0xBA, 0xAF, 0x00, 0x00};

const uint16_t CAN_ID = 0x141;

// ------------------ ACCELSTEPPER ------------------
#include <AccelStepper.h>

// Stepper pins
#define dirPin1    39
#define stepPin1   40
#define engagePin1 41

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);

// ------------------ SERVO ------------------
#include <Servo.h>
Servo myServo;

// Use a real free pin on your Teensy here
#define SERVO_PIN 14

int servo_angle = 90;
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const int SERVO_STEP = 2;

// ------------------ CONTROL STATE ------------------
volatile int can_direction = 0;     //  1 = up, -1 = down, 0 = stop
volatile int servo_direction = 0;   // -1 = left, 1 = right, 0 = stop

unsigned long last_can_send_ms = 0;
unsigned long last_servo_update_ms = 0;

const unsigned long CAN_SEND_INTERVAL_MS = 20;
const unsigned long SERVO_UPDATE_INTERVAL_MS = 20;

// =====================================================
// Helper functions
// =====================================================

void setup_accelstepper1()
{
  stepper1.setEnablePin(engagePin1);
  stepper1.setPinsInverted(false, false, true);   // enable pin active LOW
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(300.0);
  stepper1.enableOutputs();
  stepper1.setSpeed(0.0);
}

void send_can_frame(byte *frame)
{
  byte status = CAN0.sendMsgBuf(CAN_ID, 0, 8, frame);
  if (status != CAN_OK) {
    // optional debug
    // Serial.println("CAN send failed");
  }
}

void handle_can_output()
{
  unsigned long now = millis();
  if (now - last_can_send_ms < CAN_SEND_INTERVAL_MS) return;
  last_can_send_ms = now;

  if (can_direction > 0) {
    send_can_frame(can_cmd_up);
  }
  else if (can_direction < 0) {
    send_can_frame(can_cmd_down);
  }
  else {
    send_can_frame(can_cmd_stop);
  }
}

void handle_servo_output()
{
  unsigned long now = millis();
  if (now - last_servo_update_ms < SERVO_UPDATE_INTERVAL_MS) return;
  last_servo_update_ms = now;

  if (servo_direction < 0) {
    servo_angle -= SERVO_STEP;
    if (servo_angle < SERVO_MIN) servo_angle = SERVO_MIN;
    myServo.write(servo_angle);
  }
  else if (servo_direction > 0) {
    servo_angle += SERVO_STEP;
    if (servo_angle > SERVO_MAX) servo_angle = SERVO_MAX;
    myServo.write(servo_angle);
  }
}

// =====================================================
// This is the subscription callback for /joy
// =====================================================

// Button Definitions:
// A = buttons.data[0]
// B = buttons.data[1]
// X = buttons.data[2]
// Y = buttons.data[3]
// LB = buttons.data[4]
// RB = buttons.data[5]

// Axes Definitions:
// D-Pad Right = axes.data[6] == -1
// D-Pad Left  = axes.data[6] ==  1
// D-Pad Up    = axes.data[7] ==  1
// D-Pad Down  = axes.data[7] == -1

void subscription_callback(const void * msgin)
{
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;

  // -----------------------------
  // Read buttons safely
  // -----------------------------
  int a_pressed = 0;
  int y_pressed = 0;

  if (joy_msg->buttons.size > 3) {
    a_pressed = joy_msg->buttons.data[0];
    y_pressed = joy_msg->buttons.data[3];
  }

  // -----------------------------
  // Read D-pad safely
  // -----------------------------
  float dpad_lr = 0.0f;   // axes[6]
  float dpad_ud = 0.0f;   // axes[7]

  if (joy_msg->axes.size > 7) {
    dpad_lr = joy_msg->axes.data[6];
    dpad_ud = joy_msg->axes.data[7];
  }

  // -----------------------------
  // Stepper control on Y / A
  // A = one direction
  // Y = opposite direction
  // -----------------------------
    if (a_pressed == 1) {
    stepper1.setSpeed(-400.0);
  }
  else if (y_pressed == 1) {
    stepper1.setSpeed(400.0);
  }
  else {
    stepper1.setSpeed(0.0);
  }

  // -----------------------------
  // CAN control on D-pad Up / Down
  // -----------------------------
    if (dpad_ud > 0.5f) {
    can_direction = -1;   // reversed
  }
  else if (dpad_ud < -0.5f) {
    can_direction = 1;    // reversed
  }
  else {
    can_direction = 0;
  }

  // -----------------------------
  // Servo control on D-pad Left / Right
  // -----------------------------
  if (dpad_lr > 0.5f) {
    servo_direction = -1;   // left
  }
  else if (dpad_lr < -0.5f) {
    servo_direction = 1;    // right
  }
  else {
    servo_direction = 0;    // stop
  }

  // LED on whenever any main control is active
  if (a_pressed || y_pressed || (dpad_ud > 0.5f) || (dpad_ud < -0.5f) || (dpad_lr > 0.5f) || (dpad_lr < -0.5f)) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

//-----------------------
// BEGIN DO NOT TOUCH ZONE
//-----------------------

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // -----------------------------
  // Hardware init
  // -----------------------------
  Serial.begin(115200);

  // Stepper pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(engagePin1, OUTPUT);
  setup_accelstepper1();

  // Servo
  myServo.attach(SERVO_PIN);
  myServo.write(servo_angle);

  // CAN
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    delay(100);
  }
  CAN0.setMode(MCP_NORMAL);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_joy_science_node", "", &support));

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
// END DO NOT TOUCH ZONE
//---------------------

void loop() {
  // Check for new /joy message
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  // Run stepper continuously at the speed last set in callback
  stepper1.runSpeed();

  // Continuously command CAN while D-pad up/down held
  handle_can_output();

  // Continuously move servo while D-pad left/right held
  handle_servo_output();
}