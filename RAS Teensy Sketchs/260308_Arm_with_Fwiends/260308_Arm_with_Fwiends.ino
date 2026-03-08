#include <micro_ros_arduino.h>

#include <stdio.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

#define MAX_AXES 8
#define MAX_BUTTONS 16

// ===================== PINS =====================
#define LED_PIN        13

// Stepper (A4988)
#define STEP_DIR_PIN   39
#define STEP_STEP_PIN  40
#define STEP_EN_PIN    41   // LOW = enable, HIGH = disable

// Other outputs
#define SERVO_PIN      14
#define EMAG_PIN       15

// ===================== JOY MAPPING =====================
// Common mapping, but may vary depending on your joy node/controller
#define BTN_A          0
#define BTN_B          1
#define BTN_RB         5
#define AXIS_DPAD_VERT 7   // +1 up, -1 down on many mappings

// ===================== SERVO SETTINGS =====================
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_START_ANGLE   90
#define SERVO_STEP_DEG      2
#define SERVO_REPEAT_MS     40   // repeat rate while holding D-pad

// ===================== GLOBALS =====================
rcl_subscription_t subscriber;
sensor_msgs__msg__Joy msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

Servo my_servo;

bool stepper_run = false;
int servo_angle = SERVO_START_ANGLE;
bool emag_state = false;

// edge detection for RB toggle
int prev_rb_pressed = 0;

// servo repeat timing
unsigned long last_servo_update_ms = 0;

// ===================== ERROR HANDLING =====================
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) {} \
}

void error_loop() {
  digitalWrite(STEP_EN_PIN, HIGH);   // disable stepper
  digitalWrite(EMAG_PIN, LOW);       // emag off
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ===================== STEPPER FUNCTION =====================
void move_stepper_single()
{
  digitalWrite(STEP_STEP_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(STEP_STEP_PIN, LOW);
  delayMicroseconds(100);
}

// ===================== SERVO HELPERS =====================
int clamp_angle(int value) {
  if (value < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
  if (value > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
  return value;
}

void update_servo_from_dpad(int dpad_vertical)
{
  unsigned long now = millis();

  if ((now - last_servo_update_ms) < SERVO_REPEAT_MS) {
    return;
  }

  if (dpad_vertical > 0) {
    servo_angle = clamp_angle(servo_angle - SERVO_STEP_DEG);
    my_servo.write(servo_angle);
    last_servo_update_ms = now;
}
else if (dpad_vertical < 0) {
    servo_angle = clamp_angle(servo_angle + SERVO_STEP_DEG);
    my_servo.write(servo_angle);
    last_servo_update_ms = now;
}
}

// ===================== JOY CALLBACK =====================
void subscription_callback(const void * msgin)
{
  const sensor_msgs__msg__Joy * joy_msg = (const sensor_msgs__msg__Joy *)msgin;

  int a_pressed = 0;
  int b_pressed = 0;
  int rb_pressed = 0;
  int dpad_vertical = 0;

  // Buttons
  if ((int)joy_msg->buttons.size > BTN_A) {
    a_pressed = joy_msg->buttons.data[BTN_A];
  }
  if ((int)joy_msg->buttons.size > BTN_B) {
    b_pressed = joy_msg->buttons.data[BTN_B];
  }
  if ((int)joy_msg->buttons.size > BTN_RB) {
    rb_pressed = joy_msg->buttons.data[BTN_RB];
  }

  // D-pad vertical axis
  if ((int)joy_msg->axes.size > AXIS_DPAD_VERT) {
    float axis_val = joy_msg->axes.data[AXIS_DPAD_VERT];
    if (axis_val > 0.5f) {
      dpad_vertical = 1;
    } else if (axis_val < -0.5f) {
      dpad_vertical = -1;
    } else {
      dpad_vertical = 0;
    }
  }

  // ---------- Stepper control ----------
  if (a_pressed == 1 && b_pressed == 0) {
    digitalWrite(STEP_EN_PIN, LOW);
    digitalWrite(STEP_DIR_PIN, LOW);
    stepper_run = true;
  }
  else if (b_pressed == 1 && a_pressed == 0) {
    digitalWrite(STEP_EN_PIN, LOW);
    digitalWrite(STEP_DIR_PIN, HIGH);
    stepper_run = true;
  }
  else {
    digitalWrite(STEP_EN_PIN, HIGH);
    stepper_run = false;
  }

  // ---------- Servo control ----------
  update_servo_from_dpad(dpad_vertical);

  // ---------- EMAG toggle on RB rising edge ----------
  if (rb_pressed == 1 && prev_rb_pressed == 0) {
    emag_state = !emag_state;
    digitalWrite(EMAG_PIN, emag_state ? HIGH : LOW);
  }
  prev_rb_pressed = rb_pressed;

  // LED on if any output active
  digitalWrite(LED_PIN, (stepper_run || emag_state) ? HIGH : LOW);
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Stepper pins
  pinMode(STEP_STEP_PIN, OUTPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_EN_PIN, OUTPUT);
  digitalWrite(STEP_STEP_PIN, LOW);
  digitalWrite(STEP_DIR_PIN, HIGH);
  digitalWrite(STEP_EN_PIN, HIGH);   // disabled at startup

  // EMAG
  pinMode(EMAG_PIN, OUTPUT);
  digitalWrite(EMAG_PIN, LOW);

  // Servo
  my_servo.attach(SERVO_PIN);
  my_servo.write(servo_angle);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "aux_control_board", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"));

  sensor_msgs__msg__Joy__init(&msg);

  msg.axes.data = (float *)allocator.allocate(sizeof(float) * MAX_AXES, allocator.state);
  msg.axes.capacity = MAX_AXES;
  msg.axes.size = 0;

  msg.buttons.data = (int32_t *)allocator.allocate(sizeof(int32_t) * MAX_BUTTONS, allocator.state);
  msg.buttons.capacity = MAX_BUTTONS;
  msg.buttons.size = 0;

  if (msg.axes.data == NULL || msg.buttons.data == NULL) {
    error_loop();
  }

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  // Proven loop-driven stepper control
  if (stepper_run) {
    move_stepper_single();
  }
}