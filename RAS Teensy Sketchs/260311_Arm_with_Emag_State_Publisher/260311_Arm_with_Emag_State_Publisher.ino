#include <micro_ros_arduino.h>

#include <stdio.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/bool.h>

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
#define BTN_A          0
#define BTN_B          1
#define BTN_RB         5
#define AXIS_DPAD_VERT 7

// ===================== SERVO SETTINGS =====================
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_START_ANGLE   90
#define SERVO_STEP_DEG      2
#define SERVO_REPEAT_MS     40

// ===================== EMAG PUBLISH =====================
#define EMAG_PUBLISH_MS 200

// ===================== GLOBALS =====================
rcl_subscription_t subscriber;
sensor_msgs__msg__Joy msg;

rcl_publisher_t emag_pub;
std_msgs__msg__Bool emag_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

Servo my_servo;

bool stepper_run = false;
int servo_angle = SERVO_START_ANGLE;
bool emag_state = false;

int prev_rb_pressed = 0;

unsigned long last_servo_update_ms = 0;
unsigned long last_emag_publish_ms = 0;

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
  digitalWrite(STEP_EN_PIN, HIGH);
  digitalWrite(EMAG_PIN, LOW);

  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ===================== STEPPER =====================
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

  if ((now - last_servo_update_ms) < SERVO_REPEAT_MS) return;

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

  if ((int)joy_msg->buttons.size > BTN_A)
    a_pressed = joy_msg->buttons.data[BTN_A];

  if ((int)joy_msg->buttons.size > BTN_B)
    b_pressed = joy_msg->buttons.data[BTN_B];

  if ((int)joy_msg->buttons.size > BTN_RB)
    rb_pressed = joy_msg->buttons.data[BTN_RB];

  if ((int)joy_msg->axes.size > AXIS_DPAD_VERT) {
    float axis_val = joy_msg->axes.data[AXIS_DPAD_VERT];

    if (axis_val > 0.5f) dpad_vertical = 1;
    else if (axis_val < -0.5f) dpad_vertical = -1;
  }

  // ---------- Stepper ----------
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

  // ---------- Servo ----------
  update_servo_from_dpad(dpad_vertical);

  // ---------- EMAG toggle ----------
  if (rb_pressed == 1 && prev_rb_pressed == 0) {
    emag_state = !emag_state;

    digitalWrite(EMAG_PIN, emag_state ? HIGH : LOW);

    emag_msg.data = emag_state;
    RCSOFTCHECK(rcl_publish(&emag_pub, &emag_msg, NULL));
  }

  prev_rb_pressed = rb_pressed;

  digitalWrite(LED_PIN, (stepper_run || emag_state) ? HIGH : LOW);
}

// ===================== SETUP =====================
void setup() {

  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(STEP_STEP_PIN, OUTPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_EN_PIN, OUTPUT);

  digitalWrite(STEP_STEP_PIN, LOW);
  digitalWrite(STEP_DIR_PIN, HIGH);
  digitalWrite(STEP_EN_PIN, HIGH);

  pinMode(EMAG_PIN, OUTPUT);
  digitalWrite(EMAG_PIN, LOW);

  my_servo.attach(SERVO_PIN);
  my_servo.write(servo_angle);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "aux_control_board", "", &support));

  // EMAG STATE PUBLISHER
  RCCHECK(rclc_publisher_init_default(
    &emag_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/emag_state"));

  // JOY SUBSCRIBER
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

  std_msgs__msg__Bool__init(&emag_msg);
  emag_msg.data = false;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW);
}

// ===================== LOOP =====================
void loop() {

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  if (stepper_run) {
    move_stepper_single();
  }

  // periodic EMAG publish
  unsigned long now = millis();

  if (now - last_emag_publish_ms > EMAG_PUBLISH_MS) {

    emag_msg.data = emag_state;

    RCSOFTCHECK(rcl_publish(&emag_pub, &emag_msg, NULL));

    last_emag_publish_ms = now;
  }
}