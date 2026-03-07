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

//Drill Raise Lower
#define dirPin1 19
#define stepPin1 18
#define engagePin1 4

//Drill
#define drillPin 8

//Heat Cover
#define dirPin2 21
#define stepPin2 20
#define engagePin2 0

//Microscope
#define dirPin3 23
#define stepPin3 25
#define engagePin3 11

//Lightbulb
#define lightPin 12

//Heat Probe
#define probePin 15

//Vacuum pump
#define vacuumPin 14

int drill_tracker = 0;
int light_tracker = 0;
int probe_tracker = 0;
int vacuum_tracker = 0;

// Store the *previous* pressed states for toggle buttons
// so we only toggle on the rising edge of each press
int prev_lb_pressed  = 0; 
int prev_rb_pressed  = 0; 
int prev_keypad_1    = 0; 
int prev_keypad_2    = 0; 

bool stepper1 = false;
bool stepper2 = false;
bool stepper3 = false;

// Global Variables
rcl_subscription_t subscriber;          
std_msgs__msg__Int32 button_msg;

rcl_subscription_t keypad_subscriber;   
std_msgs__msg__Int32 keypad_msg;       

rcl_publisher_t publisher;
sensor_msgs__msg__JointState joint_state_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

// Error Handling Macros
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
// Timer callback: Publish states
// -----------------------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) {
    return;
  }

  // vacuum_tracker → index 0
  joint_state_msg.position.data[0] = vacuum_tracker;
  joint_state_msg.velocity.data[0] = 0;

  // light_tracker → index 1
  joint_state_msg.position.data[1] = light_tracker;
  joint_state_msg.velocity.data[1] = 0;

  // probe_tracker → index 2
  joint_state_msg.position.data[2] = probe_tracker;
  joint_state_msg.velocity.data[2] = 0;

  // drill_tracker → index 3
  joint_state_msg.position.data[3] = drill_tracker;
  joint_state_msg.velocity.data[3] = 0;

  // Publish the updated JointState message
  RCSOFTCHECK(rcl_publish(&publisher, &joint_state_msg, NULL));
}

// -----------------------------
// Callback for /button_states
// -----------------------------
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * button_msg_in = 
    (const std_msgs__msg__Int32 *)msgin;

  // Extract the button bitmask
  int32_t button_bitmask = button_msg_in->data;

  // Decode button states from bitmask
  int a_pressed  = (button_bitmask & (1 << 0)) ? 1 : 0;
  int y_pressed  = (button_bitmask & (1 << 2)) ? 1 : 0; 
  int b_pressed  = (button_bitmask & (1 << 1)) ? 1 : 0;
  int x_pressed  = (button_bitmask & (1 << 3)) ? 1 : 0; 
  int rb_pressed = (button_bitmask & (1 << 5)) ? 1 : 0;
  int lb_pressed = (button_bitmask & (1 << 4)) ? 1 : 0; 

  // Drill Raise and Lower
  if(a_pressed == 1)
  {
    digitalWrite(engagePin1, LOW);
    digitalWrite(dirPin1, LOW);
    stepper1 = true;
  }
  else if(y_pressed == 1)
  {
    digitalWrite(engagePin1, LOW);
    digitalWrite(dirPin1, HIGH);
    stepper1 = true;
  }
  else
  {
    digitalWrite(engagePin1, HIGH);
    stepper1 = false;
  }

  // Heat Cover Raise and Lower
  if(b_pressed == 1)
  {
    digitalWrite(engagePin2, LOW);
    digitalWrite(dirPin2, LOW);
    stepper2 = true;
  }
  else if(x_pressed == 1)
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

  // Heating Light power on/off (toggle on LB press)
  // Only toggle on a *new* press: lb_pressed == 1 && prev_lb_pressed == 0
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
  // update previous LB
  prev_lb_pressed = lb_pressed;

  // Drill power on/off (toggle on RB press)
  if (rb_pressed == 1 && prev_rb_pressed == 0)
  {
    if (drill_tracker == 0) {
      // Start drill
      drill_tracker = 1;
      digitalWrite(drillPin, HIGH); // or however you run it
    } else {
      // Stop drill
      drill_tracker = 0;
      digitalWrite(drillPin, LOW);
    }
  }
  prev_rb_pressed = rb_pressed;

  // Update LED state based on RB button
  digitalWrite(LED_PIN, (rb_pressed) ? HIGH : LOW); 
}

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
    move_stepper3();
  }
  else if (keypad_state == 4)
  {
    digitalWrite(engagePin3, LOW);
    digitalWrite(dirPin3, HIGH);
    move_stepper3();
  }
  else
  {
    digitalWrite(engagePin3, HIGH);
  }
  // Vacuum toggle on keypad_state == 1
  // Only toggle on rising edge
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

// -----------------------------
// Stepper Movement Functions
// -----------------------------
void move_stepper1()
{
  digitalWrite(stepPin1, HIGH);
  delayMicroseconds(200);  // Adjust for speed control
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(200);
}

void move_stepper2()
{
  digitalWrite(stepPin2, HIGH);
  delayMicroseconds(200);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(200);
}

void move_stepper3()
{
  digitalWrite(stepPin3, HIGH);
  delayMicroseconds(200);
  digitalWrite(stepPin3, LOW);
  delayMicroseconds(200);
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; }

  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(engagePin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(engagePin2, OUTPUT);

  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(engagePin3, OUTPUT);

  pinMode(lightPin, OUTPUT);
  pinMode(probePin, OUTPUT);
  pinMode(vacuumPin, OUTPUT);
  pinMode(drillPin, OUTPUT);

  // Setting Initial Outputs
  digitalWrite(engagePin1, LOW);
  digitalWrite(dirPin1, HIGH);

  digitalWrite(engagePin2, LOW);
  digitalWrite(dirPin2, HIGH);

  digitalWrite(engagePin3, LOW);
  digitalWrite(dirPin3, HIGH);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "scoop_control", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/science_power_states"));

  // Initialize the JointState message (publisher)
    // Initialize the JointState message (publisher)
  sensor_msgs__msg__JointState__init(&joint_state_msg);

  static rosidl_runtime_c__String joint_names[4];
  joint_names[0].data = (char *)"Vacuum";
  joint_names[0].size = 6;
  joint_names[0].capacity = 7;
  joint_names[1].data = (char *)"Light";
  joint_names[1].size = 5;
  joint_names[1].capacity = 6;
  joint_names[2].data = (char *)"Probe";
  joint_names[2].size = 5;
  joint_names[2].capacity = 6;
  joint_names[3].data = (char *)"Drill";
  joint_names[3].size = 5;
  joint_names[3].capacity = 6;
  joint_state_msg.name.data = joint_names;
  joint_state_msg.name.size = 4;
  joint_state_msg.name.capacity = 4;
  
  // Prepare positions and velocities for four joints
  static double positions[4] = {0, 0, 0, 0};
  static double velocities[4] = {0, 0, 0, 0};
  joint_state_msg.position.data = positions;
  joint_state_msg.position.size = 4;
  joint_state_msg.position.capacity = 4;
  joint_state_msg.velocity.data = velocities;
  joint_state_msg.velocity.size = 4;
  joint_state_msg.velocity.capacity = 4;
  
  // Effort field (not used in this example)
  joint_state_msg.effort.data = NULL;
  joint_state_msg.effort.size = 0;
  joint_state_msg.effort.capacity = 0;
  // --- End of publisher message setup ---


  // 1) Create subscriber for /button_states
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/button_states"));

  std_msgs__msg__Int32__init(&button_msg);

  // 2) Create subscriber for /keypad_states
  RCCHECK(rclc_subscription_init_default(
    &keypad_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/keypad_states"));

  std_msgs__msg__Int32__init(&keypad_msg);

  // Create timer for publishing
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Executor handles both subscriptions + the timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &button_msg,
            &subscription_callback,
            ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
            &executor,
            &keypad_subscriber,
            &keypad_msg,
            &subscription_callback_keypad,
            ON_NEW_DATA));

  // If you use MCP_CAN, configure it here...
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("MCP2515 Initializing...");
    delay(100);
  }
  Serial.println("Initialized MCP2515");
  CAN0.setMode(MCP_NORMAL); // Set to normal mode
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  if (stepper1) {
    move_stepper1();  // This pulse loop now runs continuously
  }

  if (stepper2) {
    move_stepper2();  // This pulse loop now runs continuously
  }

  if (stepper3) {
    move_stepper3();  // This pulse loop now runs continuously
  }
}
