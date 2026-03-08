#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include Int32 message
#include <std_msgs/msg/int32.h>

//Horizontal Movement
#define dirPin1 26
#define stepPin1 27
#define engagePin1 1

//Vetical Movement
#define dirPin2 23
#define stepPin2 24
#define engagePin2 2

//Emag
#define magPin 8

//Spin
#define dirPin3 18
#define stepPin3 17
#define engagePin3 3

int dir_tracker = 0;
int mag_tracker = 0;

bool stepper1 = false;
bool stepper2 = false;



// Global Variables
rcl_subscription_t subscriber;
// Use Int32 message type
std_msgs__msg__Int32 button_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// Timer is not used; remove if unnecessary
// rcl_timer_t timer;

#define LED_PIN 13

// Error Handling Macros
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){error_loop();}}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){}}

// Error Loop: Blink LED rapidly to indicate an error
void error_loop(){
  // Blink LED rapidly
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Callback function for /button_states
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * button_msg_in = (const std_msgs__msg__Int32 *)msgin;

  // Extract the button bitmask
  int32_t button_bitmask = button_msg_in->data;

  // Decode button states from bitmask
  int a_pressed = (button_bitmask & (1 << 0)) ? 1 : 0;
  int y_pressed = (button_bitmask & (1 << 2)) ? 1 : 0; 
  int b_pressed = (button_bitmask & (1 << 1)) ? 1 : 0;
  int x_pressed = (button_bitmask & (1 << 3)) ? 1 : 0; 
  int rb_pressed = (button_bitmask & (1 << 5)) ? 1 : 0;
  int lb_pressed = (button_bitmask & (1 << 4)) ? 1 : 0; 

  // Control base actuators based on A and B buttons
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



    if(lb_pressed == 1)
  {
    if (dir_tracker == 0) {
    digitalWrite(engagePin3, LOW);
    digitalWrite(dirPin3, LOW);
    move_stepper3();
    dir_tracker++;
    }
    else if (dir_tracker == 1)
    {
    digitalWrite(engagePin3, LOW);
    digitalWrite(dirPin3, HIGH);
    move_stepper3();
    dir_tracker--;
    }
  }
  
  else
  {
  digitalWrite(engagePin3, HIGH);
  }


    if(rb_pressed == 1)
  {
    if (mag_tracker == 0) {
    digitalWrite(magPin, LOW);
    mag_tracker++;
    }
    else if (dir_tracker == 1)
    {
    digitalWrite(magPin, HIGH);
    mag_tracker--;
    }
    delay(500);
  }



  // Update LED state based on A button
  digitalWrite(LED_PIN, (a_pressed) ? HIGH : LOW); 
}

// Function to control top actuator
void move_stepper1()
{
  digitalWrite(stepPin1, HIGH);
  delayMicroseconds(200);  // Adjust the delay for speed control
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(200);
}
void move_stepper2()
{
  digitalWrite(stepPin2, HIGH);
  delayMicroseconds(500);  // Adjust the delay for speed control
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(500);
}
void move_stepper3()
{
  for (int i = 1; i<100; i++){
  digitalWrite(stepPin3, HIGH);
  delayMicroseconds(500);  // Adjust the delay for speed control
  digitalWrite(stepPin3, LOW);
  delayMicroseconds(500);
  }
  //Delay so it only moves once
  delay(500);
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for serial port to connect (useful for native USB)

  set_microros_transports();
  
  //Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 


  //Set Pins to Output
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(engagePin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(engagePin2, OUTPUT);

  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(engagePin3, OUTPUT);

  pinMode(magPin, OUTPUT);


//Setting Initial Outputs
  digitalWrite(engagePin1, LOW);
  digitalWrite(dirPin1, HIGH); // Set Initial Direction

  digitalWrite(engagePin2, LOW);
  digitalWrite(dirPin2, HIGH); // Set Initial Direction

  digitalWrite(engagePin3, LOW);
  digitalWrite(dirPin3, HIGH); // Set Initial Direction
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "scoop_control", "", &support));

  // Create subscriber for /button_states
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/button_states"));

  // Initialize the Int32 message
  std_msgs__msg__Int32__init(&button_msg);

  // Create executor with one handle (subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  // Add the subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &button_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    if (stepper1) {
    move_stepper1();  // This pulse loop now runs continuously
  }

  if (stepper2) {
    move_stepper2();  // This pulse loop now runs continuously
  }

}
