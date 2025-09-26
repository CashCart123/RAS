//-----------------------
//BEGIN DO NOT TOUCH ZONE
//-----------------------

#include <micro_ros_arduino.h>

#include <mcp_can.h>
#include <SPI.h>

#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point.h>

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t pos_timer;   // 50 Hz publisher timer
rcl_timer_t stat_timer;  // 20 Hz status poll timer  (NEW)

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){ while(1){ delay(100); } }

//---------------------
//END DO NOT TOUCH ZONE
//---------------------

//================ USER CONFIG =================
#define JOINT1_ID 0x142      // shoulder/base motor CAN ID
#define JOINT2_ID 0x141      // elbow/top motor CAN ID

// Geometry
static const float L1 = 0.5f, L2 = 0.5f;

// Joint speed caps
static const float MAX_JOINT_DPS = 90.0f;        // deg/s cap per joint
static const float MAX_JOINT_RAD = MAX_JOINT_DPS * (float)M_PI / 180.0f;

// Hold (0xA4) speed cap
static const uint16_t MAX_DPS_HOLD = 120;        // dps during hold moves

// Send pacing
static const uint32_t MIN_SEND_GAP_US = 20000;   // ~50 Hz per joint
static uint32_t t1_last_send = 0, t2_last_send = 0;

// IK-frame joint state (degrees, CCW+), kept for publishing only
static float th1_prev_deg = 0.0f, th2_prev_deg = 0.0f;

// Timing
unsigned long last_command_time = 0;

// CAN
MCP_CAN CAN0(10);
byte data_kill[8] = {0x81,0,0,0,0,0,0,0};

// ===== Zero offsets (drive frame, CW+ degrees) =====
static float BASE_ZERO_OFFSET_DRIVE_DEG  = 0.0f;
static float ELBOW_ZERO_OFFSET_DRIVE_DEG = 0.0f;

// ===== Per-joint sign mapping (drive vs IK) =====
static const float BASE_SIGN  = -1.0f;   // BOTH inverted vs IK
static const float ELBOW_SIGN = -1.0f;

/* --------- Publishers --------- */
rcl_publisher_t pub_base_deg;    // std_msgs/Float32
rcl_publisher_t pub_elbow_deg;   // std_msgs/Float32
rcl_publisher_t pub_ee_xy;       // geometry_msgs/Point
std_msgs__msg__Float32 out_base_deg;
std_msgs__msg__Float32 out_elbow_deg;
geometry_msgs__msg__Point out_ee;

/* --------- NEW: Telemetry publishers --------- */
rcl_publisher_t pub_base_temp_c;     // std_msgs/Float32
rcl_publisher_t pub_base_curr_a;     // std_msgs/Float32
rcl_publisher_t pub_elbow_temp_c;    // std_msgs/Float32
rcl_publisher_t pub_elbow_curr_a;    // std_msgs/Float32
std_msgs__msg__Float32 out_base_temp, out_base_curr, out_elbow_temp, out_elbow_curr;

/* --------- Subscribers (PC-driven) --------- */
rcl_subscription_t sub_spd1;   // std_msgs/Float32: /cmd_joint1_speed_dps
rcl_subscription_t sub_spd2;   // std_msgs/Float32: /cmd_joint2_speed_dps
rcl_subscription_t sub_hold;   // geometry_msgs/Point: /hold_target_deg (x=θ1, y=θ2)

std_msgs__msg__Float32 in_spd1;
std_msgs__msg__Float32 in_spd2;
geometry_msgs__msg__Point in_hold;

// Cached last commanded speeds (deg/s)
static float v1_dps_cmd = 0.0f, v2_dps_cmd = 0.0f;

// ---------------- Utils ----------------
static inline float clampf(float v, float lo, float hi){ return (v<lo?lo:(v>hi?hi:v)); }

static inline float wrap_deg_pm180(float a){
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

static inline float joint_sign(uint32_t can_id){
  return (can_id == JOINT2_ID) ? ELBOW_SIGN : BASE_SIGN;
}

// IK → Drive (0.01°)
static inline int32_t ikAngle_to_drive_x100(uint32_t can_id, float theta_deg_IK){
  float sign = joint_sign(can_id);
  float offset = (can_id == JOINT2_ID) ? ELBOW_ZERO_OFFSET_DRIVE_DEG : BASE_ZERO_OFFSET_DRIVE_DEG;
  float drive_deg = offset + sign * theta_deg_IK;
  return (int32_t) lroundf(drive_deg * 100.0f);
}

// IK speed (deg/s) → Drive (0.01 dps)
static inline long ikSpeed_to_drive_0p01dps(uint32_t can_id, float speed_dps_IK){
  float sign = joint_sign(can_id);
  float drive_dps = sign * speed_dps_IK;
  return lroundf(drive_dps * 100.0f);
}

// Drive (0.01°) → IK (deg)
static inline float driveAngle_to_ik_deg(uint32_t can_id, int32_t angle_x100){
  float sign   = joint_sign(can_id);
  float drive  = angle_x100 * 0.01f;
  float offset = (can_id == JOINT2_ID) ? ELBOW_ZERO_OFFSET_DRIVE_DEG : BASE_ZERO_OFFSET_DRIVE_DEG;
  return sign * (drive - offset);
}

/* ---------------- CAN: read absolute angle (0.01°) ---------------- */
static bool read_angle_0x92(uint32_t id, int32_t &angle_x100){
  byte tx[8] = {0x92,0,0,0,0,0,0,0};
  CAN0.sendMsgBuf(id, 0, 8, tx);
  unsigned long t0 = millis();
  while (millis() - t0 < 60) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId; byte len; byte rx[8];
      CAN0.readMsgBuf(&rxId, &len, rx);
      if ((uint32_t)rxId == (id + 0x100) && len >= 8 && rx[0] == 0x92) {
        angle_x100 = (int32_t)((uint32_t)rx[4] |
                               ((uint32_t)rx[5]<<8) |
                               ((uint32_t)rx[6]<<16) |
                               ((uint32_t)rx[7]<<24));
        return true;
      }
    }
  }
  return false;
}

/* ---------------- CAN: read Motor Status 2 (0x9C) ----------------
   byte0=0x9C, byte1=int8 temperature (°C), byte2..3=int16 iq (0.01A/LSB)
   byte4..5=int16 speed (dps), byte6..7=int16 angle (deg) */
static bool read_status_0x9C(uint32_t id, int8_t &temp_c, int16_t &iq_0p01A, int16_t &speed_dps, int16_t &angle_deg){
  byte tx[8] = {0x9C,0,0,0,0,0,0,0};
  CAN0.sendMsgBuf(id, 0, 8, tx);
  unsigned long t0 = millis();
  while (millis() - t0 < 60) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId; byte len; byte rx[8];
      CAN0.readMsgBuf(&rxId, &len, rx);
      if ((uint32_t)rxId == (id + 0x100) && len >= 8 && rx[0] == 0x9C) {
        temp_c     = (int8_t)rx[1];
        iq_0p01A   = (int16_t)((uint16_t)rx[2] | ((uint16_t)rx[3]<<8));
        speed_dps  = (int16_t)((uint16_t)rx[4] | ((uint16_t)rx[5]<<8));
        angle_deg  = (int16_t)((uint16_t)rx[6] | ((uint16_t)rx[7]<<8));
        return true;
      }
    }
  }
  return false;
}

/* ---------------- Software zeroing (NO MOTION) ----------------
   Goal: current physical pose => IK: base=0°, elbow=180° */
static bool calibrate_zero_deg(float desired_base_deg = 0.0f, float desired_elbow_deg = 180.0f) {
  int32_t a1_x100 = 0, a2_x100 = 0;
  bool ok1 = false, ok2 = false;
  for (int i=0; i<10 && !ok1; ++i){ ok1 = read_angle_0x92(JOINT1_ID, a1_x100); if(!ok1) delay(10); }
  for (int i=0; i<10 && !ok2; ++i){ ok2 = read_angle_0x92(JOINT2_ID, a2_x100); if(!ok2) delay(10); }
  if (!(ok1 && ok2)) return false;

  float base_drive_deg  = a1_x100 * 0.01f;
  float elbow_drive_deg = a2_x100 * 0.01f;

  BASE_ZERO_OFFSET_DRIVE_DEG  = base_drive_deg  - (desired_base_deg  / BASE_SIGN);
  ELBOW_ZERO_OFFSET_DRIVE_DEG = elbow_drive_deg - (desired_elbow_deg / ELBOW_SIGN);

  BASE_ZERO_OFFSET_DRIVE_DEG  = base_drive_deg  + wrap_deg_pm180(BASE_ZERO_OFFSET_DRIVE_DEG  - base_drive_deg);
  ELBOW_ZERO_OFFSET_DRIVE_DEG = elbow_drive_deg + wrap_deg_pm180(ELBOW_ZERO_OFFSET_DRIVE_DEG - elbow_drive_deg);

  return true;
}

// ===== CAN Command Helpers =====
static inline void send_pos_a4_IK(uint32_t can_id, float theta_deg_IK, uint16_t max_dps){
  int32_t angle_x100 = ikAngle_to_drive_x100(can_id, theta_deg_IK);
  byte d[8];
  d[0]=0xA4; d[1]=0x00;
  d[2]=(uint8_t)(max_dps & 0xFF);
  d[3]=(uint8_t)((max_dps>>8) & 0xFF);
  d[4]=(uint8_t)(angle_x100 & 0xFF);
  d[5]=(uint8_t)((angle_x100>>8) & 0xFF);
  d[6]=(uint8_t)((angle_x100>>16)& 0xFF);
  d[7]=(uint8_t)((angle_x100>>24)& 0xFF);
  CAN0.sendMsgBuf(can_id, 0, 8, d);
}

static inline void send_speed_a2_IK(uint32_t can_id, float speed_dps_IK){
  // clamp to safe range
  float sp = clampf(speed_dps_IK, -MAX_JOINT_DPS, +MAX_JOINT_DPS);
  long scaled = ikSpeed_to_drive_0p01dps(can_id, sp);
  byte dir = 0x00;
  if (scaled < 0) {
    scaled = (0x0000FFFFL + scaled + 1) & 0x0000FFFFL;
    dir = 0xFF;
  }
  byte low  = (byte)(scaled & 0xFF);
  byte high = (byte)((scaled >> 8) & 0xFF);
  byte d[8] = {0xA2,0x00,0x00,0x00, low, high, dir, dir};
  CAN0.sendMsgBuf(can_id, 0, 8, d);
}

/* -------- Subscribers -------- */
void cb_speed1(const void * msgin){
  const std_msgs__msg__Float32 * m = (const std_msgs__msg__Float32 *)msgin;
  v1_dps_cmd = m->data;
  last_command_time = millis();
  uint32_t nowus = micros();
  if ((nowus - t1_last_send) > MIN_SEND_GAP_US){
    send_speed_a2_IK(JOINT1_ID, v1_dps_cmd);
    t1_last_send = nowus;
  }
  digitalWrite(LED_PIN, LOW);
}

void cb_speed2(const void * msgin){
  const std_msgs__msg__Float32 * m = (const std_msgs__msg__Float32 *)msgin;
  v2_dps_cmd = m->data;
  last_command_time = millis();
  uint32_t nowus = micros();
  if ((nowus - t2_last_send) > MIN_SEND_GAP_US){
    send_speed_a2_IK(JOINT2_ID, v2_dps_cmd);
    t2_last_send = nowus;
  }
  digitalWrite(LED_PIN, LOW);
}

void cb_hold(const void * msgin){
  const geometry_msgs__msg__Point * m = (const geometry_msgs__msg__Point *)msgin;
  last_command_time = millis();
  // One-shot A4 to both joints
  send_pos_a4_IK(JOINT1_ID, (float)m->x, MAX_DPS_HOLD);
  send_pos_a4_IK(JOINT2_ID, (float)m->y, MAX_DPS_HOLD);
  digitalWrite(LED_PIN, LOW);
}

/* -------- 50 Hz publisher timer: joints + EE -------- */
void pos_timer_cb(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  // Refresh angles from encoders
  int32_t a1_x100=0, a2_x100=0;
  if (read_angle_0x92(JOINT1_ID, a1_x100)) th1_prev_deg = driveAngle_to_ik_deg(JOINT1_ID, a1_x100);
  if (read_angle_0x92(JOINT2_ID, a2_x100)) th2_prev_deg = driveAngle_to_ik_deg(JOINT2_ID, a2_x100);

  // Publish IK-frame joint angles
  out_base_deg.data  = th1_prev_deg;
  out_elbow_deg.data = th2_prev_deg;
  RCSOFTCHECK(rcl_publish(&pub_base_deg,  &out_base_deg,  NULL));
  RCSOFTCHECK(rcl_publish(&pub_elbow_deg, &out_elbow_deg, NULL));

  // Compute & publish EE x,y from current angles (for your plotter)
  float t1 = th1_prev_deg * (float)M_PI / 180.0f;
  float t2 = th2_prev_deg * (float)M_PI / 180.0f;
  float x1 = L1 * cosf(t1), y1 = L1 * sinf(t1);
  out_ee.x = x1 + L2 * cosf(t1 + t2);
  out_ee.y = y1 + L2 * sinf(t1 + t2);
  out_ee.z = 0.0f;
  RCSOFTCHECK(rcl_publish(&pub_ee_xy, &out_ee, NULL));
}

/* -------- 20 Hz status poll: temps & currents (0x9C) -------- */
void stat_timer_cb(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
  int8_t tC; int16_t iq; int16_t spd; int16_t deg;

  // Base
  if (read_status_0x9C(JOINT1_ID, tC, iq, spd, deg)) {
    out_base_temp.data = (float)tC;
    out_base_curr.data = (float)iq * 0.01f;   // 0.01 A/LSB
    RCSOFTCHECK(rcl_publish(&pub_base_temp_c, &out_base_temp, NULL));
    RCSOFTCHECK(rcl_publish(&pub_base_curr_a, &out_base_curr, NULL));
  }

  // Elbow
  if (read_status_0x9C(JOINT2_ID, tC, iq, spd, deg)) {
    out_elbow_temp.data = (float)tC;
    out_elbow_curr.data = (float)iq * 0.01f;
    RCSOFTCHECK(rcl_publish(&pub_elbow_temp_c, &out_elbow_temp, NULL));
    RCSOFTCHECK(rcl_publish(&pub_elbow_curr_a, &out_elbow_curr, NULL));
  }
}

//-----------------------
//BEGIN DO NOT TOUCH ZONE
//-----------------------

static inline void write_accel(uint32_t can_id, uint8_t index, uint32_t accel_dps2){
  byte d[8]; d[0]=0x43; d[1]=index; d[2]=0; d[3]=0;
  d[4]=(uint8_t)(accel_dps2      & 0xFF);
  d[5]=(uint8_t)((accel_dps2>>8) & 0xFF);
  d[6]=(uint8_t)((accel_dps2>>16)& 0xFF);
  d[7]=(uint8_t)((accel_dps2>>24)& 0xFF);
  CAN0.sendMsgBuf(can_id, 0, 8, d);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // init support & node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // ---- Publishers: joint & EE ----
  RCCHECK(rclc_publisher_init_default(
    &pub_base_deg, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/base_joint_deg"));
  RCCHECK(rclc_publisher_init_default(
    &pub_elbow_deg, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/elbow_joint_deg"));
  RCCHECK(rclc_publisher_init_default(
    &pub_ee_xy, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
    "/ee_xy"));

  // ---- NEW: Telemetry publishers ----
  RCCHECK(rclc_publisher_init_default(
    &pub_base_temp_c, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/base_temp_c"));
  RCCHECK(rclc_publisher_init_default(
    &pub_base_curr_a, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/base_current_a"));
  RCCHECK(rclc_publisher_init_default(
    &pub_elbow_temp_c, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/elbow_temp_c"));
  RCCHECK(rclc_publisher_init_default(
    &pub_elbow_curr_a, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/elbow_current_a"));

  // ---- Subscribers for PC-driven commands ----
  RCCHECK(rclc_subscription_init_default(
    &sub_spd1, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_joint1_speed_dps"));
  RCCHECK(rclc_subscription_init_default(
    &sub_spd2, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_joint2_speed_dps"));
  RCCHECK(rclc_subscription_init_default(
    &sub_hold, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
    "/hold_target_deg"));

  // Timers
  RCCHECK(rclc_timer_init_default(&pos_timer,  &support, RCL_MS_TO_NS(20), pos_timer_cb)); // 50 Hz
  RCCHECK(rclc_timer_init_default(&stat_timer, &support, RCL_MS_TO_NS(50), stat_timer_cb)); // 20 Hz

  // Executor: 3 subs + 2 timers = 5 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_spd1, &in_spd1, &cb_speed1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_spd2, &in_spd2, &cb_speed2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_hold, &in_hold, &cb_hold, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &pos_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &stat_timer));

  // CAN Initialisation
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) { delay(100); }
  CAN0.setMode(MCP_NORMAL);

  // Planner config writes (no motion)
  write_accel(JOINT1_ID, 0x00, 2000); // pos accel (dps/s)
  write_accel(JOINT1_ID, 0x01, 2000); // pos decel
  write_accel(JOINT2_ID, 0x00, 2000);
  write_accel(JOINT2_ID, 0x01, 2000);
  write_accel(JOINT1_ID, 0x02, 2000); // speed accel (dps/s)
  write_accel(JOINT1_ID, 0x03, 2000); // speed decel
  write_accel(JOINT2_ID, 0x02, 2000);
  write_accel(JOINT2_ID, 0x03, 2000);

  // ---- Software zero to IK base=0°, elbow=180° (no motion) ----
  (void)calibrate_zero_deg(0.0f, 180.0f);

  // Initial states (no commands until PC sends)
  th1_prev_deg = 0.0f;
  th2_prev_deg = 180.0f;
  t1_last_send = t2_last_send = micros();
  last_command_time = millis();
}

//---------------------
//END DO NOT TOUCH ZONE
//---------------------

void loop() {
  // Run executor (subs + timers)
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));

  // 2s inactivity -> kill to both joints (safe)
  if (millis() - last_command_time > 2000) {
    CAN0.sendMsgBuf(JOINT1_ID, 0, 8, data_kill);
    CAN0.sendMsgBuf(JOINT2_ID, 0, 8, data_kill);
    digitalWrite(LED_PIN, HIGH);
  }
}

