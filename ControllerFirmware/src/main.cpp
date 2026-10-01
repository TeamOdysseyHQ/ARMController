#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AS5600.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32_multi_array.h>

/**
 * ============================================================================
 *   ESP32 micro-ROS 7-Channel AS5600 Encoder Publisher (TCA9548A MUX)
 * ============================================================================
 *
 * MAPPING:
 * - MUX Channel 0 <--> "base"
 * - MUX Channel 1 <--> "joint_5"
 * - MUX Channel 2 <--> "joint_3"
 * - MUX Channel 3 <--> "joint_4"
 * - MUX Channel 4 <--> "joint_2"
 * - MUX Channel 5 <--> "trigger"
 * - MUX Channel 6 <--> "joint_1"
 *
 * ROS 2 TOPICS PUBLISHED:
 * 1. /joint_states     (sensor_msgs/msg/JointState)
 *    - name[]:     ["base", "joint_5", "joint_3", "joint_4", "joint_2", "trigger", "joint_1"]
 *    - position[]: 7 joint angles in normal angles [0.0 to 180.0]
 * 2. /arm/joint_raw    (std_msgs/msg/Int32MultiArray)
 *    - data[]:     7 raw 12-bit encoder counts [0 to 4095]
 *
 * MODES:
 * - USE_MICROROS = 1: Uses Serial for micro-ROS agent communication.
 *   LED blinks while waiting for agent; turns solid ON when connected.
 * - USE_MICROROS = 0: Standalone Serial Monitor debug table with "Change" column.
 */

// ----------------------------------------------------------------------------
// CONFIGURATION & CONSTANTS
// ----------------------------------------------------------------------------

#define USE_MICROROS 1 // 1 = micro-ROS mode (Serial), 0 = Serial Monitor debug table

#define LED_PIN 2
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define TCA9548A_ADDR 0x70

#define NUM_CHANNELS 7
#define ROS_PUBLISH_FREQ_HZ 50
#define PUBLISH_INTERVAL_MS (1000 / ROS_PUBLISH_FREQ_HZ)
#define STANDALONE_INTERVAL_MS 500

const char *const JOINT_NAMES[NUM_CHANNELS] = {
    "base", "joint_5", "joint_3", "joint_4", "joint_2", "trigger", "joint_1"};

// ----------------------------------------------------------------------------
// DATA STRUCTURES & GLOBAL STATE
// ----------------------------------------------------------------------------

Adafruit_AS5600 as5600;

struct EncoderData {
  bool connected;
  uint16_t raw_angle;
  float degrees;       // 0 to 360 degrees
  float angle_180;     // Normal angle 0 to 180 degrees
  float radians;
  bool magnet_ok;
  bool has_prev;
  uint16_t prev_raw_angle;
  int16_t raw_change;
  bool valid_change;
};

EncoderData channel_data[NUM_CHANNELS];

// micro-ROS State Machine
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState agent_state = WAITING_AGENT;

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// Publishers
rcl_publisher_t joint_state_publisher;
rcl_publisher_t raw_array_publisher;

// Messages
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Int32MultiArray raw_array_msg;

// Static allocation buffers for micro-ROS messages
rosidl_runtime_c__String name_array[NUM_CHANNELS];
char name_buffers[NUM_CHANNELS][20];
double position_array[NUM_CHANNELS];
int32_t raw_array_data[NUM_CHANNELS];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

// ----------------------------------------------------------------------------
// HARDWARE / SENSOR FUNCTIONS
// ----------------------------------------------------------------------------

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  if (channel < NUM_CHANNELS) {
    Wire.write(1 << channel);
  } else {
    Wire.write(0x00); // Disable all
  }
  Wire.endTransmission();
}

void readActiveChannel(uint8_t ch) {
  if (as5600.begin(AS5600_DEFAULT_ADDR, &Wire)) {
    uint16_t current_raw = as5600.getRawAngle();
    channel_data[ch].connected = true;
    channel_data[ch].raw_angle = current_raw;
    channel_data[ch].degrees = (current_raw * 360.0f) / 4096.0f;
    channel_data[ch].angle_180 = (current_raw * 180.0f) / 4096.0f;
    channel_data[ch].radians = (current_raw * 2.0f * (float)PI) / 4096.0f;
    channel_data[ch].magnet_ok = as5600.isMagnetDetected();

    if (channel_data[ch].has_prev) {
      int16_t diff = (int16_t)current_raw - (int16_t)channel_data[ch].prev_raw_angle;
      // Handle 12-bit circular wrap-around (0 - 4095)
      if (diff > 2048) {
        diff -= 4096;
      } else if (diff < -2048) {
        diff += 4096;
      }
      channel_data[ch].raw_change = diff;
      channel_data[ch].valid_change = true;
    } else {
      channel_data[ch].raw_change = 0;
      channel_data[ch].valid_change = false;
      channel_data[ch].has_prev = true;
    }
    channel_data[ch].prev_raw_angle = current_raw;
  } else {
    channel_data[ch].connected = false;
    channel_data[ch].magnet_ok = false;
    channel_data[ch].has_prev = false;
    channel_data[ch].valid_change = false;
    channel_data[ch].raw_change = 0;
  }
}

void readAllEncoders() {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    selectMuxChannel(ch);
    delay(2); // Short settling delay for I2C switching
    readActiveChannel(ch);
  }
  selectMuxChannel(255); // Disable all channels after read
}

// ----------------------------------------------------------------------------
// MICRO-ROS FUNCTIONS
// ----------------------------------------------------------------------------

void init_messages() {
  // 1. JointState message
  joint_state_msg.name.capacity = NUM_CHANNELS;
  joint_state_msg.name.size = NUM_CHANNELS;
  joint_state_msg.name.data = name_array;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    strncpy(name_buffers[i], JOINT_NAMES[i], sizeof(name_buffers[i]) - 1);
    name_buffers[i][sizeof(name_buffers[i]) - 1] = '\0';
    name_array[i].data = name_buffers[i];
    name_array[i].size = strlen(name_buffers[i]);
    name_array[i].capacity = sizeof(name_buffers[i]);
  }

  joint_state_msg.position.capacity = NUM_CHANNELS;
  joint_state_msg.position.size = NUM_CHANNELS;
  joint_state_msg.position.data = position_array;

  joint_state_msg.velocity.capacity = 0;
  joint_state_msg.velocity.size = 0;
  joint_state_msg.velocity.data = NULL;

  joint_state_msg.effort.capacity = 0;
  joint_state_msg.effort.size = 0;
  joint_state_msg.effort.data = NULL;

  // 2. Int32MultiArray message
  raw_array_msg.layout.dim.capacity = 0;
  raw_array_msg.layout.dim.size = 0;
  raw_array_msg.layout.dim.data = NULL;
  raw_array_msg.layout.data_offset = 0;

  raw_array_msg.data.capacity = NUM_CHANNELS;
  raw_array_msg.data.size = NUM_CHANNELS;
  raw_array_msg.data.data = raw_array_data;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm_encoder_node", "", &support));

  // Publisher for sensor_msgs/msg/JointState
  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  // Publisher for std_msgs/msg/Int32MultiArray
  RCCHECK(rclc_publisher_init_default(
    &raw_array_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/arm/joint_raw"));

  // Synchronize time with the ROS 2 agent
  rmw_uros_sync_session(1000);

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&joint_state_publisher, &node));
  RCSOFTCHECK(rcl_publisher_fini(&raw_array_publisher, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void publish_data() {
  int64_t time_ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
  joint_state_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    // Normal angles in 0 to 180 degrees for joint_state
    position_array[ch] = (double)channel_data[ch].angle_180;
    raw_array_data[ch] = (int32_t)channel_data[ch].raw_angle;
  }

  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
  RCSOFTCHECK(rcl_publish(&raw_array_publisher, &raw_array_msg, NULL));
}

// ----------------------------------------------------------------------------
// STANDALONE SERIAL MONITOR TABLE (ACTIVE WHEN USE_MICROROS == 0)
// ----------------------------------------------------------------------------

#if !USE_MICROROS
void print_serial_table() {
  int connected_count = 0;
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    if (channel_data[ch].connected) connected_count++;
  }

  Serial.println();
  Serial.printf("ENCODER STATUS @ %lu ms  (%d/%d connected)\n",
                millis(), connected_count, NUM_CHANNELS);
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println(" Joint      | Values                          | Change      | Channel | SD/SC  ");
  Serial.println("--------------------------------------------------------------------------------");

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    char change_buf[16];
    if (channel_data[ch].connected && channel_data[ch].valid_change) {
      if (channel_data[ch].raw_change != 0) {
        snprintf(change_buf, sizeof(change_buf), "YES (%+d)", channel_data[ch].raw_change);
      } else {
        snprintf(change_buf, sizeof(change_buf), "NO");
      }
    } else {
      snprintf(change_buf, sizeof(change_buf), "---");
    }

    if (channel_data[ch].connected) {
      Serial.printf(" %-10s | Raw: %4u  Deg: %6.1f  Mag: %-3s | %-11s | CH%-5u | SD%u/SC%u\n",
                    JOINT_NAMES[ch], channel_data[ch].raw_angle,
                    channel_data[ch].degrees,
                    channel_data[ch].magnet_ok ? "OK" : "NO",
                    change_buf, ch, ch, ch);
    } else {
      Serial.printf(" %-10s | NOT CONNECTED                   | %-11s | CH%-5u | SD%u/SC%u\n",
                    JOINT_NAMES[ch], change_buf, ch, ch, ch);
    }
  }
  Serial.println("--------------------------------------------------------------------------------");
}
#endif

// ----------------------------------------------------------------------------
// MAIN SETUP AND LOOP
// ----------------------------------------------------------------------------

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize I2C bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400 kHz fast I2C
  delay(100);

#if USE_MICROROS
  // Configure micro-ROS custom Serial transport
  set_microros_transports();
  init_messages();
#else
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[INFO] Starting in Standalone Debug Table Mode");
#endif
}

void loop() {
#if USE_MICROROS
  static unsigned long last_ping_time = 0;
  static unsigned long last_publish_time = 0;
  unsigned long now = millis();

  switch (agent_state) {
    case WAITING_AGENT:
      if (now - last_ping_time >= 500) {
        last_ping_time = now;
        // Non-blocking ping: 1 attempt, 100 ms timeout
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          agent_state = AGENT_AVAILABLE;
        } else {
          // Blink LED while waiting for agent
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        agent_state = AGENT_CONNECTED;
        digitalWrite(LED_PIN, HIGH); // Solid ON when connected to agent
      } else {
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (now - last_publish_time >= PUBLISH_INTERVAL_MS) {
        last_publish_time = now;

        // Verify agent connection periodically
        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) {
          readAllEncoders();
          publish_data();
        } else {
          agent_state = AGENT_DISCONNECTED;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      digitalWrite(LED_PIN, LOW);
      agent_state = WAITING_AGENT;
      break;
  }

#else
  // Standalone Serial Debug Table Mode
  static unsigned long last_table_time = 0;
  unsigned long now = millis();

  if (now - last_table_time >= STANDALONE_INTERVAL_MS) {
    last_table_time = now;
    readAllEncoders();
    print_serial_table();
  }
#endif
}
