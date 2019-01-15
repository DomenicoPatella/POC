#include <GCS_MAVLink.h>
#include <mavlink_msg_sys_status.h>
#define bRate 57600



void setup()
{ 
  Serial.begin(bRate);
 
}


void loop() {

  command_heartbeat();
  send_sys_status();

}

void command_heartbeat() {

  //< ID 1 for this system
  int sysid = 1;                   
  //< The component sending the message.
  int compid = MAV_COMP_ID_SYSTEM_CONTROL;    
  
  // Define the system type, in this case ground control station
  uint8_t system_type =MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
  
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  //delay(1);
  Serial.write(buf, len);
}


void send_sys_status() {

//< ID 1 for this system
  int system_id = 1;                   
  //< The component sending the message.
  int component_id = MAV_COMP_ID_SYSTEM_CONTROL;    
  
    // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

/*
typedef struct __mavlink_sys_status_t
{
 uint32_t onboard_control_sensors_present; ///< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 uint32_t onboard_control_sensors_enabled; ///< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 uint32_t onboard_control_sensors_health; ///< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 uint16_t load; ///< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 uint16_t voltage_battery; ///< Battery voltage, in millivolts (1 = 1 millivolt)
 int16_t current_battery; ///< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 uint16_t drop_rate_comm; ///< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 uint16_t errors_comm; ///< Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 uint16_t errors_count1; ///< Autopilot-specific errors
 uint16_t errors_count2; ///< Autopilot-specific errors
 uint16_t errors_count3; ///< Autopilot-specific errors
 uint16_t errors_count4; ///< Autopilot-specific errors
 int8_t battery_remaining; ///< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
} mavlink_sys_status_t;
*/

  
  /**
 * @brief Encode a sys_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_status C-struct to read the message contents from
 */
/*static inline uint16_t mavlink_msg_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status)
{
	return mavlink_msg_sys_status_pack(system_id, component_id, msg, 
             sys_status->onboard_control_sensors_present, 
             sys_status->onboard_control_sensors_enabled, 
             sys_status->onboard_control_sensors_health, 
             sys_status->load, sys_status->voltage_battery, 
             sys_status->current_battery, 
             sys_status->battery_remaining, 
             sys_status->drop_rate_comm, 
             sys_status->errors_comm, 
             sys_status->errors_count1, 
             sys_status->errors_count2, 
             sys_status->errors_count3, 
             sys_status->errors_count4);
//}*/
  
  
 uint32_t onboard_control_sensors_present; ///< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 uint32_t onboard_control_sensors_enabled; ///< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 uint32_t onboard_control_sensors_health; ///< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
 uint16_t load; ///< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 uint16_t voltage_battery; ///< Battery voltage, in millivolts (1 = 1 millivolt)
 int16_t current_battery; ///< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 uint16_t drop_rate_comm; ///< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 uint16_t errors_comm; ///< Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 uint16_t errors_count1; ///< Autopilot-specific errors
 uint16_t errors_count2; ///< Autopilot-specific errors
 uint16_t errors_count3; ///< Autopilot-specific errors
 uint16_t errors_count4; ///< Autopilot-specific errors
 int8_t battery_remaining; ///< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
  
  
  
  
  mavlink_msg_sys_status_pack(system_id, component_id, &msg, 
             onboard_control_sensors_present, 
             onboard_control_sensors_enabled, 
             onboard_control_sensors_health,
             load, 
             voltage_battery, 
             current_battery, 
             battery_remaining, 
             drop_rate_comm, 
             errors_comm, 
             errors_count1, 
             errors_count2, 
             errors_count3, 
             errors_count4);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}




