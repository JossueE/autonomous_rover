#pragma once

#include <modbus/modbus.h>
#include <iostream>
#include <cerrno>
#include <cmath>
#include <utility>
#include <string>
#include <algorithm>
#include <optional>
#include <sstream>
#include <iomanip>


// If conditional compilation directives connected to CMake flags in CMakeLists.txt
#ifdef CMAKE_DEBUG_FLAG
  #define DBG(msg) do { std::cerr << "[Debug] " << msg << "\n"; } while(0)
#else
  #define DBG(msg) do {} while(0)
#endif

#define M_PI 3.14159265358979323846
#define counts_per_rad (4096 / (2 * M_PI)) // Assuming 4096 counts per revolution

// ---------------------------------------- Common ---------------------------------------- 

#define CONTROL_REG 0x200E
#define OPR_MODE  0x200D
#define SAVE_EEPROM 0x2010

// ----------------------------------- For Internal Ramps -----------------------------------

#define L_ACL_TIME  0x2080
#define R_ACL_TIME  0x2081

#define L_DCL_TIME  0x2082
#define R_DCL_TIME  0x2083

// ----------------------------------- For Velocity Mode ----------------------------------- 

#define SET_SPEED_RESOLUTION 0x2022
#define RPM_RES_CERO_POINT_ONE 0x000A
#define RPM_RES_ONE 0x0001

#define SET_RPM  0x2088
#define GET_RPM  0x20AB

// ----------------------- For Position Mode - Absolute and Relative ----------------------- 

#define SET_POS  0x208A  // 0x208A (high) 0x208B (low) = Left, 0x208C (high) 0x208D (low) = Right
#define L_MAX_RPM_POS  0x208E
#define R_MAX_RPM_POS  0x208F

// ----------------------- For Torque Mode - Absolute and Relative ----------------------- 

#define SET_TORQUE 0x2090

// ------------------------------------- Parking Mode --------------------------------------

#define PARKING_MODE_REG 0x200C

// ---------------------------------------- Encoder ---------------------------------------- 
#define ENC_LEFT  0x20A7
#define ENC_RIGHT 0x20A9

// ------------------------ CONTROL GAINS for Velocity control -----------------------------
// LEFT
#define VL_KP 0x203C
#define VL_KI 0x203D
#define VL_KF 0x203E

// RIGHT
#define VR_KP 0x206C
#define VR_KI 0x206D
#define VR_KF 0x206E

// ------------------------ CONTROL GAINS for Position control -----------------------------
// LEFT
#define PL_KP 0x203F
#define PL_KF 0x2040

// RIGHT
#define PR_KP 0x206F
#define PR_KF 0x2070

// ------------------------------------ Troubleshooting ------------------------------------ 

#define FAULTS_LEFT 0x20A5 // <- High 16 bits = Left and Low 16 bits = Right
#define FAULTS_RIGHT 0x20A6 // <- High 16 bits = Left and Low 16 bits = Right
#define READ_ACTUAL_RPM  0x20AB // <- Left and Right motors actual RPM (unit: RPM)
#define READ_ACTUAL_CURRENT 0x20AD // <- Left and Right motors actual Current (unit: A)
#define READ_ACTUAL_SOFTWARE_VERSION 0x20A0 // <- Provided by the manufacturer, can be used to check if the driver is correctly reading registers
#define READ_TEMPERATURE 0x20A4 // <- High 8 bits = Left and Low 8 bits = Right --- Unit = °C

// ------------------------------------ Control Command ------------------------------------ 

#define EMER_STOP  0x05
#define ALRM_CLR 0x06
#define DOWN_TIME 0x07
#define ENABLE 0x08
#define POS_SYNC 0x10
#define POS_L_START 0x11
#define POS_R_START 0x12


// ----------------------- Structs for Position - Velocity Control ------------------------

struct VelocityMotorGains { int16_t kp, ki, kf; };
struct PositionMotorGains { int16_t kp, kf; };

struct VelocityGains { VelocityMotorGains left, right; };
struct PositionGains { PositionMotorGains left, right; };

// ------------------------------------ To Change Mode ------------------------------------

enum class OperationMode {
  VELOCITY,
  RELATIVE_POSITION,
  ABSOLUTE_POSITION,
  TORQUE
};	

enum class ResolutionMode {
	ONE_RPM,
	CERO_POINT_ONE_RPM,
	UNKNOWN
};

// ------------------------------------ To Read Encoder ------------------------------------

struct GetEncoder { 
	int32_t left, right;
	bool status;
};

// ---------------------------- Overload to Print Operation Mode ----------------------------

inline std::ostream& operator<<(std::ostream& os, const OperationMode& m) {
  switch (m) {
    case OperationMode::VELOCITY: return os << "velocity";
    case OperationMode::RELATIVE_POSITION: return os << "relative_position";
    case OperationMode::ABSOLUTE_POSITION: return os << "absolute_position";
    case OperationMode::TORQUE:   return os << "torque";
    default:                      return os << "unknown";
  }
}

// ---------------------------------------- Driver Class ----------------------------------------

class ZLAC8015D
{
public:

	ZLAC8015D(const std::string& port = "/dev/ttyUSB0",
				int baudrate = 115200,
				const OperationMode& mode = OperationMode::VELOCITY	
			);

    ~ZLAC8015D();

// ---------------------------------------- Communication  ----------------------------------------
	/**
   	 * @brief Establishes a Modbus RTU connection to the ZLAC8015D driver.
   	 *
   	 * Creates and connects a libmodbus RTU context using the configured serial
   	 * port and baudrate, sets slave ID = 1, then applies the initial operation
   	 * mode and enables the motor. On failure, cleans up resources and returns false.
   	 *
   	 * @return true if already connected or connection/setup succeeds; false otherwise.
   	*/
	bool connect();

	/**
   	 * @brief Disconnect Modbus RTU connection to the ZLAC8015D driver.
   	 *
   	 * If a connection exists, sends an emergency_stop to the driver, closes the Modbus and frees the context. 
   	 * If no connection exists, simply logs that there was nothing to disconnect.
   	 *
   	 * @return --
   	 * @note After calling disconnect(), the emergency_stop command disable the motor and tu use again the driver you need to call connect() - enable_motor() again.
   	*/
	void disconnect();
	
// ---------------------------------------- General  ----------------------------------------
	/**
   	 * @brief Stop the motor immediately by sending the emergency stop command to the driver.
   	 * @return -1 if no active connection, or if the emergency stop command fails; 0 on success.
   	 * @note After calling disconnect(), the emergency_stop command disable the motor and tu use again the driver you need to call connect() - enable_motor() again.
   	*/
	int emergency_stop();

	/**
   	 * @brief The motor is disable without cutting the power, so it can be enabled calling enable_motor() again.
   	 * @return -1 if no active connection, or if the emergency stop command fails; 0 on success.
   	*/		
	int disable_motor();

	/**
 	 * @brief Enable the motor if it was disabled by disable_motor() or after an emergency stop.
  	 * @return -1 if no active connection, or if the emergency stop command fails; 0 on success.
  	*/
    int enable_motor();

	/**
  	 * @brief Clear the alarm if there is any active. It is useful to validate if the error is presented. 
  	 * @return -1 if no active connection, or if the alarm clear command fails; 0 on success.
  	 * 
  	 * @note You can see if there is an active alarm if the red LEDs on the driver are flashing.
  	*/
	int reset_alarm();

	/**
  	 * @brief It will change to that mode if it's valid, and modify mode_ attribute.
  	 * @return -1 if no active connection, or if the emergency stop command fails; 0 on success. Print the mode to change.
  	*/	
	int change_mode(OperationMode mode);
	
	/**
  	 * @brief Save to EEPROM
  	 * @return -1 if no active connection, or if the emergency stop command fails; 0 on success. Print the mode to change.
  	*/
	int save_to_eeprom();

// ---------------------------------------- Encoder -----------------------------------------
	/**
   	 * @brief Read left and right motor encoder counts.
   	 * @return GetEncoder struct, If read process is incorrect return {-1, -1, false}, else {int32 left, int32 right, true} 
   	*/
	GetEncoder get_encoder_count();

// --------------------------- Control Gains Velocity - Position  ---------------------------
	/**
	 * @brief Read the current control gains for velocity mode.
	 * @return 2 Structs with the control gains for velocity mode for left and right motor on success, std::nullopt on error (not connected, Modbus failure).
	 * @note This is a placeholder implementation. You should read the actual registers for the control gains and return them in a suitable format.
	*/
	std::optional<VelocityGains> get_control_gains_velocity();

	/**
   	 * @brief Read the current control gains for position mode.
   	 * @return 2 Structs with the control gains for position mode for left and right motor on success, std::nullopt on error (not connected, Modbus failure).
   	 * @note This is a placeholder implementation. You should read the actual registers for the control gains and return them in a suitable format.
   	*/
	std::optional<PositionGains> get_control_gains_position();

	/**
   	 * @brief Set the control gains for velocity mode.
   	 * @param gains Structs with the control gains for velocity mode for left and right motor.
   	 * leff.kp, left.ki, left.kf, right.kp, right.ki, right.kf
   	 * @return 0 on success, -1 on error (not connected, Modbus failure).
   	 * @note This is a placeholder implementation. You should write the actual registers for the control gains according to the format you choose for the get_control_gains_velocity() method.
   	*/
	int set_control_gains_velocity(VelocityGains &gains);

	/**
   	 * @brief Set the control gains for position mode.
   	 * @param gains Structs with the control gains for position mode for left and right motor.
   	 * left.kp, left.kf, right.kp, right.kf
   	 * @return 0 on success, -1 on error (not connected, Modbus failure).
   	 * @note This is a placeholder implementation. You should write the actual registers for the control gains according to the format you choose for the get_control_gains_position() method.
   	*/
	int set_control_gains_position(PositionGains &gains);
	
// -------------------------------------  Velocity  Mode -------------------------------------

	/**
   	 * @brief Set the speed resolution for velocity mode.
   	 * @return 0 on success, -1 on error (not connected, Modbus failure).
   	*/
	int set_speed_resolution(const ResolutionMode &Mode);

	/**
   	 * @brief Get the current speed resolution for velocity mode.
   	 * @return "0.1 RPM" or "1 RPM" on success, empty string on error (not connected, Modbus failure).
   	*/
	ResolutionMode get_speed_resolution();

	/**
   	 * @brief Set the target speed for both motors in velocity mode.
   	 * @param L_rpm Target speed for the left motor in RPM (range: -1000.0 to 1000.0).
   	 * @param R_rpm Target speed for the right motor in RPM (range: -1000.0 to 1000.0).
   	 * @return 0 on success, -1 on error (not connected, Modbus failure, invalid mode, or invalid speed resolution).
	 * @note Every param is going to be round to the nearest 0.1 RPM. 
   	*/
	int set_sync_rpm(float L_rpm, float R_rpm, const ResolutionMode &mode); 

	/**
   	 * @brief Read left and right motor actual speed
   	 * @return rpm_left and rmp_right on success (unit: RPM), -1 on error (not connected, Modbus failure).
   	*/
	std::pair<float, float> get_rpm();

// ---------------------------  Absolute and Relative Position  Mode ---------------------------
// Depending on the mode, the position command is going to be executed as absolute or relative, but the method to set the position is the same for both modes.

	/**
   	 * @brief Sets the maximum allowed motor speed (RPM) used in position modes.
   	 *
   	 * Writes the per-motor max speed limits for position control (left/right) to the
   	 * corresponding registers. Input values are clamped to [0, 50] RPM for safety
   	 * before being written. The range is based on Official Documentation.
   	 *
   	 * @param L_rpm Maximum speed for the left motor in position mode (0..50 RPM).
   	 * @param R_rpm Maximum speed for the right motor in position mode (0..50 RPM).
   	 * @return 0 on success, -1 on error (not connected or Modbus write failure).
	*/
	int set_max_speed_position_mode(int16_t L_rpm, int16_t R_rpm);

	/**
	 * @brief Reads the configured maximum speed limits (RPM) used in position modes.
	 *
	 * Retrieves the left and right "max RPM in position mode" values from the driver
	 * registers and returns them as a pair {left, right}. On read failure or if not
	 * connected, returns {-1, -1}.
	 *
	 * @return std::pair<int,int> {left_max_rpm, right_max_rpm} on success, or {-1, -1} on error.
	 */
	std::pair<int, int> get_max_speed_position_mode();

	/**
	 * @brief Sends a synchronized position command (relative or absolute) to both motors.
	 *
	 * Requires the driver to be in a position mode ("relative_position" or
	 * "absolute_position"); if not, it switches to relative position mode for safety.
	 * Reads the current per-motor max speed limits for position mode and clamps them
	 * to 50 RPM if needed. Converts input angles (radians) to encoder counts using
	 * the fixed counts_per_rad factor, writes the 32-bit target positions for left
	 * and right motors (4 registers starting at SET_POS), then triggers a synchronized
	 * position start via CONTROL_REG (POS_SYNC).
	 *
	 * @param L_rad Target left position in radians (interpreted as relative/absolute depending on current mode).
	 * @param R_rad Target right position in radians (interpreted as relative/absolute depending on current mode).
	 * @return 0 on success, -1 on error (not connected, Modbus failure, or partial write).
	*/
	int set_sync_position(float L_rad, float R_rad);

// -------------------------------------- Torque Mode --------------------------------------

	/**
	 * @brief Sends a synchronized current command to both motors.
	 *
	 * Requires the driver to be in "current" mode; if not, it switches to current
	 * mode for safety. Writes the target currents for left and right motors (2 registers
	 * starting at SET_CURRENT), then triggers a synchronized current start via CONTROL_REG (CURR_SYNC).
	 *
	 * @param L_mA Target left current in milliamps.
	 * @param R_mA Target right current in milliamps.
	 * @return 0 on success, -1 on error (not connected, Modbus failure, or partial write).
	*/			
	int set_sync_current(int16_t L_mA, int16_t R_mA);

	/**
   	 * @brief Read left and right motor actual current
   	 * @return current_left and current_right on success (unit: A), -1 on error (not connected, Modbus failure).
   	*/
	std::pair<float, float> get_current();

// -------------------------------------- Parking Mode --------------------------------------

	/**
     * @brief Enable parking mode, limit the current of the motor to 3A and this function to prevent the motor from over temperature problem.
     * @return 0 on success, -1 on error (not connected, Modbus failure).
    */
	int enable_parking_mode();

	/**
   	 * @brief Disable parking mode, allowing the motor to move freely.
   	 * @return 0 on success, -1 on error (not connected, Modbus failure).
   	*/
	int disable_parking_mode();

// -------------------------------- Error and troubleshooting --------------------------------
	
	/**
   	 * @brief Read the error registers for both motors and return their values as hexadecimal strings.
   	 * @return A pair of strings containing the hexadecimal representation of the left and right error registers on success, or error messages on failure (not connected, Modbus failure).
   	*/
	std::pair<std::string, std::string> get_error();

	/**
     * @brief Decode the error register value into human-readable faults and possible causes.
     * @param reg The 16-bit error register value read from the driver for either left or right motor.
     * @param side A string indicating which motor the error register corresponds to ("Left" or "Right") for clearer output.
     * @return 0 on success, -1 on error (not connected, Modbus failure).
    */
	std::string decode_error(uint16_t reg, const char* side);

	/**
     * @brief Read left and right motor temperature in Celsius.
     * @return temp_left and temp_right in °C on success, -1 on error (not connected, Modbus failure).
    */
	std::pair<int, int> get_temperature();

	/**
     * @brief Read the driver software version.
     * @return version number on success, -1 on error (not connected, Modbus failure).
    */
	std::string get_software_version();

// ------------------------------------- Internal Ramps -------------------------------------
	/**
     * @brief Set the deceleration time for velocity mode. 
     * @param decel_time_ms Deceleration time in milliseconds (e.g., 1000 for 1 second).
     * @return 0 on success, -1 on error (not connected, Modbus failure).
     * @note The Default value is 500ms.
	*/
	int set_decel_time(uint16_t decel_time_ms);

	/**
	 * @brief Set the acceleration time for velocity mode. 
     * @param accel_time_ms Acceleration time in milliseconds (e.g., 1000 for 1 second).
     * @return 0 on success, -1 on error (not connected, Modbus failure).
     * @note The Default value is 500ms.
    */
	int set_accel_time(uint16_t accel_time_ms);

private:
	std::string port_;
    int baudrate_;
	OperationMode mode_;
    modbus_t* client_ = nullptr;

	/**
  	* @brief Private method that set the mode of the driver according to the mode_ attribute, it will be called by change_mode() and connect() methods.
  	* @return -1 if no active connection, or if the emergency stop command fails; 0 on success. Print the mode to change.
  	*/
	int set_mode();

	/** 
   	 * @brief Convert a 16-bit unsigned integer to a hexadecimal string in the format "0xABCD".
     * @param v The 16-bit unsigned integer to convert.
     * @return A string representing the hexadecimal value of v, formatted as "0xABCD".
    */
	std::string hex16(uint16_t v); 
};

