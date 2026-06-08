/**
 * @file zlac706.h
 * @author Andrés Castro (andres.castro@octopy.com)
 * @brief
 * @version 1.0
 * @date 2022-06-28
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef ZLAC706_H
#define ZLAC706_H

#include "AsyncSerial.h"
// xd
#include <array>
#include <cstring>
#include <iostream>
#include <math.h>
#include <memory>
#include <string>
#include <vector>

#define Alarm_status {0x60, 0x00, 0x00, 0x60} // Baterry failure status

#define M_PI 3.14159265358979323846

/**
 * @brief Control for driver zlac706
 *
 */

// ----------------------- Structs for Position - Velocity Control
// ------------------------

struct VelocityMotorGains {
  int16_t kp, ki, kf;
};
struct VelocityGains {
  VelocityMotorGains left, right;
};

// ----------------------- Structs for Position - Velocity Control
// ------------------------

class ZLAC706 {
public:
  /**
   * @brief Receive serial data
   *
   * @param data
   * @param len
   */
  void received(const char *data, unsigned int len);

  /**
   * @brief Set the Speed Mode
   *
   */
  void setSpeedMode();

  /**
   * @brief Set acceleration and decacceleration values
   *
   * @param acc Acceleration
   * @param decc Decacceleration
   */
  void configAcc(const uint8_t acc, const uint8_t decc);

  /**
   * @brief Set the Speed on RPM
   *
   * @param speed speed of motor on RPM
   */
  void setSpeed(int speed);

  /**
   * @brief Start run motor.
   *
   */
  void start();

  /**
   * @brief Keep running the motor.
   */
  void keepRunning();

  /**
   * @brief Stop run motor.
   */
  void stop();

  /**
   * @brief Set PID gains.
   * @param gain 0=Kp, 1=ki and 2=kd
   * @param value value of gain
   */
  void setGain(const uint8_t gain, const int value);

  /**
   * @brief Get battery Volts, value is save on response
   */
  void getVolts();

  /**
   * @brief Get driver Amps, value is save on response
   */
  void getAmps();

  /**
   * @brief Get motor Speed, value is save on response
   */
  void getSpeed();

  /**
   * @brief Get encoder ticks, value is saved on ticks.
   */
  void getTicks();

  /**
   * @brief Clear driver Alarms
   */
  void clearAlarms();

  /**
   * @brief Get can id,kp,ki and kd.
   */
  void getParams();

  /**
   * @brief Alarm status
   */
  void EstAlarm();

  /**
   * @brief Get zltech devices, value is saved o 'devices' vector
   *
   * @param devices List of available devices
   */
  void getDevices(std::vector<std::string> &devices);

  /**
   * @brief Close serial comunication
   */
  void close();

  std::string port = "";   ///< Serial port
  int baudrate = 0;        ///< Baudrate
  int16_t can_id = 666;    ///< Driver can id
  int16_t kp = 0;          ///< Proportional Gain
  int16_t ki = 0;          ///< Integrate Gain
  int16_t kd = 0;          ///< Derivative Gain
  int16_t speed = 0;       ///< Speed
  int16_t response = 0;    ///< Driver response
  int16_t voltage = 0;     ///< Motor Voltage
  int32_t ticks = 0;       ///< Motor Ticks
  std::string status = ""; ///< Motor Status

  std::vector<unsigned char> str_msg = std::vector<unsigned char>{
      startCommad, 0, 1, 1}; ///< Motor start, Write motor enable
  std::vector<unsigned char> stp_msg = std::vector<unsigned char>{
      startCommad, 0, 0, 0}; ///< Motor stop, Write motor disable
  std::vector<unsigned char> spm_msg = std::vector<unsigned char>{
      speedMCommand, 0, 0xC4, 0xC6}; ///< Speed mode selection, Control mode
                                     ///< given command source selection
  std::vector<unsigned char> alm_msg = std::vector<unsigned char>{
      alarmCommand, 0, 0, alarmCommand}; ///< Clear fault
  std::vector<unsigned char> cfg_msg =
      std::vector<unsigned char>{configCommand, 0, 0, 0}; ///< Speed mode
  std::vector<unsigned char> sSpeed_msg = std::vector<unsigned char>{
      setSpeedCommand, 0, 0, 0}; ///< Speed debug mode
  std::vector<unsigned char> kpG_msg = std::vector<unsigned char>{
      setKpCommand, 0, 0, 0}; ///<  Speed proportional gain
  std::vector<unsigned char> kiG_msg = std::vector<unsigned char>{
      setKiCommand, 0, 0, 0}; ///<  Speed integral gain
  std::vector<unsigned char> kdG_msg = std::vector<unsigned char>{
      setKdCommand, 0, 0, 0}; ///<  Speed differential gain

  std::vector<unsigned char> gVolt_msg = std::vector<unsigned char>{
      getVoltCommand, 0, 0, getVoltCommand}; ///< Bus voltage
  std::vector<unsigned char> gAmps_msg = std::vector<unsigned char>{
      getAmpsCommnad, 0, 0, getAmpsCommnad}; ///< Output current
  std::vector<unsigned char> gSpeed_msg = std::vector<unsigned char>{
      getSpeedCommand, 0, 0, getSpeedCommand}; ///< Motor speed
  std::vector<unsigned char> gTicks_msg = std::vector<unsigned char>{
      getTicksCommand, 0x00, 0x00, getTicksCommand}; ///< Position feedback
  std::vector<unsigned char> gParams_msg =
      std::vector<unsigned char>{readParamsCommand, 0x00, 0x14, 0x5D};

  std::vector<unsigned char> alv_msg =
      std::vector<unsigned char>{aliveCommand, 0, aliveCommand};

  std::vector<unsigned char> alm_status =
      std::vector<unsigned char>{alarmstatus, 0x00, 0x00, alarmstatus};

private:
  /** Command headers.
   *  The headers for commands driver
   */
  enum command : unsigned char {
    startCommad = 0x00,       /**< enum start command */
    speedMCommand = 0x02,     /**< enum speed commnad */
    alarmCommand = 0x4A,      /**< enum alarm commnad */
    configCommand = 0x0A,     /**< enum config commnad */
    setSpeedCommand = 0x06,   /**< enum set speed commnad */
    setKpCommand = 0x40,      /**< enum set proportional gain commnad */
    setKiCommand = 0x41,      /**< enum set integral gain commnad */
    setKdCommand = 0x42,      /**< enum set derivative commnad */
    getKpCommand = 0xC0,      /**< enum get proportional gain commnad */
    getKiCommand = 0xC1,      /**< enum get integral gain commnad */
    getKdCommand = 0xC2,      /**< enum get derivative commnad */
    getVoltCommand = 0x61,    /**< enum get voltage commnad */
    getAmpsCommnad = 0x62,    /**< enum get current commnad */
    getSpeedCommand = 0x63,   /**< enum get speed commnad */
    getTicksCommand = 0x65,   /**< enum get ticks commnad */
    aliveCommand = 0x80,      /**< enum alive  commnad */
    readParamsCommand = 0x49, /**< enum read params commnad */
    getCanIDCommand = 0x8D,   /**< enum get can id commnad */
    voltageCommand = 0xE1,    /**< enum voltage commnad */
    speedCommand = 0xE4,      /**< enum speed commnad */
    postionHCommand = 0XE8,   /**< enum get position high commnad */
    postionLCommand = 0xE9,   /**< enum get position low commnad */
    alarmstatus = 0x60,       /**< enum alarma status commnad */
    readParamsFlag = 0xFA     /**< enum read params flags commnad */
  };

  /**
   * @brief Serial read
   *
   * @param v Vector char with data
   */
  void readData(const std::vector<char> &v);

  /**
   * @brief Get alarm status from data and save on status
   *
   * @param data
   */
  void Alarmstatus(const char *data);

  /**
   * @brief Call a method from check command hexCommand from vector v
   *
   * @param v vector of char with data to read
   * @param hexCommand commnad to compare data
   */
  void checkCommnand(const std::vector<char> &v, unsigned int hexCommand);

  /**
   * @brief Get the Response object
   *
   * @param v vector of char with data to read
   * @param hexCommand commnad to compare data
   */
  void getResponse(const std::vector<char> &v, unsigned int hexCommand);

  /**
   * @brief read params from inpunt
   *
   * @param v vector of char with data to read
   */
  void readParams(const std::vector<char> &v);

  /**
   * @brief Execute bash command
   *
   * @param cmd
   * @return std::string
   */
  std::string exec(const char *cmd);

  volatile bool isCommand = false;  ///< Is true if data contains a command
  bool isCommandResponse = false;   ///< Is true is data contains a response
  bool isE8 = false;                ///< Is true if data contains E8 header
  bool isMessageE8 = false;         ///< Is true if data contains E8 message
  bool isE9 = false;                ///< Is true if data contains E9 header
  bool isMessageE9 = false;         ///< Is true if data contains E9 message
  uint8_t contBytes = 0;            ///< Count bytes in data message
  uint8_t commandCount = 0;         ///< Count bytes in command
  uint8_t contE8 = 0;               ///< Count E8 headers in vector data
  uint8_t contE9 = 0;               ///< Count E9 headers in vector data
  unsigned char commandSent = 0xFF; ///< Command 0xFF
  std::vector<unsigned char> commandResponse =
      std::vector<unsigned char>(5, (unsigned char)0); ///< Message response
  std::vector<unsigned char> ticks_v = std::vector<unsigned char>(
      4, (unsigned char)0); ///< Message response for ticks

  std::vector<unsigned char> messageE8 = std::vector<unsigned char>(
      2, (unsigned char)0); ///< Message response for E8
  std::vector<unsigned char> messageE9 = std::vector<unsigned char>(
      2, (unsigned char)0); ///< Message response for E9
};

#endif
