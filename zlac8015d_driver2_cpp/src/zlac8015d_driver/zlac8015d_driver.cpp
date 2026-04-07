#include "zlac8015d_driver.h"

ZLAC8015D::ZLAC8015D(const std::string& port, int baudrate, const OperationMode& mode): port_(port), baudrate_(baudrate), mode_(mode){
  std::cout << "ZLAC8015D driver created with port: " << port_ << ", baudrate: " << baudrate_ << ", mode: " << mode_ << std::endl;
}

ZLAC8015D::~ZLAC8015D(){
  disconnect();
}

bool ZLAC8015D::connect(){
 
  if (client_) {
    DBG("You have a previous connection, we recommended to reconnect, disconnecting first.");
    return true;
  }
  DBG("Trying port: " << port_ << " baudrate: " << baudrate_);
  client_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
  if (!client_) {
    std::cerr << "Error creating context Modbus RTU (modbus_new_rtu fail)." << std::endl;
    return false;
  }
  modbus_set_slave(client_, 1);
  if (modbus_connect(client_) == -1) {
    std::cerr << "modbus_connect failed: " << modbus_strerror(errno) << "\n";
    modbus_free(client_);
    client_ = nullptr;
    return false;
  }

  // Initial mode
  if (set_mode() == -1 || enable_motor() == -1) {
    disconnect();
    return false;
  }
  
  std::cout << "Connecting to ZLAC8015D by: port " << port_.c_str() << ", baudrate "<< baudrate_ << std::endl;
  return true;
}

void ZLAC8015D::disconnect(){

  if (client_) {
    emergency_stop();
    modbus_close(client_);
    modbus_free(client_);
    client_ = nullptr;
    DBG("Successfully disconnected\n");
  } else {
    DBG("No active connection to disconnect\n");
  }
}

int ZLAC8015D::emergency_stop(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, EMER_STOP);
};

int ZLAC8015D::disable_motor(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, DOWN_TIME);
}

int ZLAC8015D::enable_motor(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, ENABLE);
}

int ZLAC8015D::reset_alarm(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, ALRM_CLR);
}

int ZLAC8015D::change_mode(OperationMode mode) {

  if (!client_) return -1;
  mode_ = mode; // Modify The Argument of the class
  return set_mode();
}

int ZLAC8015D::set_mode() {

  if (!client_) return -1;
  if (mode_ == OperationMode::VELOCITY) {
    DBG("Successfully setted to velocity mode");
    return modbus_write_register(client_, OPR_MODE, 0X0003);
  }
  else if (mode_ == OperationMode::RELATIVE_POSITION) {
    DBG("Successfully setted to relative_position mode");
    return modbus_write_register(client_, OPR_MODE, 0X0001);
  }
  else if (mode_ == OperationMode::ABSOLUTE_POSITION) {
    DBG("Successfully setted to absolute_position mode");
    return modbus_write_register(client_, OPR_MODE, 0X0002);
  }
  else if (mode_ == OperationMode::TORQUE) {
    DBG("Successfully setted to torque mode");
    return modbus_write_register(client_, OPR_MODE, 0X0004);
  }
  return -1;
}


GetEncoder ZLAC8015D::get_encoder_count() {
  GetEncoder enc{-1, -1, false};

  if (!client_) {
    return enc;
  } 

  uint16_t L[2] = {0, 0};
  uint16_t R[2] = {0, 0};

  int rc_l = modbus_read_registers(client_, ENC_LEFT, 2, L);
  int rc_r = modbus_read_registers(client_, ENC_RIGHT, 2, R);

  if (rc_l == -1 || rc_r == -1) {
    std::cerr << "get_encoder_count failed: " << modbus_strerror(errno) << std::endl;
    return enc;
  }

  uint32_t left_u  = (static_cast<uint32_t>(L[0]) << 16) | static_cast<uint32_t>(L[1]);
  uint32_t right_u = (static_cast<uint32_t>(R[0]) << 16) | static_cast<uint32_t>(R[1]);

  enc.left = static_cast<int32_t>(left_u);
  enc.right = static_cast<int32_t>(right_u);
  enc.status = true;

  DBG("Encoder counts: Left: " << enc.left << " Right: " << enc.right);

  return enc;
}

std::optional<VelocityGains> ZLAC8015D::get_control_gains_velocity(){

  if (!client_) return std::nullopt;

  uint16_t kp_u_l {0}, ki_u_l {0}, kf_u_l {0};
  uint16_t kp_u_r {0}, ki_u_r {0}, kf_u_r {0};

  // Left
  if(modbus_read_registers(client_, VL_KP, 1, &kp_u_l) != 1 ||
     modbus_read_registers(client_, VL_KI, 1, &ki_u_l) != 1 ||
     modbus_read_registers(client_, VL_KF, 1, &kf_u_l) != 1) {
    std::cerr << "Failed to read left velocity gains for left_motor: " << modbus_strerror(errno) << std::endl;
    return std::nullopt;
  }

  // Right
  if(modbus_read_registers(client_, VR_KP, 1, &kp_u_r) != 1 ||
     modbus_read_registers(client_, VR_KI, 1, &ki_u_r) != 1 ||
     modbus_read_registers(client_, VR_KF, 1, &kf_u_r) != 1) {
    std::cerr << "Failed to read left velocity gains for right_motor: " << modbus_strerror(errno) << std::endl;
    return std::nullopt;
  } 

  VelocityGains gains;
  gains.left.kp = static_cast<int16_t>(kp_u_l);
  gains.left.ki = static_cast<int16_t>(ki_u_l);
  gains.left.kf = static_cast<int16_t>(kf_u_l);  
  
  gains.right.kp = static_cast<int16_t>(kp_u_r);
  gains.right.ki = static_cast<int16_t>(ki_u_r);
  gains.right.kf = static_cast<int16_t>(kf_u_r);  

  DBG("Velocity Gains - Left: Kp=" << gains.left.kp << " Ki=" << gains.left.ki << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Ki=" << gains.right.ki << " Kf=" << gains.right.kf);

  return gains;
}


std::optional<PositionGains> ZLAC8015D::get_control_gains_position(){

  if (!client_) return std::nullopt;
  uint16_t kp_u_l {0}, kf_u_l {0};
  uint16_t kp_u_r {0}, kf_u_r {0};

  // Left
  if(modbus_read_registers(client_, PL_KP, 1, &kp_u_l) != 1 ||
     modbus_read_registers(client_, PL_KF, 1, &kf_u_l) != 1) {
    std::cerr << "Failed to read left position gains for left_motor: " << modbus_strerror(errno) << std::endl;  
    return std::nullopt;
  }

  // Right
  if(modbus_read_registers(client_, PR_KP, 1, &kp_u_r) != 1 ||
     modbus_read_registers(client_, PR_KF, 1, &kf_u_r) != 1) {
    std::cerr << "Failed to read left position gains for right_motor: " << modbus_strerror(errno) << std::endl;
    return std::nullopt;
  }

  PositionGains gains;
  gains.left.kp = static_cast<int16_t>(kp_u_l);
  gains.left.kf = static_cast<int16_t>(kf_u_l);

  gains.right.kp = static_cast<int16_t>(kp_u_r);
  gains.right.kf = static_cast<int16_t>(kf_u_r);

  DBG("Position Gains - Left: Kp=" << gains.left.kp << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Kf=" << gains.right.kf);

  return gains;
}

int ZLAC8015D::set_control_gains_velocity(VelocityGains &gains){

  if (!client_) return -1;
  uint16_t regs[6] = {
    static_cast<uint16_t>(gains.left.kp),
    static_cast<uint16_t>(gains.left.ki),
    static_cast<uint16_t>(gains.left.kf),
    static_cast<uint16_t>(gains.right.kp),
    static_cast<uint16_t>(gains.right.ki),
    static_cast<uint16_t>(gains.right.kf)
  };

  if(modbus_write_registers(client_, VL_KP, 1, &regs[0]) == -1 ||
     modbus_write_registers(client_, VL_KI, 1, &regs[1]) == -1 ||
     modbus_write_registers(client_, VL_KF, 1, &regs[2]) == -1 ){
    std::cerr << "set_control_gains_velocity for left side failed: " << modbus_strerror(errno) << std::endl;
    return -1;
    }
  
  if(modbus_write_registers(client_, VR_KP, 1, &regs[3]) == -1 ||
     modbus_write_registers(client_, VR_KI, 1, &regs[4]) == -1 ||
     modbus_write_registers(client_, VR_KF, 1, &regs[5]) == -1 ){
    std::cerr << "set_control_gains_velocity for right side failed: " << modbus_strerror(errno) << std::endl;  
    return -1;
    }
  
  DBG("Successfully set velocity gains: " << "Left: Kp=" << gains.left.kp << " Ki=" << gains.left.ki << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Ki=" << gains.right.ki << " Kf=" << gains.right.kf);
  return 0;
}

int ZLAC8015D::set_control_gains_position(PositionGains &gains){

  if (!client_) return -1;
  uint16_t regs[4] = {
    static_cast<uint16_t>(gains.left.kp),
    static_cast<uint16_t>(gains.left.kf),
    static_cast<uint16_t>(gains.right.kp),
    static_cast<uint16_t>(gains.right.kf)
  };

  if(modbus_write_registers(client_, PL_KP, 1, &regs[0]) == -1 ||
     modbus_write_registers(client_, PL_KF, 1, &regs[1]) == -1 ){
    std::cerr << "set_control_gains_position for left side failed: " << modbus_strerror(errno) << std::endl;
    return -1;
    }
  
  if(modbus_write_registers(client_, PR_KP, 1, &regs[2]) == -1 ||
     modbus_write_registers(client_, PR_KF, 1, &regs[3]) == -1 ){
    std::cerr << "set_control_gains_position for right side failed: " << modbus_strerror(errno) << std::endl;  
    return -1;
    }
  
  DBG("Successfully set position gains: " << "Left: Kp=" << gains.left.kp << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Kf=" << gains.right.kf);    
  return 0;
}


int ZLAC8015D::set_speed_resolution(const ResolutionMode &mode) {

  if (!client_) return -1;
  if (mode == ResolutionMode::CERO_POINT_ONE_RPM){
    if (modbus_write_register(client_, SET_SPEED_RESOLUTION, RPM_RES_CERO_POINT_ONE) == -1) return -1; 
  }
  if (mode == ResolutionMode::ONE_RPM || mode == ResolutionMode::UNKNOWN){
    if (modbus_write_register(client_, SET_SPEED_RESOLUTION, RPM_RES_ONE) == -1) return -1; 
    if(mode == ResolutionMode::UNKNOWN){
      std::cerr << "The Value wasn't possible to be assigned - 1 RPM setted \n";
    }
  }
  std::cerr << "Remember to Restart the system to make effective the changes\n";
  return 0;
}

ResolutionMode ZLAC8015D::get_speed_resolution() {
  if (!client_) return ResolutionMode::UNKNOWN;
  uint16_t val{0}; 
  if(modbus_read_registers(client_, SET_SPEED_RESOLUTION, 1, &val)== -1) return ResolutionMode::UNKNOWN;
  ResolutionMode mode = ResolutionMode::UNKNOWN;
  if (val == RPM_RES_CERO_POINT_ONE) mode = ResolutionMode::CERO_POINT_ONE_RPM;
  else if (val == RPM_RES_ONE) mode = ResolutionMode::ONE_RPM;
  return mode;
}

int ZLAC8015D::save_to_eeprom(){

  if (!client_) return -1;
  if (modbus_write_register(client_, SAVE_EEPROM, 0x0001) == -1) return -1;
  std::cerr << "Values saved to EEPROM Remember to Restart the system to make effective the changes\n";
  return 0;
}


int ZLAC8015D::set_sync_rpm(float L_rpm, float R_rpm, const ResolutionMode &mode) {

  if (!client_) return -1;

  if (mode_ != OperationMode::VELOCITY) {
    DBG("To set RPM you have to be in velocity mode and you are in " << mode_ << "\n");
    DBG("For security we change your mode to Velocity\n");
    if (change_mode(OperationMode::VELOCITY) != 0) return -1;
  }

  const float step_rpm = (mode == ResolutionMode::CERO_POINT_ONE_RPM) ? 0.1f : 1.0f;
  constexpr float MAX_RPM = 3000.0f;

  if (L_rpm < -MAX_RPM || L_rpm > MAX_RPM || R_rpm < -MAX_RPM || R_rpm > MAX_RPM) {
    std::cerr << "\033[31m"
              << "RPM must be between -" << MAX_RPM << " and " << MAX_RPM << "\n"
              << "\033[0m";
    L_rpm = std::clamp(L_rpm, -MAX_RPM, MAX_RPM);
    R_rpm = std::clamp(R_rpm, -MAX_RPM, MAX_RPM);
  }

  int32_t l_units = static_cast<int32_t>(std::lround(L_rpm / step_rpm));
  int32_t r_units = static_cast<int32_t>(std::lround(R_rpm / step_rpm));

  l_units = std::clamp<int32_t>(l_units, -32768, 32767);
  r_units = std::clamp<int32_t>(r_units, -32768, 32767);

  uint16_t regs[2];
  regs[0] = static_cast<uint16_t>(static_cast<int16_t>(l_units));
  regs[1] = static_cast<uint16_t>(static_cast<int16_t>(r_units));

  // SET_RPM = 0x2088
  int rc = modbus_write_registers(client_, SET_RPM, 2, regs);
  if (rc == -1) {
    std::cerr << "\033[31m"
              << "set_sync_rpm failed: " << modbus_strerror(errno) << "\n"
              << "\033[0m";
    return -1;
  }

  DBG("Set RPM (cmd): Left=" << L_rpm << " Right=" << R_rpm
      << " | raw units: L=" << l_units << " R=" << r_units
      << " | step=" << step_rpm << " RPM/unit\n");

  return 0;
}


std::pair<float, float> ZLAC8015D::get_rpm(){

  if (!client_) return {-1.0f, -1.0f};
  uint16_t raw[2];
  int rc = modbus_read_registers(client_, GET_RPM, 2, raw);
  if (rc != 2) {
    std::cerr << "get_rpm failed: " << modbus_strerror(errno) << std::endl;
    return {-1.0f, -1.0f};
  }

  int16_t left_raw  = static_cast<int16_t>(raw[0]);
  int16_t right_raw = static_cast<int16_t>(raw[1]);

  float left_rpm  = static_cast<float>(left_raw) * 0.1f;
  float right_rpm = static_cast<float>(right_raw) * 0.1f;

  DBG("Current RPM: Left: " << left_rpm << " Right: " << right_rpm);
  return {left_rpm, right_rpm};
}

int ZLAC8015D::set_max_speed_position_mode(int16_t L_rpm, int16_t R_rpm){

  if (!client_) return -1;

  if(mode_ != OperationMode::RELATIVE_POSITION && mode_ != OperationMode::ABSOLUTE_POSITION) {
    DBG("To set RPM you have to be in relative_position or absolute_position mode and you are in " << mode_ << std::endl);
    DBG("For security we change your mode to relative_position" << std::endl);
    change_mode(OperationMode::RELATIVE_POSITION);
  }

  if(L_rpm < 0 || L_rpm > 1000 || R_rpm < 0 || R_rpm > 1000) { // Max 1000 RPM
    std::cerr << "RPM must be between 0 and 50\n";
    L_rpm = std::clamp<int16_t>(L_rpm, 0, 1000);
    R_rpm = std::clamp<int16_t>(R_rpm, 0, 1000);
    DBG("For security we set the max speed for position mode to: " << L_rpm << " for left and " << R_rpm << " for right\n");
  }

  if (modbus_write_register(client_, L_MAX_RPM_POS, L_rpm) == -1 ||
      modbus_write_register(client_, R_MAX_RPM_POS, R_rpm) == -1) {
    std::cerr << "Failed to set pos max speed: " << modbus_strerror(errno) << "\n";
    return -1;
  }
  DBG("Successfully set pos max speed: " << "right = " << R_rpm << ", left = " << L_rpm);
  return 0;
}

std::pair<int, int> ZLAC8015D::get_max_speed_position_mode(){

  if (!client_) return {-1, -1};

  uint16_t max_rpm_L = 0;
  uint16_t max_rpm_R = 0;
  if (modbus_read_registers(client_, L_MAX_RPM_POS, 1, &max_rpm_L) == -1 ||
      modbus_read_registers(client_, R_MAX_RPM_POS, 1, &max_rpm_R) == -1) {
    std::cerr << "Failed to get pos max speed: " << modbus_strerror(errno) << "\n";
    return {-1, -1};
  }
  DBG("Max motor speed limit for position mode: " << "right = " << max_rpm_R << ", left = " << max_rpm_L);
  return { static_cast<int>(max_rpm_L) , static_cast<int>(max_rpm_R)};
}


int ZLAC8015D::set_sync_position(float L_rad, float R_rad){

  if (!client_) return -1;
  if(mode_ != OperationMode::RELATIVE_POSITION && mode_ != OperationMode::ABSOLUTE_POSITION) {
    DBG("To set RPM you have to be in relative_position or absolute_position mode and you are in " << mode_ << std::endl);
    DBG("For security we change your mode to relative_position" << std::endl);
    change_mode(OperationMode::RELATIVE_POSITION);
  }

  auto [max_rpm_L, max_rpm_R]= get_max_speed_position_mode();

  if(max_rpm_L == -1 || max_rpm_R == -1) {
    DBG("Failed to get max speed for position mode, cannot set relative position\n");
  }

  if (max_rpm_L > 1000 || max_rpm_R > 1000 || max_rpm_L <= 0 || max_rpm_R <= 0) {
    if(set_max_speed_position_mode(1000, 1000) == 0){
      DBG("FOR SECURITY: Set default max speed for position mode to 50 RPM for both motors\n");
    }
    else {
      std::cerr << "Failed to set default max speed for position mode, cannot set relative position\n";
      return -1;
    }
  }

  int32_t L_pulses = static_cast<int32_t>(std::lround(L_rad * counts_per_rad)); // Assuming 4096 counts per revolution
  int32_t R_pulses = static_cast<int32_t>(std::lround(R_rad * counts_per_rad));

  DBG("Set succesfully:" << " Left: " << L_rad << " rad (" << L_pulses << " pulses), "
            << "Right: " << R_rad << " rad (" << R_pulses << " pulses)\n");

  uint16_t regs[4] = {
    static_cast<uint16_t>((L_pulses >> 16) & 0xFFFF),
    static_cast<uint16_t>( L_pulses        & 0xFFFF),
    static_cast<uint16_t>((R_pulses >> 16) & 0xFFFF),
    static_cast<uint16_t>( R_pulses        & 0xFFFF),
  };

  int rc = modbus_write_registers(client_, SET_POS, 4, regs);
  modbus_write_register(client_, CONTROL_REG, POS_SYNC);
  return (rc == 4) ? 0 : -1;
}

int ZLAC8015D::set_sync_current(int16_t L_mA, int16_t R_mA){

  if (!client_) return -1;
  if(mode_ != OperationMode::TORQUE) {
    DBG("To set torque you have to be in torque mode and you are in " << mode_ << std::endl);
    DBG("For security we change your mode to torque" << std::endl);
    change_mode(OperationMode::TORQUE);
  }

  if (L_mA < -30000 || L_mA > 30000 || R_mA < -30000 || R_mA > 30000) { // Max 30A
    std::cerr << "Current must be between -2000 mA and 2000 mA\n";
    L_mA = std::clamp<int16_t>(L_mA, -30000, 30000);
    R_mA = std::clamp<int16_t>(R_mA, -30000, 30000);
    DBG("For security we set the max current for torque mode to: " << L_mA << " mA for left and " << R_mA << " mA for right\n");
  }

  uint16_t regs[2] = {
    static_cast<uint16_t>(L_mA),
    static_cast<uint16_t>(R_mA)
  };

  int rc = modbus_write_registers(client_, SET_TORQUE, 2, regs);
  return (rc == 2) ? 0 : -1;
}

std::pair<float, float> ZLAC8015D::get_current(){

  if (!client_) return {-1, -1};
  uint16_t current[2];
  int rc = modbus_read_registers(client_, READ_ACTUAL_CURRENT, 2, current);
  if (rc == -1) {
    std::cerr << "get_current failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }
  DBG("Current actual mA: Left: " << current[0] * 0.1f << " Right: " << current[1] * 0.1f << std::endl);
  return { static_cast<float>(current[0]) * 0.1f, static_cast<float>(current[1]) * 0.1f };
}


int ZLAC8015D::enable_parking_mode(){

  if (!client_) return -1;
  return modbus_write_register(client_, PARKING_MODE_REG, 0x0001);
}

int ZLAC8015D::disable_parking_mode(){

  if (!client_) return -1;
  return modbus_write_register(client_, PARKING_MODE_REG, 0x0000);
}

std::pair<std::string, std::string> ZLAC8015D::get_error() {
  if (!client_) return {"Not connected", "Not connected"};

  uint16_t error_left{0};
  uint16_t error_right{0};

  if (modbus_read_registers(client_, FAULTS_LEFT, 1, &error_left) == -1) {
    return {std::string("Failed to read left error register: ") + modbus_strerror(errno),
            "Right: not read"};
  }

  if (modbus_read_registers(client_, FAULTS_RIGHT, 1, &error_right) == -1) {
    return {"Left: read ok",
            std::string("Failed to read right error register: ") + modbus_strerror(errno)};
  }

  return {hex16(error_left), hex16(error_right)};
}

std::string ZLAC8015D::decode_error(uint16_t reg, const char* side){

  if (!client_) return "Not connected";
  if (reg == 0) {
    return std::string(side) + ": No error. Driver is working properly.";
  }

  std::ostringstream out;
  out << side << ": Active faults (0x"
      << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << reg
      << std::dec << ")\n";

  uint16_t known_mask = 0;

  for (uint16_t bit = 1; bit != 0; bit <<= 1) {
    if (!(reg & bit)) continue;

    known_mask |= bit;

    switch (bit) {
      case 0x0001:
        out << "\n- Over Voltage Error:\n"
                "  - Power supply voltage is too high.\n"
                "  - Excessive back EMF (consider bleeder circuit)\n";
        break;

      case 0x0002:
        out << "\n- Under Voltage Error:\n"
                "  - Power supply voltage is too low.\n"
                "  - Check wiring connector.\n"
                "  - Check motor parameters.\n";
        break;

      case 0x0004:
        out << "\n- Motor Overcurrent Fault:\n"
                "  - Instantaneous current is too high.\n"
                "  - Motor power cable is loose.\n";
        break;

      case 0x0008:
        out << "\n- Motor Overload Fault:\n"
                "  - Check if the motor cable is loose.\n"
                "  - Check wiring and motor parameters.\n"
                "  - Motor is stall.\n"
                "  - Motor or driver problem.\n";
        break;

      case 0x0020:
        out << "\n- Encoder Value out of Tolerance:\n"
                "  - Motor is stall.\n"
                "  - Encoder problem.\n";
        break;

      case 0x0080:
        out << "\n- Reference Voltage Error:\n"
                "  - Reference voltage circuit issue.\n";
        break;

      case 0x0100:
        out << "\n- EEPROM Read/Write Error:\n"
                "  - Firmware upgraded (needs factory settings).\n"
                "  - EEPROM circuit damaged.\n";
        break;

      case 0x0200:
        out << "\n- Hall Error:\n"
                "  - Check motor cable.\n"
                "  - Motor problem.\n"
                "  - Driver problem.\n";
        break;

      case 0x0400:
        out << "\n- Temperature too High:\n"
                "  - Motor current too high (monitor and reduce current).\n"
                "  - Motor thermistor damaged.\n"
                "  - Driver circuit damaged.\n";
        break;

      case 0x0800:
        out << "\n- Encoder Error:\n"
                "  - Check encoder cable.\n"
                "  - Check if encoder cable is disconnected.\n";
        break;

      case 0x2000:
        out << "\n- Speed Setting Error:\n"
                "  - Given speed exceeds rated speed.\n";
        break;

      default:
        out << "\n- Unknown fault bit set: 0x"
            << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << bit
            << std::dec << "\n";
        break;
    }
  }

  uint16_t unknown = reg & ~known_mask;
  if (unknown) {
    out << "\n- Unmapped bits present: 0x"
        << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << unknown
        << std::dec << "\n";
  }

  return out.str();
};

std::string ZLAC8015D::hex16(uint16_t v) {

  std::ostringstream oss;
  oss << "0x"
      << std::hex << std::uppercase
      << std::setw(4) << std::setfill('0')
      << v;
  return oss.str();
}


std::pair<int, int> ZLAC8015D::get_temperature() {

  if (!client_) return {-1, -1};
  uint16_t temp;
  int rc = modbus_read_registers(client_, READ_TEMPERATURE, 1, &temp);
  if (rc == -1) {
    std::cerr << "get_temperature failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }
  DBG("Current Temperature: Left: " << (temp >> 8) << " °C Right: " << (temp & 0xFF) << " °C" << std::endl);
  return { static_cast<int>(temp >> 8), static_cast<int>(temp & 0xFF) };
}
  
std::string ZLAC8015D::get_software_version(){

  if (!client_) return "Not connected";
  uint16_t version;
  int rc = modbus_read_registers(client_, READ_ACTUAL_SOFTWARE_VERSION, 1, &version);
  if (rc == -1) {
    std::cerr << "get_software_version failed: " << modbus_strerror(errno) << std::endl;
    return "Not connected";
  }
  DBG("Driver Software Version: " << hex16(version));
  return hex16(version);
}

int ZLAC8015D::set_decel_time(uint16_t decel_time_ms) {

  if (!client_) return -1;

  if (decel_time_ms > 32767) {
    DBG("Deceleration time must be between 0 and 32767 ms\n");
    decel_time_ms = std::clamp<uint16_t>(decel_time_ms, 0u, 32767u);
    DBG("For security we set the deceleration time to: " << decel_time_ms << " ms\n");
  }

  int rc = modbus_write_register(client_, L_DCL_TIME, decel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set deceleration time in Left motor - Right motor is not setted: " << modbus_strerror(errno) << std::endl;
    return -1;
  }

  rc = modbus_write_register(client_, R_DCL_TIME, decel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set deceleration time in Right motor - Left motor was set: " << modbus_strerror(errno) << std::endl;
    return -1;
  }
  return 0;
}

int ZLAC8015D::set_accel_time(uint16_t accel_time_ms) {

  if (!client_) return -1;
  
  if (accel_time_ms > 32767) {
    DBG( "Acceleration time must be between 0 and 32767 ms\n");
    accel_time_ms = std::clamp<uint16_t>(accel_time_ms, 0u, 32767u);
    DBG("For security we set the acceleration time to: " << accel_time_ms << " ms\n");
  }

  int rc = modbus_write_register(client_, L_ACL_TIME, accel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set acceleration time in Left motor - Right motor is not setted: " << modbus_strerror(errno) << std::endl;
    return -1;
  }

  rc = modbus_write_register(client_, R_ACL_TIME, accel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set acceleration time in Right motor - Left motor was set: " << modbus_strerror(errno) << std::endl;
    return -1;
  }
  return 0;
} 

                
        