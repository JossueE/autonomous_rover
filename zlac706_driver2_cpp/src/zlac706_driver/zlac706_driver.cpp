#include "zlac706_driver.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;

void ZLAC706::setSpeedMode() { commandSent = speedMCommand; }
void ZLAC706::configAcc(const uint8_t acc, const uint8_t decc) {
  commandSent = configCommand;
}
void ZLAC706::setSpeed(int speed) {

  speed = (int)((speed * 8192.0) / 3000); //(int) ((speed / 3000.0) * 8192);
  sSpeed_msg[1] = ((speed >> 8) & 0xff);  // speed / 256; //H;
  sSpeed_msg[2] = (speed & 0xff);         // speed % 256; //L;
  sSpeed_msg[3] = (setSpeedCommand + sSpeed_msg[1] + sSpeed_msg[2]) &
                  0xff; //(setSpeedCommand+H+L)%256;
  commandSent = setSpeedCommand;
}
void ZLAC706::start() { commandSent = startCommad; }
void ZLAC706::EstAlarm() { commandSent = alarmstatus; }
void ZLAC706::keepRunning() { commandSent = aliveCommand; }
void ZLAC706::stop() { commandSent = startCommad; }

void ZLAC706::setGain(const uint8_t gain, const int value) {
  switch (gain) {
  case 1:
    kiG_msg[1] = ((value >> 8) & 0xff); // value / 256;
    kiG_msg[2] = (value & 0xff);        // value % 256;
    kiG_msg[3] = (setKiCommand + kiG_msg[1] + kiG_msg[2]) & 0xff;
    commandSent = setKiCommand;
    break;
  case 2:
    kdG_msg[1] = ((value >> 8) & 0xff); // value / 256;
    kdG_msg[2] = (value & 0xff);        // value % 256;
    kdG_msg[3] = (setKdCommand + kdG_msg[1] + kdG_msg[2]) & 0xff;
    commandSent = setKdCommand;
    break;
  default:
    kpG_msg[1] = ((value >> 8) & 0xff); // value / 256;
    kpG_msg[2] = (value & 0xff);        // value % 256;
    kpG_msg[3] = (setKpCommand + kpG_msg[1] + kpG_msg[2]) & 0xff;
    commandSent = setKpCommand;
    break;
  }
}
void ZLAC706::getVolts() { commandSent = getVoltCommand; }
void ZLAC706::getAmps() { commandSent = getAmpsCommnad; }
void ZLAC706::getSpeed() { commandSent = getSpeedCommand; }
void ZLAC706::getTicks() { commandSent = getTicksCommand; }
void ZLAC706::clearAlarms() { commandSent = alarmCommand; }
void ZLAC706::getParams() { commandSent = readParamsFlag; }

void ZLAC706::getDevices(vector<string> &devices) {
  string result, result2, grepProduct, grepCommand;
  grepProduct = "grep PRODUCT= /sys/bus/usb-serial/devices//../uevent";
  result = exec("ls /dev/ttyU*");
  bool moreData = true;
  std::size_t found = 0;
  std::size_t bfound = 0;
  while (moreData) {
    found = result.find('\n', found);
    moreData = false;
    if (found != std::string::npos) {
      grepCommand = grepProduct;
      grepCommand.insert(42, result.substr(bfound + 5, found - bfound - 5));
      char *tab2 = new char[grepCommand.length() + 1];
      strcpy(tab2, grepCommand.c_str());
      result2 = exec(tab2);
      if (result2 == "PRODUCT=67b/2303/300\n")
        devices.push_back(result.substr(bfound + 5, found - bfound - 5));
      moreData = true;
      found++;
      bfound = found;
    }
  }
}
void ZLAC706::close() {
  // ZLAC706::serial.close();
}
void ZLAC706::Alarmstatus(const char *data) {
  if (data[4] == 0x00) {
    status = "Stop state";
    // Stop state
  } else if (data[4] == 0x01) {
    status = "Startup state";
    // Startup state
  } else if (data[4] == 0x02) {
    status = "Overcurrent";
    // Overcurrent
  } else if (data[4] == 0x04) {
    status = "Overvoltage";
    // Overvoltage
  } else if (data[4] == 0x08) {
    status = "Encoder error";
    // Encoder error
  } else if (data[4] == 0x10) {
    status = "Overheat";
    // Overheat
  } else if (data[4] == 0x20) {
    status = "Undervoltage";
    // Undervoltage
  } else if (data[4] == 0x40) {
    status = "Overload";
    // Overload
  } else {
    status = "";
  }
}

void ZLAC706::readData(const vector<char> &v) {
  for (unsigned int i = 0; i < v.size(); i++) {
    if (((unsigned char)v[i] == aliveCommand ||
         (unsigned char)v[i] == voltageCommand ||
         (unsigned char)v[i] == speedCommand ||
         (unsigned char)v[i] == postionHCommand ||
         (unsigned char)v[i] == postionLCommand)) {
      if ((int)commandCount == 0) {
        isCommand = false;
      }
    }
    // Verify command header
    if (!isCommand && ((unsigned char)v[i] == aliveCommand ||
                       (unsigned char)v[i] == voltageCommand ||
                       (unsigned char)v[i] == speedCommand ||
                       (unsigned char)v[i] == postionHCommand ||
                       (unsigned char)v[i] == postionLCommand)) {
      isCommand = true;
      commandResponse[0] = (unsigned char)v[i];
      commandCount = 1;
    }
    // Read command values
    else if (isCommand && commandCount > 0 && commandCount < 3) {
      commandResponse[commandCount] = (unsigned char)v[i];
      commandCount++;
    }
    // Analyze checkSum
    else if (isCommand && commandCount == 3) {
      if ((unsigned char)v[i] ==
          (commandResponse[0] + commandResponse[1] + commandResponse[2]) %
              256) {
        isCommandResponse = true;
      } else {
        isCommand = false;
        isCommandResponse = false;
        commandCount = 0;
      }
    }
    // Process command response
    if (isCommandResponse) {
      switch (commandResponse[0]) {
      case aliveCommand:
        if (commandResponse[2] == 0x00) {
          status = "Stop state";
        } else if (commandResponse[2] == 0x01) {
          status = "Startup state";
        } else if (commandResponse[2] == 0x02) {
          status = "Overcurrent";
        } else if (commandResponse[2] == 0x04) {
          status = "Overvoltage";
        } else if (commandResponse[2] == 0x08) {
          status = "Encoder error";
        } else if (commandResponse[2] == 0x10) {
          status = "Overheat";
        } else if (commandResponse[2] == 0x20) {
          status = "Undervoltage";
        } else if (commandResponse[2] == 0x40) {
          status = "Overload";
        } else {
          status = "Unknown";
        }
        isE8 = false;
        break;
      case voltageCommand:
        voltage = (commandResponse[1] << 8) | commandResponse[2];
        break;
      case speedCommand:
        speed = 6000 * ((commandResponse[1] << 8) | commandResponse[2]) / 16384;
        break;
      case postionHCommand:
        messageE8[0] = commandResponse[1];
        messageE8[1] = commandResponse[2];
        isE8 = true;
        break;
      case postionLCommand:
        ticks_v[2] = commandResponse[1];
        ticks_v[3] = commandResponse[2];
        if (isE8) {
          ticks = (((messageE8[0] << 8) | messageE8[1]) << 16) |
                  ((commandResponse[1] << 8) | commandResponse[2]);
        }
        isE8 = false;
        break;
      default:
        break;
      }
      isCommand = false;
      isCommandResponse = false;
      commandCount = 0;
    }
  }
}
void ZLAC706::checkCommnand(const vector<char> &v, unsigned int hexCommand) {
  for (unsigned int i = 0; i < v.size(); i++) {
    // Analyze the packet starting from E8
    if (!isCommand && (unsigned char)v[i] == hexCommand) {
      isCommand = true;
      commandCount = 0;
    } else if (isCommand && commandCount == 0 &&
               (unsigned char)v[i] == hexCommand) {
      isCommand = false;
      isCommandResponse = true;
      commandCount++;
    }
    if (isCommandResponse) {
      // cout << "Comando recibido" << endl;
      isCommandResponse = false;
    }
  }
}
void ZLAC706::getResponse(const vector<char> &v, unsigned int hexCommand) {
  for (unsigned int i = 0; i < v.size(); i++) {
    // Analyze the packet starting from E8
    if (!isCommand && (unsigned char)v[i] == hexCommand) {
      isCommand = true;
      commandResponse[0] = hexCommand;
      commandCount = 0;
    } else if (isCommand && commandCount < 4) {
      commandResponse[commandCount] = (unsigned char)v[i];
      commandCount++;
    } else if (isCommand && commandCount == 4) {

      if ((unsigned char)v[i] ==
          (commandResponse[1] + commandResponse[2] + commandResponse[3]) %
              256) {
        isCommandResponse = true;
      }
    }
    if (isCommandResponse) {
      response = (commandResponse[2] << 8) | commandResponse[3];
      // response = (256 * commandResponse[2]) + commandResponse[3];
      // cout << "Comando recibido " << response << endl;
      isCommand = false;
      isCommandResponse = false;
    }
  }
}

void ZLAC706::readParams(const vector<char> &v) {
  for (unsigned int i = 0; i < v.size(); i++) {
    // Verify command header
    if (!isCommand && ((unsigned char)v[i] == getKpCommand ||
                       (unsigned char)v[i] == getKiCommand ||
                       (unsigned char)v[i] == getKdCommand ||
                       (unsigned char)v[i] == getCanIDCommand)) {

      isCommand = true;
      commandResponse[0] = (unsigned char)v[i];
      commandCount = 1;
    }
    // Read command values
    else if (isCommand && commandCount > 0 && commandCount < 3) {
      commandResponse[commandCount] = (unsigned char)v[i];
      commandCount++;
    }
    // Analyze checkSum
    else if (isCommand && commandCount == 3) {
      if ((unsigned char)v[i] ==
          (commandResponse[0] + commandResponse[1] + commandResponse[2]) %
              256) {
        isCommandResponse = true;
      } else {
        isCommand = false;
        isCommandResponse = false;
        commandCount = 0;
      }
    }
    // Process command response
    if (isCommandResponse) {
      switch (commandResponse[0]) {
      case getKpCommand:
        kp = (commandResponse[1] << 8) | commandResponse[2];
        break;
      case getKiCommand:
        ki = (commandResponse[1] << 8) | commandResponse[2];
        break;
      case getKdCommand:
        kd = (commandResponse[1] << 8) | commandResponse[2];
        break;
      default:
        can_id = (commandResponse[1] << 8) | commandResponse[2];
        break;
      }
      isCommand = false;
      isCommandResponse = false;
      commandCount = 0;
    }
  }
}

void ZLAC706::received(const char *data, unsigned int len) {
  vector<char> v(data, data + len);

  switch (commandSent) {
  case getVoltCommand:
  case getAmpsCommnad:
  case getSpeedCommand:
    getResponse(v, commandSent);
    break;
  case getTicksCommand:
    readData(v);
    break;
  case readParamsFlag:
    readParams(v);
    break;
  case aliveCommand:
    readData(v);
    break;
  case alarmstatus:
    Alarmstatus(data);
    break;
  default: /* echo commads */
    checkCommnand(v, commandSent);
    break;
  }
}

string ZLAC706::exec(const char *cmd) {
  std::array<char, 128> buffer;
  string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  std::size_t found = 0, last;
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

// printf("%02x,", (unsigned char) v[i]);
