#include <iostream>
#include "zlac8015d_driver.h"
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <unistd.h> // For access() F_OK definition

static std::atomic_bool running{true};
static void on_sigint(int) { 
  running = false;
  std::cout << "Stopped by Ctrl+C\n";
}

int main(int argc, char * argv[]) {

  std::string port = "";
  if (argc >= 2) {
    port = argv[1];
    // Validate if port exists
    if (access(port.c_str(), F_OK) != 0) {
      std::cerr << "Port " << port << " does not exist\n";
      return 1;
    }
    std::cout << "Port: " << port << "\n";
  } else {
    std::cout << "No port specified\nUsage:\n$ ros2 run <package_name> <node_name> <port>\n";
    return 1;
  }

  std::signal(SIGINT, on_sigint);

  // Initialize the driver on the specified port, 115200 baud, velocity mode
  ZLAC8015D driver(port, 115200, OperationMode::VELOCITY);
  if (!driver.connect()) return (std::cerr << "Failed to connect\n", 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Check and setup speed resolution if needed
  if(driver.get_speed_resolution() != ResolutionMode::CERO_POINT_ONE_RPM){
      driver.set_sync_rpm(0, 0, ResolutionMode::ONE_RPM);
      driver.enable_motor();

    if(driver.set_speed_resolution(ResolutionMode::CERO_POINT_ONE_RPM) == 0){
      std::cerr << "\033[31m" << "0.1 RPM Resolution Successfully Set\n" << "\033[0m";
      if(driver.save_to_eeprom() == -1){
        std::cerr << "\033[31m" << "ERROR saving in EEPROM Memory\n" << "\033[0m";
        return 1;
      }
      std::cerr << "\033[31m" << "Driver must be restarted\n" << "\033[0m";
      std::cerr << "\033[31m" << "This is only supported for some ZLAC8015D models. Be careful after running it a second time.\n" << "\033[0m";
      return 1;
    }

    std::cerr << "\033[31m"
          << "ERROR: RPM resolution different than 0.1 RPM and problems setting it\n"
          << "\033[0m";
    return 1;
  }

  // Set acceleration and deceleration time
  driver.set_decel_time(3000); 
  driver.set_accel_time(3000);

  std::cout << "\x1B[1;33m Motors will move ...\x1B[0m\n";

  while (running) {
    // Read and decode active faults/errors
    auto [err_left, err_right] = driver.get_error();

    if (err_left == "0x0002" && err_right == "0x0002") {
      driver.reset_alarm();
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      driver.enable_motor();
      std::cout << "Driver connected...\n";
    }

    // Command motors to move
    driver.set_sync_rpm(10, -10, ResolutionMode::ONE_RPM);

    // Read encoder
    auto [enc_left, enc_right, status_flag] = driver.get_encoder_count();
    if(status_flag){
      std::cout << "Encoder counts: Left: " << enc_left << " Right: " << enc_right << "\n";
    } else {
      std::cout << "\x1B[1;33m A problem reading the driver was detected \x1B[0m\n";
    }

    // Read temperature
    auto [temp_left, temp_right] = driver.get_temperature();
    std::cout << "Temperatures: Left: " << temp_left << "°C Right: " << temp_right << "°C\n";

    // Read driver software version
    std::string software_version = driver.get_software_version();
    std::cout << "Software Version: " << software_version << "\n";

    // Read power current
    auto [current_left, current_right] = driver.get_current();
    std::cout << "Current: Left: " << current_left << "mA Right: " << current_right << "mA\n";

    // Read real RPM
    auto [left_rpm, right_rpm] = driver.get_rpm();
    std::cout << "RPM: Left: " << left_rpm << " Right: " << right_rpm << "\n"; 

    // Decode errors to string definitions
    std::string decoded_left = err_left;
    std::string decoded_right = err_right;

    try {
      int code_left = std::stoi(err_left, nullptr, 16);   // base 16, accepts "0x...."
      decoded_left = driver.decode_error(code_left, "left");
    } catch (const std::exception& e) {
      decoded_left = "Decode failed for left error: " + err_left;
    }

    try {
      int code_right = std::stoi(err_right, nullptr, 16);
      decoded_right = driver.decode_error(code_right, "right");
    } catch (const std::exception& e) {
      decoded_right = "Decode failed for right error: " + err_right;
    }

    std::cout << "Decoded left: " << decoded_left << "\n";
    std::cout << "Decoded right: " << decoded_right << "\n\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  driver.set_sync_rpm(0, 0, ResolutionMode::ONE_RPM);
  driver.disable_motor();
  return 0;
}
