#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include "zlac706_driver.h"

class WheelWriter {
public:
  WheelWriter(ZLAC706 &motor, CallbackAsyncSerial &serial);

  void start();
  void stop_thread();
  void request_start();
  void request_stop();
  void request_speed(double rpm);
  void request_encoder();

private:
  void loop();

  ZLAC706 &motor_;
  CallbackAsyncSerial &serial_;

  std::thread thread_;
  std::mutex mtx_;
  std::condition_variable cv_;

  bool running_{false};
  bool start_requested_{false};
  bool stop_requested_{false};
  bool speed_requested_{false};
  bool encoder_requested_{false};
  double latest_rpm_{0.0};
};
