#include "wheels_writer.h"

WheelWriter::WheelWriter(ZLAC706 &motor, CallbackAsyncSerial &serial)
: motor_(motor), serial_(serial) {}

void WheelWriter::start() {
  running_ = true;
  thread_ = std::thread(&WheelWriter::loop, this);
}

void WheelWriter::stop_thread() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    running_ = false;
  }
  cv_.notify_one();

  if (thread_.joinable()) {
    thread_.join();
  }
}

void WheelWriter::request_start() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    start_requested_ = true;
  }
  cv_.notify_one();
}

void WheelWriter::request_stop() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_requested_ = true;
  }
  cv_.notify_one();
}

void WheelWriter::request_speed(double rpm) {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_rpm_ = rpm;
    speed_requested_ = true;
  }
  cv_.notify_one();
}

void WheelWriter::request_encoder() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    encoder_requested_ = true;
  }
  cv_.notify_one();
}

void WheelWriter::loop() {
  while (true) {
    bool do_start = false;
    bool do_stop = false;
    bool do_speed = false;
    bool do_encoder = false;
    double rpm = 0.0;

    {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_.wait(lock, [this]() {
        return !running_ || start_requested_ || stop_requested_ ||
               speed_requested_ || encoder_requested_;
      });

      if (!running_) {
        break;
      }

      do_start = start_requested_;
      do_stop = stop_requested_;
      do_speed = speed_requested_;
      do_encoder = encoder_requested_;
      rpm = latest_rpm_;

      start_requested_ = false;
      stop_requested_ = false;
      speed_requested_ = false;
      encoder_requested_ = false;
    }

    if (do_start) {
      motor_.start();
      serial_.write(motor_.str_msg);
    }

    if (do_stop) {
      motor_.stop();
      serial_.write(motor_.stp_msg);
    }

    if (do_speed) {
      motor_.setSpeed(rpm);
      serial_.write(motor_.sSpeed_msg);
    }

    if (do_encoder) {
      motor_.getTicks();
      serial_.write(motor_.gTicks_msg);
    }
  }
}
