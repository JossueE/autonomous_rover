#include <memory>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <execinfo.h>
#include <unistd.h>

#include <QApplication>
#include <QCoreApplication>

#include <rclcpp/rclcpp.hpp>

#include "eyes_gui/face_state_node.hpp"
#include "eyes_gui/main_window.hpp"

namespace
{

void crashSignalHandler(int signal_number)
{
  void * frames[64];
  const int frame_count = ::backtrace(frames, 64);

  std::fprintf(
    stderr,
    "\n[robot_face_debug_ui][crash] caught signal %d, frames=%d\n",
    signal_number,
    frame_count);
  std::fflush(stderr);
  ::backtrace_symbols_fd(frames, frame_count, STDERR_FILENO);
  std::fflush(stderr);

  std::_Exit(128 + signal_number);
}

void installCrashHandlers()
{
  std::signal(SIGSEGV, crashSignalHandler);
  std::signal(SIGABRT, crashSignalHandler);
  std::signal(SIGBUS, crashSignalHandler);
  std::signal(SIGILL, crashSignalHandler);
}

void debugStep(const char * step)
{
  std::fprintf(stderr, "[robot_face_debug_ui][main] %s\n", step);
  std::fflush(stderr);
}

}  // namespace

int main(int argc, char ** argv)
{
  installCrashHandlers();
  debugStep("main: rclcpp init begin");
  rclcpp::init(argc, argv);
  debugStep("main: rclcpp init end");

  debugStep("main: QApplication create begin");
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication app(argc, argv);
  QApplication::setApplicationName("robot_face_debug_ui");
  debugStep("main: QApplication create end");

  debugStep("main: FaceStateNode create begin");
  auto face_state_node = std::make_shared<eyes_gui::FaceStateNode>();
  debugStep("main: FaceStateNode create end");
  debugStep("main: MainWindow create begin");
  eyes_gui::MainWindow window;
  debugStep("main: MainWindow create end");

  QObject::connect(
    face_state_node.get(),
    &eyes_gui::FaceStateNode::stateCommandReceived,
    &window,
    &eyes_gui::MainWindow::handleStateCommand,
    Qt::QueuedConnection);
  QObject::connect(
    &window,
    &eyes_gui::MainWindow::currentStateChangedForRos,
    face_state_node.get(),
    &eyes_gui::FaceStateNode::publishCurrentState,
    Qt::QueuedConnection);

  debugStep("main: executor setup begin");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(face_state_node);
  if (window.debugNode()) {
    executor.add_node(window.debugNode());
  }

  std::thread ros_thread([&executor]() {
    executor.spin();
  });
  debugStep("main: executor setup end");

  debugStep("main: show window begin");
  window.publishCurrentUiState();
  window.show();
  debugStep("main: show window end");

  debugStep("main: app exec begin");
  const int result = app.exec();
  debugStep("main: app exec end");

  debugStep("main: shutdown begin");
  executor.cancel();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  debugStep("main: shutdown end");
  return result;
}
