#ifndef EYES_GUI__MAIN_WINDOW_HPP_
#define EYES_GUI__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QStackedWidget>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include "eyes_gui/debug_view.hpp"
#include "eyes_gui/face_view.hpp"
#include "eyes_gui/idle_event_filter.hpp"

namespace eyes_gui
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);

  rclcpp::Node::SharedPtr debugNode() const;

public Q_SLOTS:
  void showFaceView();
  void showDebugView();
  void setCrying(bool enabled);
  void recover();
  void blink();
  void toggleFullscreen();
  void handleStateCommand(const QString & state);
  void publishCurrentUiState();

Q_SIGNALS:
  void currentStateChangedForRos(const QString & state);

protected:
  void keyPressEvent(QKeyEvent * event) override;

private:
  void resetIdleTimer();
  void updateDebugStatusLabels();
  void publishStateIfChanged();

  QStackedWidget * stack_;
  FaceView * face_view_;
  DebugView * debug_view_;
  IdleEventFilter * idle_filter_;
  QTimer idle_timer_;
  QString last_published_state_;
  bool is_fullscreen_;
};

}  // namespace eyes_gui

#endif  // EYES_GUI__MAIN_WINDOW_HPP_
