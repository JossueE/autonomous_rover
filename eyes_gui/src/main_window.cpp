#include "eyes_gui/main_window.hpp"

#include <QApplication>
#include <QKeyEvent>

namespace eyes_gui
{

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent),
  stack_(new QStackedWidget(this)),
  face_view_(new FaceView(this)),
  debug_view_(new DebugView(this)),
  idle_filter_(new IdleEventFilter(this)),
  is_fullscreen_(false)
{
  setWindowTitle("robot_face_debug_ui");
  setObjectName("RobotFaceDebugUi");
  setFocusPolicy(Qt::StrongFocus);
  resize(1280, 720);

  stack_->addWidget(face_view_);
  stack_->addWidget(debug_view_);
  setCentralWidget(stack_);

  idle_timer_.setInterval(30000);
  idle_timer_.setSingleShot(true);

  connect(face_view_, &FaceView::debugRequested, this, &MainWindow::showDebugView);
  connect(debug_view_, &DebugView::backRequested, this, &MainWindow::showFaceView);
  connect(debug_view_, &DebugView::activityDetected, this, &MainWindow::resetIdleTimer);
  connect(idle_filter_, &IdleEventFilter::activityDetected, this, &MainWindow::resetIdleTimer);
  connect(&idle_timer_, &QTimer::timeout, this, &MainWindow::showFaceView);

  debug_view_->installEventFilter(idle_filter_);
  if (debug_view_->renderPanel()) {
    debug_view_->renderPanel()->installEventFilter(idle_filter_);
  }

  showFaceView();
}

rclcpp::Node::SharedPtr MainWindow::debugNode() const
{
  return debug_view_->debugNode();
}

void MainWindow::showFaceView()
{
  stack_->setCurrentWidget(face_view_);
  idle_timer_.stop();
  idle_filter_->setEnabled(false);
  updateDebugStatusLabels();
  publishStateIfChanged();
}

void MainWindow::showDebugView()
{
  stack_->setCurrentWidget(debug_view_);
  debug_view_->ensureRvizInitialized();
  idle_filter_->setEnabled(true);
  resetIdleTimer();
  updateDebugStatusLabels();
  publishStateIfChanged();
}

void MainWindow::setCrying(bool enabled)
{
  face_view_->setCrying(enabled);
  updateDebugStatusLabels();
  publishStateIfChanged();
}

void MainWindow::recover()
{
  setCrying(false);
}

void MainWindow::blink()
{
  face_view_->blink();
}

void MainWindow::toggleFullscreen()
{
  is_fullscreen_ = !is_fullscreen_;
  if (is_fullscreen_) {
    showFullScreen();
  } else {
    showNormal();
  }
}

void MainWindow::handleStateCommand(const QString & state)
{
  const QString normalized = state.trimmed().toLower();

  if (normalized == "normal" || normalized == "recover") {
    recover();
    showFaceView();
  } else if (normalized == "crying") {
    setCrying(true);
    showFaceView();
  } else if (normalized == "debug") {
    showDebugView();
  } else if (normalized == "face") {
    showFaceView();
  } else if (normalized == "fullscreen") {
    toggleFullscreen();
  } else if (normalized == "blink") {
    blink();
  }
}

void MainWindow::publishCurrentUiState()
{
  last_published_state_.clear();
  publishStateIfChanged();
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
  switch (event->key()) {
    case Qt::Key_Escape:
      close();
      event->accept();
      return;
    case Qt::Key_Space:
      blink();
      event->accept();
      return;
    case Qt::Key_C:
      setCrying(true);
      event->accept();
      return;
    case Qt::Key_R:
      recover();
      event->accept();
      return;
    case Qt::Key_F:
      toggleFullscreen();
      event->accept();
      return;
    case Qt::Key_D:
      showDebugView();
      event->accept();
      return;
    case Qt::Key_B:
    case Qt::Key_Backspace:
      showFaceView();
      event->accept();
      return;
    default:
      QMainWindow::keyPressEvent(event);
      return;
  }
}

void MainWindow::resetIdleTimer()
{
  if (stack_->currentWidget() == debug_view_) {
    idle_timer_.start();
  }
}

void MainWindow::updateDebugStatusLabels()
{
  debug_view_->setStatusLabels(
    stack_->currentWidget() == debug_view_ ? "Debug" : "Face",
    face_view_->isCrying());
}

void MainWindow::publishStateIfChanged()
{
  QString state;
  if (stack_->currentWidget() == debug_view_) {
    state = "debug";
  } else {
    state = face_view_->isCrying() ? "crying" : "normal";
  }

  if (state == last_published_state_) {
    return;
  }

  last_published_state_ = state;
  Q_EMIT currentStateChangedForRos(state);
}

}  // namespace eyes_gui
