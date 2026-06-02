#include "eyes_gui/debug_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <OgreColourValue.h>

#include <QApplication>
#include <QColor>
#include <QContextMenuEvent>
#include <QDockWidget>
#include <QEvent>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMenuBar>
#include <QMouseEvent>
#include <QPushButton>
#include <QStatusBar>
#include <QStyle>
#include <QTouchEvent>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QVariant>
#include <QWheelEvent>

#include <rviz_common/properties/property.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp>
#include <rviz_rendering/render_system.hpp>
#include <rviz_rendering/render_window.hpp>

namespace eyes_gui
{

namespace
{

constexpr double kRotateScale = 0.006;
constexpr double kPanScale = 0.025;
constexpr double kPinchZoomScale = 0.035;
constexpr double kWheelZoomScale = 0.002;

QString stampToText(const builtin_interfaces::msg::Time & stamp)
{
  return QString("%1.%2").arg(stamp.sec).arg(stamp.nanosec, 9, 10, QLatin1Char('0'));
}

QString normalizeTopic(const QString & topic)
{
  return topic.trimmed();
}

void debugStep(const char * step)
{
  std::fprintf(stderr, "[robot_face_debug_ui][debug_view] %s\n", step);
  std::fflush(stderr);
}

void debugStepPtr(const char * step, const void * pointer)
{
  std::fprintf(stderr, "[robot_face_debug_ui][debug_view] %s ptr=%p\n", step, pointer);
  std::fflush(stderr);
}

}  // namespace

DebugView::DebugView(QWidget * parent)
: QWidget(parent),
  render_panel_(nullptr),
  visualization_frame_(nullptr),
  render_container_(nullptr),
  render_layout_(nullptr),
  render_placeholder_(nullptr),
  rviz_node_(nullptr),
  manager_(nullptr),
  debug_node_(
    std::make_shared<rclcpp::Node>(
      "eyes_gui_debug_text",
      rclcpp::NodeOptions().use_global_arguments(false))),
  side_panel_(nullptr),
  restore_panel_(nullptr),
  side_panel_toggle_(nullptr),
  side_panel_restore_(nullptr),
  screen_label_(nullptr),
  face_state_label_(nullptr),
  fixed_frame_label_(nullptr),
  rtab_text_(nullptr),
  last_pinch_distance_(0.0),
  last_touch_count_(0),
  mouse_dragging_(false),
  mouse_button_(Qt::NoButton),
  rviz_initialized_(false),
  rviz_failed_(false),
  side_panel_collapsed_(false)
{
  debugStep("DebugView constructor: begin");
  setObjectName("DebugView");
  setAttribute(Qt::WA_AcceptTouchEvents, true);
  setFocusPolicy(Qt::StrongFocus);
  setStyleSheet(
    "QWidget#DebugView { background: #101216; color: #edf1f7; }"
    "QWidget#SidePanel { background: #171b21; border-left: 1px solid #2a3038; }"
    "QLabel { color: #edf1f7; }"
    "QPushButton { background: #2a3038; color: #f5f7fb; border: 1px solid #3b444f; "
    "border-radius: 4px; padding: 8px 10px; }"
    "QPushButton:hover { background: #37414d; }"
    "QCheckBox { color: #dfe6ee; spacing: 8px; }"
    "QPlainTextEdit { background: #0c0e11; color: #cfd8e3; border: 1px solid #313842; "
    "font-family: monospace; font-size: 11px; }");

  debugStep("DebugView constructor: buildUi begin");
  buildUi();
  debugStep("DebugView constructor: buildUi end");
  debugStep("DebugView constructor: setupRtabSubscriptions begin");
  setupRtabSubscriptions();
  debugStep("DebugView constructor: setupRtabSubscriptions end");
  debugStep("DebugView constructor: end");
}

DebugView::~DebugView()
{
  debugStep("DebugView destructor: begin");
  if (manager_) {
    debugStep("DebugView destructor: manager stopUpdate begin");
    manager_->stopUpdate();
    debugStep("DebugView destructor: manager stopUpdate end");
  }
  debugStep("DebugView destructor: end");
}

rclcpp::Node::SharedPtr DebugView::debugNode() const
{
  return debug_node_;
}

rviz_common::RenderPanel * DebugView::renderPanel() const
{
  return render_panel_;
}

bool DebugView::ensureRvizInitialized()
{
  debugStep("ensureRvizInitialized: enter");
  if (rviz_initialized_) {
    debugStep("ensureRvizInitialized: already initialized");
    return true;
  }
  if (rviz_failed_) {
    debugStep("ensureRvizInitialized: previous failure");
    return false;
  }

  try {
    debugStep("ensureRvizInitialized: initializeRviz begin");
    initializeRviz();
    debugStep("ensureRvizInitialized: initializeRviz end");
    rviz_initialized_ = true;
    debugStep("ensureRvizInitialized: success");
    return true;
  } catch (const std::exception & exception) {
    std::fprintf(
      stderr,
      "[robot_face_debug_ui][debug_view] ensureRvizInitialized: std::exception: %s\n",
      exception.what());
    std::fflush(stderr);
    rviz_failed_ = true;
    manager_ = nullptr;
    rviz_node_.reset();
    displays_.clear();
    setRtabDebugText(
      "RViz render initialization failed:\n" + QString::fromLocal8Bit(exception.what()));
  } catch (...) {
    debugStep("ensureRvizInitialized: unknown exception");
    rviz_failed_ = true;
    manager_ = nullptr;
    rviz_node_.reset();
    displays_.clear();
    setRtabDebugText("RViz render initialization failed with an unknown exception.");
  }

  return false;
}

void DebugView::setStatusLabels(const QString & screen, bool crying)
{
  if (screen_label_) {
    screen_label_->setText("Current screen: " + screen);
  }
  if (face_state_label_) {
    face_state_label_->setText("Face state: " + QString(crying ? "Crying" : "Normal"));
  }
  if (fixed_frame_label_) {
    fixed_frame_label_->setText("Fixed frame: map");
  }
}

void DebugView::setRtabDebugText(const QString & text)
{
  if (rtab_text_) {
    rtab_text_->setPlainText(text);
  }
}

bool DebugView::eventFilter(QObject * watched, QEvent * event)
{
  if (watched != render_panel_) {
    return QWidget::eventFilter(watched, event);
  }

  switch (event->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
      Q_EMIT activityDetected();
      handleTouchEvent(static_cast<QTouchEvent *>(event));
      event->accept();
      return true;
    case QEvent::MouseButtonPress:
      Q_EMIT activityDetected();
      handleMousePress(static_cast<QMouseEvent *>(event));
      event->accept();
      return true;
    case QEvent::MouseMove:
      Q_EMIT activityDetected();
      handleMouseMove(static_cast<QMouseEvent *>(event));
      event->accept();
      return true;
    case QEvent::MouseButtonRelease:
      Q_EMIT activityDetected();
      handleMouseRelease(static_cast<QMouseEvent *>(event));
      event->accept();
      return true;
    case QEvent::Wheel:
      Q_EMIT activityDetected();
      handleWheel(static_cast<QWheelEvent *>(event));
      event->accept();
      return true;
    case QEvent::ContextMenu:
    case QEvent::MouseButtonDblClick:
    case QEvent::Gesture:
    case QEvent::GestureOverride:
      Q_EMIT activityDetected();
      event->accept();
      return true;
    default:
      return QWidget::eventFilter(watched, event);
  }
}

void DebugView::buildUi()
{
  debugStep("buildUi: begin");
  auto * root_layout = new QHBoxLayout(this);
  debugStepPtr("buildUi: root layout created", root_layout);
  root_layout->setContentsMargins(0, 0, 0, 0);
  root_layout->setSpacing(0);

  render_container_ = new QWidget(this);
  debugStepPtr("buildUi: render container created", render_container_);
  render_container_->setObjectName("RenderContainer");
  render_container_->setStyleSheet("QWidget#RenderContainer { background: #05070a; }");
  render_layout_ = new QVBoxLayout(render_container_);
  render_layout_->setContentsMargins(0, 0, 0, 0);
  render_layout_->setSpacing(0);

  render_placeholder_ = new QLabel("Debug view", render_container_);
  debugStepPtr("buildUi: render placeholder created", render_placeholder_);
  render_placeholder_->setAlignment(Qt::AlignCenter);
  render_placeholder_->setStyleSheet("color: #66717f; background: #05070a;");
  render_layout_->addWidget(render_placeholder_);

  root_layout->addWidget(render_container_, 1);

  side_panel_ = new QWidget(this);
  debugStepPtr("buildUi: side panel created", side_panel_);
  side_panel_->setObjectName("SidePanel");
  side_panel_->setFixedWidth(280);

  auto * side_layout = new QVBoxLayout(side_panel_);
  side_layout->setContentsMargins(14, 14, 14, 14);
  side_layout->setSpacing(10);

  auto * top_row = new QHBoxLayout();
  top_row->setContentsMargins(0, 0, 0, 0);
  top_row->setSpacing(8);

  auto * back_button = new QPushButton("Back to Eyes", side_panel_);
  back_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  connect(back_button, &QPushButton::clicked, this, &DebugView::backRequested);
  top_row->addWidget(back_button);

  side_panel_toggle_ = new QPushButton(side_panel_);
  side_panel_toggle_->setObjectName("PanelToggleButton");
  side_panel_toggle_->setIcon(style()->standardIcon(QStyle::SP_ArrowRight));
  side_panel_toggle_->setToolTip("Hide debug panel");
  side_panel_toggle_->setAccessibleName("Hide debug panel");
  side_panel_toggle_->setFixedSize(36, 36);
  connect(side_panel_toggle_, &QPushButton::clicked, this, [this]() {
    setSidePanelCollapsed(true);
  });
  top_row->addWidget(side_panel_toggle_);
  side_layout->addLayout(top_row);

  screen_label_ = new QLabel("Current screen: Debug", side_panel_);
  face_state_label_ = new QLabel("Face state: Normal", side_panel_);
  fixed_frame_label_ = new QLabel("Fixed frame: map", side_panel_);
  side_layout->addWidget(screen_label_);
  side_layout->addWidget(face_state_label_);
  side_layout->addWidget(fixed_frame_label_);

  auto * layer_box = new QGroupBox("Layers", side_panel_);
  auto * layer_layout = new QVBoxLayout(layer_box);
  layer_layout->setContentsMargins(8, 10, 8, 8);
  layer_layout->setSpacing(6);

  const QList<QPair<QString, QString>> layers = {
    {"Global Map", "global_map"},
    {"RTAB Grid Map", "rtab_grid_map"},
    {"SDV Trajectory", "sdv_trajectory"},
    {"PointCloud Ground Filtered", "filtered_cloud"},
    {"Available Paths", "available_paths"},
    {"Robot Footprint", "robot_footprint"},
    {"Global Planner", "global_planner"},
    {"RTAB Debug Info", "rtab_debug"}};

  for (const auto & layer : layers) {
    auto * checkbox = new QCheckBox(layer.first, layer_box);
    checkbox->setChecked(true);
    layer_checks_.insert(layer.second, checkbox);
    layer_layout->addWidget(checkbox);
  }

  layer_layout->addStretch(1);
  side_layout->addWidget(layer_box);

  rtab_text_ = new QPlainTextEdit(side_panel_);
  debugStepPtr("buildUi: rtab text panel created", rtab_text_);
  rtab_text_->setReadOnly(true);
  rtab_text_->setMinimumHeight(190);
  rtab_text_->setPlainText(
    "RTAB debug info waiting.\n"
    "Check /rtabmap/odom_local_map with:\n"
    "ros2 topic info /rtabmap/odom_local_map");
  side_layout->addWidget(rtab_text_, 1);

  if (layer_checks_.contains("rtab_debug")) {
    connect(layer_checks_.value("rtab_debug"), &QCheckBox::toggled, rtab_text_, &QWidget::setVisible);
  }

  root_layout->addWidget(side_panel_);

  restore_panel_ = new QWidget(this);
  restore_panel_->setObjectName("RestorePanel");
  restore_panel_->setFixedWidth(48);
  restore_panel_->setStyleSheet("QWidget#RestorePanel { background: #05070a; }");

  auto * restore_layout = new QVBoxLayout(restore_panel_);
  restore_layout->setContentsMargins(6, 12, 6, 0);
  restore_layout->setSpacing(0);

  side_panel_restore_ = new QPushButton(restore_panel_);
  side_panel_restore_->setObjectName("PanelRestoreButton");
  side_panel_restore_->setIcon(style()->standardIcon(QStyle::SP_ArrowLeft));
  side_panel_restore_->setToolTip("Show debug panel");
  side_panel_restore_->setAccessibleName("Show debug panel");
  side_panel_restore_->setFixedSize(36, 36);
  side_panel_restore_->setStyleSheet(
    "QPushButton#PanelRestoreButton {"
    "background: rgba(31, 36, 43, 220);"
    "border: 1px solid #3b444f;"
    "border-radius: 4px;"
    "padding: 0;"
    "}"
    "QPushButton#PanelRestoreButton:hover { background: #37414d; }");
  side_panel_restore_->hide();
  connect(side_panel_restore_, &QPushButton::clicked, this, [this]() {
    setSidePanelCollapsed(false);
  });
  restore_layout->addWidget(side_panel_restore_, 0, Qt::AlignTop | Qt::AlignHCenter);
  restore_layout->addStretch(1);
  restore_panel_->hide();
  root_layout->addWidget(restore_panel_);
  debugStep("buildUi: end");
}

void DebugView::setSidePanelCollapsed(bool collapsed)
{
  if (side_panel_collapsed_ == collapsed) {
    return;
  }

  side_panel_collapsed_ = collapsed;
  if (side_panel_) {
    side_panel_->setVisible(!collapsed);
  }
  if (side_panel_restore_) {
    side_panel_restore_->setVisible(collapsed);
  }
  if (restore_panel_) {
    restore_panel_->setVisible(collapsed);
  }
}

void DebugView::initializeRviz()
{
  debugStep("initializeRviz: begin");
  debugStep("initializeRviz: RenderSystem::get begin");
  RCLCPP_INFO(debug_node_->get_logger(), "Initializing RViz render system");
  rviz_rendering::RenderSystem::get();
  debugStep("initializeRviz: RenderSystem::get end");

  debugStep("initializeRviz: RosNodeAbstraction begin");
  RCLCPP_INFO(debug_node_->get_logger(), "Creating RViz ROS node abstraction");
  rviz_node_ =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("eyes_gui_rviz");
  debugStepPtr("initializeRviz: RosNodeAbstraction end", rviz_node_.get());

  debugStepPtr("initializeRviz: render_container before VisualizationFrame", render_container_);
  debugStepPtr("initializeRviz: qApp before VisualizationFrame", qApp);
  debugStep("initializeRviz: VisualizationFrame constructor begin");
  RCLCPP_INFO(debug_node_->get_logger(), "Creating RViz visualization frame");
  visualization_frame_ = new rviz_common::VisualizationFrame(rviz_node_, render_container_);
  visualization_frame_->setWindowFlags(Qt::Widget);
  visualization_frame_->setParent(render_container_);
  visualization_frame_->hide();
  debugStepPtr("initializeRviz: VisualizationFrame constructor end", visualization_frame_);
  debugStep("initializeRviz: VisualizationFrame setApp begin");
  visualization_frame_->setApp(qApp);
  debugStep("initializeRviz: VisualizationFrame setApp end");
  debugStep("initializeRviz: VisualizationFrame setSplashPath begin");
  visualization_frame_->setSplashPath("");
  debugStep("initializeRviz: VisualizationFrame setSplashPath end");
  debugStep("initializeRviz: VisualizationFrame setHelpPath skipped; default path retained");
  debugStep(
    "initializeRviz: VisualizationFrame setHideButtonVisibility skipped; chrome hidden after init");

  debugStep("initializeRviz: VisualizationFrame initialize begin");
  RCLCPP_INFO(debug_node_->get_logger(), "Initializing RViz visualization frame");
  visualization_frame_->initialize(rviz_node_, "");
  debugStep("initializeRviz: VisualizationFrame initialize end");
  visualization_frame_->setWindowFlags(Qt::Widget);
  visualization_frame_->setParent(render_container_);
  visualization_frame_->hide();
  debugStep("initializeRviz: getManager begin");
  manager_ = visualization_frame_->getManager();
  debugStepPtr("initializeRviz: getManager end", manager_);
  debugStep("initializeRviz: getRenderPanel begin");
  render_panel_ = manager_ ? manager_->getRenderPanel() : nullptr;
  debugStepPtr("initializeRviz: getRenderPanel end", render_panel_);
  if (!manager_ || !render_panel_) {
    throw std::runtime_error("VisualizationFrame did not create a manager/render panel");
  }

  debugStep("initializeRviz: hide RViz chrome begin");
  visualization_frame_->setHideButtonVisibility(false);
  visualization_frame_->menuBar()->hide();
  visualization_frame_->statusBar()->hide();
  for (auto * toolbar : visualization_frame_->findChildren<QToolBar *>()) {
    toolbar->hide();
  }
  for (auto * tool_button : visualization_frame_->findChildren<QToolButton *>()) {
    tool_button->hide();
  }
  for (auto * dock : visualization_frame_->findChildren<QDockWidget *>()) {
    dock->hide();
  }
  visualization_frame_->setStyleSheet(
    "rviz_common--VisualizationFrame, QMainWindow { background: #05070a; }"
    "QSplitter::handle { background: #05070a; border: 0; }"
    "QDockWidget { background: #05070a; border: 0; }");
  debugStep("initializeRviz: hide RViz chrome end");

  debugStep("initializeRviz: insert VisualizationFrame in layout begin");
  if (render_placeholder_) {
    render_placeholder_->hide();
    render_layout_->removeWidget(render_placeholder_);
    render_placeholder_->deleteLater();
    render_placeholder_ = nullptr;
  }
  render_layout_->addWidget(visualization_frame_);
  visualization_frame_->show();
  debugStep("initializeRviz: insert VisualizationFrame in layout end");

  debugStep("initializeRviz: configure RenderPanel events begin");
  render_panel_->setAttribute(Qt::WA_AcceptTouchEvents, true);
  render_panel_->setContextMenuPolicy(Qt::NoContextMenu);
  render_panel_->installEventFilter(this);
  debugStep("initializeRviz: configure RenderPanel events end");

  debugStep("initializeRviz: setFixedFrame begin");
  manager_->setFixedFrame("map");
  debugStep("initializeRviz: setFixedFrame end");

  debugStep("initializeRviz: set view controller begin");
  if (manager_->getViewManager()) {
    manager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/XYOrbit");
  }
  debugStep("initializeRviz: set view controller end");
  debugStep("initializeRviz: reset view controller begin");
  if (render_panel_->getViewController()) {
    render_panel_->getViewController()->reset();
    render_panel_->getViewController()->lookAt(0.0f, 0.0f, 0.0f);
  }
  debugStep("initializeRviz: reset view controller end");

  debugStep("initializeRviz: set background color begin");
  Ogre::ColourValue background(0.02f, 0.025f, 0.03f, 1.0f);
  rviz_rendering::RenderWindowOgreAdapter::setBackgroundColor(
    render_panel_->getRenderWindow(), &background);
  debugStep("initializeRviz: set background color end");

  debugStep("initializeRviz: createDisplays begin");
  RCLCPP_INFO(debug_node_->get_logger(), "Creating RViz displays");
  createDisplays();
  debugStep("initializeRviz: createDisplays end");
  debugStep("initializeRviz: end");
}

void DebugView::createRenderPanel()
{
  if (render_panel_) {
    return;
  }

  render_panel_ = new rviz_common::RenderPanel(render_container_);
  render_panel_->setObjectName("EmbeddedRvizRenderPanel");
  render_panel_->setAttribute(Qt::WA_AcceptTouchEvents, true);
  render_panel_->setContextMenuPolicy(Qt::NoContextMenu);
  render_panel_->installEventFilter(this);

  if (render_placeholder_) {
    render_placeholder_->hide();
    render_layout_->removeWidget(render_placeholder_);
    render_placeholder_->deleteLater();
    render_placeholder_ = nullptr;
  }

  render_layout_->addWidget(render_panel_);
}

void DebugView::createDisplays()
{
  debugStep("createDisplays: begin");
  debugStep("createDisplays: grid begin");
  configureGrid(addDisplay("grid", "rviz_default_plugins/Grid", "Grid", true));
  debugStep("createDisplays: grid end");
  debugStep("createDisplays: global map begin");
  configureMap(
    addDisplay("global_map", "rviz_default_plugins/Map", "Global Map", true),
    "/global_planner_occupancy_grid",
    "/global_planner_occupancy_grid_updates");
  debugStep("createDisplays: global map end");
  debugStep("createDisplays: sdv trajectory begin");
  configurePath(
    addDisplay("sdv_trajectory", "rviz_default_plugins/Path", "SDV Trajectory", true),
    "/sdv_trajectory");
  debugStep("createDisplays: sdv trajectory end");
  debugStep("createDisplays: filtered cloud begin");
  configurePointCloud(
    addDisplay("filtered_cloud", "rviz_default_plugins/PointCloud2", "PointCloud Ground Filtered", true),
    "/points_rotated_notground",
    0.01);
  debugStep("createDisplays: filtered cloud end");
  debugStep("createDisplays: rtab grid map begin");
  configureMap(
    addDisplay("rtab_grid_map", "rviz_default_plugins/Map", "RTAB Grid Map", true),
    "/rtabmap/grid_prob_map",
    "/rtabmap/grid_prob_map_updates");
  debugStep("createDisplays: rtab grid map end");
  debugStep("createDisplays: global planner begin");
  configureMarker(
    addDisplay("global_planner", "rviz_default_plugins/MarkerArray", "Global Planner", true),
    "/global_planner");
  debugStep("createDisplays: global planner end");
  debugStep("createDisplays: available paths begin");
  configureMarker(
    addDisplay("available_paths", "rviz_default_plugins/MarkerArray", "Available Paths", true),
    "/all_available_paths");
  debugStep("createDisplays: available paths end");
  debugStep("createDisplays: robot footprint begin");
  configureMarker(
    addDisplay("robot_footprint", "rviz_default_plugins/Marker", "Robot Footprint", true),
    "/robot_footprint_polygon");
  debugStep("createDisplays: robot footprint end");

  debugStep("createDisplays: connect checkboxes begin");
  for (auto it = layer_checks_.cbegin(); it != layer_checks_.cend(); ++it) {
    if (it.key() == "rtab_debug") {
      continue;
    }
    const auto display = displays_.value(it.key(), nullptr);
    if (!display) {
      continue;
    }
    connect(it.value(), &QCheckBox::toggled, this, [display](bool checked) {
      display->setEnabled(checked);
    });
  }
  debugStep("createDisplays: connect checkboxes end");
  debugStep("createDisplays: end");
}

rviz_common::Display * DebugView::addDisplay(
  const QString & key,
  const QString & class_id,
  const QString & display_name,
  bool enabled)
{
  if (!manager_) {
    return nullptr;
  }

  rviz_common::Display * display = manager_->createDisplay(class_id, display_name, enabled);
  std::fprintf(
    stderr,
    "[robot_face_debug_ui][debug_view] addDisplay key=%s class=%s name=%s display=%p\n",
    key.toLocal8Bit().constData(),
    class_id.toLocal8Bit().constData(),
    display_name.toLocal8Bit().constData(),
    static_cast<void *>(display));
  std::fflush(stderr);
  if (display) {
    displays_.insert(key, display);
  }
  return display;
}

void DebugView::configureGrid(rviz_common::Display * display)
{
  if (!display) {
    return;
  }

  setDisplayProperty(display, {"Plane"}, "XY");
  setDisplayProperty(display, {"Cell Size"}, 1.0);
  setDisplayProperty(display, {"Color"}, QColor(160, 160, 164));
  setDisplayProperty(display, {"Alpha"}, 0.5);
  setDisplayProperty(display, {"Line Style", "Line Width"}, 0.03);
  setDisplayProperty(display, {"Line Style", "Value"}, "Lines");
  setDisplayProperty(display, {"Normal Cell Count"}, 0);
  setDisplayProperty(display, {"Plane Cell Count"}, 10);
}

void DebugView::configureMap(
  rviz_common::Display * display,
  const QString & topic,
  const QString & update_topic)
{
  if (!display) {
    return;
  }

  setAnyDisplayProperty(display, {{"Topic"}, {"Map Topic"}}, normalizeTopic(topic));
  setAnyDisplayProperty(display, {{"Update Topic"}, {"Topic", "Update Topic"}}, normalizeTopic(update_topic));
  setDisplayProperty(display, {"Alpha"}, 0.7);
  setDisplayProperty(display, {"Binary representation"}, false);
  setDisplayProperty(display, {"Binary threshold"}, 100);
  setDisplayProperty(display, {"Draw Behind"}, false);
  setDisplayProperty(display, {"Use Timestamp"}, false);
  setAnyDisplayProperty(display, {{"Color Scheme"}, {"Color scheme"}}, "map");
  setDisplayProperty(display, {"Topic", "Depth"}, 5);
  setDisplayProperty(display, {"Topic", "Filter size"}, 10);
  setDisplayProperty(display, {"Topic", "History Policy"}, "Keep Last");
  setAnyDisplayProperty(display, {{"Topic", "Reliability Policy"}, {"Reliability Policy"}}, "Reliable");
  setAnyDisplayProperty(display, {{"Topic", "Durability Policy"}, {"Durability Policy"}}, "Volatile");
  setDisplayProperty(display, {"Update Topic", "Depth"}, 5);
  setDisplayProperty(display, {"Update Topic", "History Policy"}, "Keep Last");
  setDisplayProperty(display, {"Update Topic", "Reliability Policy"}, "Reliable");
  setDisplayProperty(display, {"Update Topic", "Durability Policy"}, "Volatile");
}

void DebugView::configurePath(rviz_common::Display * display, const QString & topic)
{
  if (!display) {
    return;
  }

  setAnyDisplayProperty(display, {{"Topic"}, {"Path Topic"}}, normalizeTopic(topic));
  setDisplayProperty(display, {"Alpha"}, 1.0);
  setDisplayProperty(display, {"Buffer Length"}, 1);
  setDisplayProperty(display, {"Color"}, QColor(25, 255, 0));
  setDisplayProperty(display, {"Line Style"}, "Billboards");
  setAnyDisplayProperty(display, {{"Line Width"}, {"Line Style", "Line Width"}}, 0.1);
  setDisplayProperty(display, {"Offset", "Z"}, 2.0);
  setDisplayProperty(display, {"Pose Style"}, "None");
  setDisplayProperty(display, {"Topic", "Depth"}, 5);
  setDisplayProperty(display, {"Topic", "Filter size"}, 10);
  setDisplayProperty(display, {"Topic", "History Policy"}, "Keep Last");
  setDisplayProperty(display, {"Topic", "Reliability Policy"}, "Reliable");
  setDisplayProperty(display, {"Topic", "Durability Policy"}, "Volatile");
}

void DebugView::configurePointCloud(
  rviz_common::Display * display,
  const QString & topic,
  double size_m)
{
  if (!display) {
    return;
  }

  setAnyDisplayProperty(display, {{"Topic"}, {"PointCloud2 Topic"}}, normalizeTopic(topic));
  setDisplayProperty(display, {"Alpha"}, 1.0);
  setDisplayProperty(display, {"Style"}, "Points");
  setDisplayProperty(display, {"Size (Pixels)"}, 3);
  setAnyDisplayProperty(display, {{"Size (m)"}, {"Size"}}, size_m);
  setDisplayProperty(display, {"Color Transformer"}, "Intensity");
  setDisplayProperty(display, {"Channel Name"}, "intensity");
  setDisplayProperty(display, {"Autocompute Intensity Bounds"}, true);
  setDisplayProperty(display, {"Min Intensity"}, 0);
  setDisplayProperty(display, {"Max Intensity"}, 4096);
  setDisplayProperty(display, {"Decay Time"}, 0.0);
  setDisplayProperty(display, {"Position Transformer"}, "XYZ");
  setDisplayProperty(display, {"Selectable"}, true);
  setDisplayProperty(display, {"Use Fixed Frame"}, true);
  setAnyDisplayProperty(display, {{"Use rainbow"}, {"Use Rainbow"}}, true);
  setDisplayProperty(display, {"Invert Rainbow"}, false);
  setDisplayProperty(display, {"Topic", "Depth"}, 5);
  setDisplayProperty(display, {"Topic", "History Policy"}, "Keep Last");
  setAnyDisplayProperty(display, {{"Topic", "Reliability Policy"}, {"Reliability Policy"}}, "Reliable");
  setAnyDisplayProperty(
    display,
    {{"Topic", "Durability Policy"}, {"Durability Policy"}},
    "Transient Local");
}

void DebugView::configureMarker(rviz_common::Display * display, const QString & topic)
{
  if (!display) {
    return;
  }

  setAnyDisplayProperty(
    display,
    {{"Topic"}, {"Marker Topic"}, {"MarkerArray Topic"}, {"Marker Topic", "Topic"}},
    normalizeTopic(topic));
  setDisplayProperty(display, {"Topic", "Depth"}, 5);
  setDisplayProperty(display, {"Topic", "History Policy"}, "Keep Last");
  setDisplayProperty(display, {"Topic", "Reliability Policy"}, "Reliable");
  setDisplayProperty(display, {"Topic", "Durability Policy"}, "Volatile");
  setDisplayProperty(display, {"Topic", "Filter size"}, 10);
}

bool DebugView::setDisplayProperty(
  rviz_common::Display * display,
  const QStringList & path,
  const QVariant & value)
{
  if (!display || path.isEmpty()) {
    return false;
  }

  rviz_common::properties::Property * property = display;
  for (const auto & name : path) {
    property = property->subProp(name);
    if (!property) {
      return false;
    }
  }

  return property->setValue(value);
}

bool DebugView::setAnyDisplayProperty(
  rviz_common::Display * display,
  const QList<QStringList> & paths,
  const QVariant & value)
{
  for (const auto & path : paths) {
    if (setDisplayProperty(display, path, value)) {
      return true;
    }
  }
  return false;
}

void DebugView::setupRtabSubscriptions()
{
#ifdef HAVE_RTABMAP_MSGS
  const auto qos = rclcpp::QoS(1).best_effort();

  rtab_info_sub_ = debug_node_->create_subscription<rtabmap_msgs::msg::Info>(
    "/rtabmap/info",
    qos,
    [this](rtabmap_msgs::msg::Info::ConstSharedPtr msg)
    {
      QStringList lines;
      lines << "RTAB Info";
      lines << "stamp: " + stampToText(msg->header.stamp);
      lines << QString("loop closure id: %1").arg(msg->loop_closure_id);
      lines << QString("proximity detection id: %1").arg(msg->proximity_detection_id);
      lines << QString("landmark id: %1").arg(msg->landmark_id);
      lines << QString("wm state: %1").arg(msg->wm_state.size());
      lines << QString("local path: %1").arg(msg->local_path.size());
      lines << QString("stats: %1").arg(msg->stats_keys.size());

      const int stats_to_show = std::min<int>(msg->stats_keys.size(), 8);
      for (int i = 0; i < stats_to_show && i < static_cast<int>(msg->stats_values.size()); ++i) {
        lines << QString("%1: %2")
          .arg(QString::fromStdString(msg->stats_keys.at(i)))
          .arg(msg->stats_values.at(i), 0, 'f', 3);
      }

      const QString text = lines.join('\n');
      QMetaObject::invokeMethod(this, [this, text]() {
        setRtabDebugText(text);
      }, Qt::QueuedConnection);
    });

  rtab_map_graph_sub_ = debug_node_->create_subscription<rtabmap_msgs::msg::MapGraph>(
    "/rtabmap/mapGraph",
    qos,
    [this](rtabmap_msgs::msg::MapGraph::ConstSharedPtr msg)
    {
      const QString text = QString(
        "RTAB MapGraph\n"
        "stamp: %1\n"
        "poses: %2\n"
        "pose ids: %3\n"
        "links: %4\n"
        "TODO: draw graph nodes/edges as markers in a later phase.")
        .arg(stampToText(msg->header.stamp))
        .arg(msg->poses.size())
        .arg(msg->poses_id.size())
        .arg(msg->links.size());

      QMetaObject::invokeMethod(this, [this, text]() {
        setRtabDebugText(text);
      }, Qt::QueuedConnection);
    });

  rtab_map_data_sub_ = debug_node_->create_subscription<rtabmap_msgs::msg::MapData>(
    "/rtabmap/mapData",
    qos,
    [this](rtabmap_msgs::msg::MapData::ConstSharedPtr msg)
    {
      const QString text = QString(
        "RTAB MapData\n"
        "stamp: %1\n"
        "graph poses: %2\n"
        "graph links: %3\n"
        "nodes/signatures: %4\n"
        "TODO: full MapData rendering is intentionally not implemented in this minimal view.")
        .arg(stampToText(msg->header.stamp))
        .arg(msg->graph.poses.size())
        .arg(msg->graph.links.size())
        .arg(msg->nodes.size());

      QMetaObject::invokeMethod(this, [this, text]() {
        setRtabDebugText(text);
      }, Qt::QueuedConnection);
    });
#else
  rtab_text_->setPlainText(
    "rtabmap_msgs was not found at build time.\n"
    "Install ros-jazzy-rtabmap-msgs and rebuild to enable /rtabmap/info, "
    "/rtabmap/mapGraph, and /rtabmap/mapData summaries.");
#endif
}

void DebugView::handleTouchEvent(QTouchEvent * event)
{
  const auto points = event->touchPoints();
  if (points.isEmpty()) {
    last_touch_count_ = 0;
    last_pinch_distance_ = 0.0;
    return;
  }

  QPointF center;
  for (const auto & point : points) {
    center += point.pos();
  }
  center /= points.size();

  if (points.size() == 1) {
    if (last_touch_count_ == 1) {
      rotateCamera(center - last_touch_center_);
    }
    last_pinch_distance_ = 0.0;
  } else {
    if (last_touch_count_ >= 2) {
      panCamera(center - last_touch_center_);
    }

    const QPointF delta = points.at(0).pos() - points.at(1).pos();
    const qreal distance = std::sqrt(QPointF::dotProduct(delta, delta));
    if (last_pinch_distance_ > 0.0) {
      zoomCamera(static_cast<float>((distance - last_pinch_distance_) * kPinchZoomScale));
    }
    last_pinch_distance_ = distance;
  }

  last_touch_center_ = center;
  last_touch_count_ = points.size();
}

void DebugView::handleMousePress(QMouseEvent * event)
{
  mouse_dragging_ = true;
  mouse_button_ = event->button();
  last_mouse_pos_ = event->pos();
}

void DebugView::handleMouseMove(QMouseEvent * event)
{
  if (!mouse_dragging_) {
    return;
  }

  const QPointF delta = event->pos() - last_mouse_pos_;
  if (mouse_button_ == Qt::LeftButton) {
    rotateCamera(delta);
  } else if (mouse_button_ == Qt::MiddleButton || mouse_button_ == Qt::RightButton) {
    panCamera(delta);
  }
  last_mouse_pos_ = event->pos();
}

void DebugView::handleMouseRelease(QMouseEvent * event)
{
  Q_UNUSED(event);
  mouse_dragging_ = false;
  mouse_button_ = Qt::NoButton;
}

void DebugView::handleWheel(QWheelEvent * event)
{
  zoomCamera(static_cast<float>(event->angleDelta().y() * kWheelZoomScale));
}

void DebugView::rotateCamera(const QPointF & delta)
{
  auto * orbit = dynamic_cast<rviz_default_plugins::view_controllers::OrbitViewController *>(
    render_panel_->getViewController());
  if (!orbit) {
    return;
  }

  orbit->yaw(static_cast<float>(-delta.x() * kRotateScale));
  orbit->pitch(static_cast<float>(-delta.y() * kRotateScale));
  if (manager_) {
    manager_->queueRender();
  }
}

void DebugView::panCamera(const QPointF & delta)
{
  auto * orbit = dynamic_cast<rviz_default_plugins::view_controllers::OrbitViewController *>(
    render_panel_->getViewController());
  if (!orbit) {
    return;
  }

  orbit->move(
    static_cast<float>(-delta.x() * kPanScale),
    static_cast<float>(delta.y() * kPanScale),
    0.0f);
  if (manager_) {
    manager_->queueRender();
  }
}

void DebugView::zoomCamera(float amount)
{
  auto * orbit = dynamic_cast<rviz_default_plugins::view_controllers::OrbitViewController *>(
    render_panel_->getViewController());
  if (!orbit) {
    return;
  }

  orbit->zoom(amount);
  if (manager_) {
    manager_->queueRender();
  }
}

}  // namespace eyes_gui
