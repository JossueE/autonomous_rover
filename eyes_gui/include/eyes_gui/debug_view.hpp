#ifndef EYES_GUI__DEBUG_VIEW_HPP_
#define EYES_GUI__DEBUG_VIEW_HPP_

#include <memory>

#include <QCheckBox>
#include <QLabel>
#include <QList>
#include <QMap>
#include <QPlainTextEdit>
#include <QPoint>
#include <QPointF>
#include <QPushButton>
#include <QStringList>
#include <QVariant>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef HAVE_RTABMAP_MSGS
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap_msgs/msg/map_graph.hpp>
#endif

namespace eyes_gui
{

class DebugView : public QWidget
{
  Q_OBJECT

public:
  explicit DebugView(QWidget * parent = nullptr);
  ~DebugView() override;

  rclcpp::Node::SharedPtr debugNode() const;
  rviz_common::RenderPanel * renderPanel() const;
  bool ensureRvizInitialized();

public Q_SLOTS:
  void setStatusLabels(const QString & screen, bool crying);
  void setRtabDebugText(const QString & text);

Q_SIGNALS:
  void backRequested();
  void activityDetected();

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  void buildUi();
  void setSidePanelCollapsed(bool collapsed);
  void createRenderPanel();
  void initializeRviz();
  void createDisplays();
  rviz_common::Display * addDisplay(
    const QString & key,
    const QString & class_id,
    const QString & display_name,
    bool enabled);
  void configureGrid(rviz_common::Display * display);
  void configureMap(
    rviz_common::Display * display,
    const QString & topic,
    const QString & update_topic);
  void configurePath(rviz_common::Display * display, const QString & topic);
  void configurePointCloud(
    rviz_common::Display * display,
    const QString & topic,
    double size_m);
  void configureMarker(rviz_common::Display * display, const QString & topic);
  void configureRtabInfoDisplay(rviz_common::Display * display);
  void configureRtabMapCloud(rviz_common::Display * display);
  void configureRtabMapGraph(rviz_common::Display * display);
  bool setDisplayProperty(
    rviz_common::Display * display,
    const QStringList & path,
    const QVariant & value);
  bool setAnyDisplayProperty(
    rviz_common::Display * display,
    const QList<QStringList> & paths,
    const QVariant & value);
  void setupAvailablePathsOffsetRelay();
  void setupRtabSubscriptions();
  void handleTouchEvent(QTouchEvent * event);
  void handleMousePress(QMouseEvent * event);
  void handleMouseMove(QMouseEvent * event);
  void handleMouseRelease(QMouseEvent * event);
  void handleWheel(QWheelEvent * event);
  void rotateCamera(const QPointF & delta);
  void panCamera(const QPointF & delta);
  void zoomCamera(float amount);

  rviz_common::RenderPanel * render_panel_;
  rviz_common::VisualizationFrame * visualization_frame_;
  QWidget * render_container_;
  QVBoxLayout * render_layout_;
  QLabel * render_placeholder_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node_;
  rviz_common::VisualizationManager * manager_;
  rclcpp::Node::SharedPtr debug_node_;

  QWidget * side_panel_;
  QWidget * restore_panel_;
  QPushButton * side_panel_toggle_;
  QPushButton * side_panel_restore_;
  QLabel * screen_label_;
  QLabel * face_state_label_;
  QLabel * fixed_frame_label_;
  QPlainTextEdit * rtab_text_;

  QMap<QString, rviz_common::Display *> displays_;
  QMap<QString, QCheckBox *> layer_checks_;

  QPointF last_touch_center_;
  qreal last_pinch_distance_;
  int last_touch_count_;
  QPoint last_mouse_pos_;
  bool mouse_dragging_;
  Qt::MouseButton mouse_button_;
  bool rviz_initialized_;
  bool rviz_failed_;
  bool side_panel_collapsed_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr available_paths_offset_pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr available_paths_sub_;

#ifdef HAVE_RTABMAP_MSGS
  rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr rtab_info_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapGraph>::SharedPtr rtab_map_graph_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr rtab_map_data_sub_;
#endif
};

}  // namespace eyes_gui

#endif  // EYES_GUI__DEBUG_VIEW_HPP_
