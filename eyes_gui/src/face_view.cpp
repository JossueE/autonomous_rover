#include "eyes_gui/face_view.hpp"

#include <algorithm>
#include <stdexcept>

#include <QEvent>
#include <QMouseEvent>
#include <QRandomGenerator>
#include <QResizeEvent>
#include <QTouchEvent>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace eyes_gui
{

namespace
{

constexpr int kMinBlinkDelayMs = 2200;
constexpr int kMaxBlinkDelayMs = 5200;

}  // namespace

FaceView::FaceView(QWidget * parent)
: QWidget(parent),
  image_label_(new QLabel(this)),
  blink_index_(0),
  is_crying_(false),
  is_blinking_(false)
{
  setObjectName("FaceView");
  setAttribute(Qt::WA_AcceptTouchEvents, true);
  setFocusPolicy(Qt::StrongFocus);
  setStyleSheet("QWidget#FaceView { background: black; } QLabel { background: black; }");

  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(image_label_);

  loadPixmaps();
  showOpen();

  auto_blink_timer_.setSingleShot(true);
  connect(&auto_blink_timer_, &QTimer::timeout, this, &FaceView::blink);
  scheduleNextBlink();
}

bool FaceView::isCrying() const
{
  return is_crying_;
}

void FaceView::showOpen()
{
  setCurrentPixmap(is_crying_ ? "Abiertos_llorando.PNG" : "Abiertos.PNG");
}

void FaceView::blink()
{
  if (is_blinking_) {
    return;
  }

  auto_blink_timer_.stop();
  is_blinking_ = true;
  blink_index_ = 0;

  if (is_crying_) {
    blink_sequence_ = QStringList{
      "Cerrando_llorando.PNG",
      "Cerrados_llorando.PNG",
      "Cerrando_llorando.PNG",
      "Abiertos_llorando.PNG"};
  } else {
    blink_sequence_ = QStringList{
      "Cerrando.PNG",
      "Cerrados.PNG",
      "Cerrando.PNG",
      "Abiertos.PNG"};
  }

  blink_durations_ms_ = {90, 120, 90, 1};
  showBlinkFrame();
}

void FaceView::setCrying(bool enabled)
{
  if (is_crying_ == enabled) {
    return;
  }

  is_crying_ = enabled;
  if (!is_blinking_) {
    showOpen();
  }
}

void FaceView::recover()
{
  setCrying(false);
}

bool FaceView::event(QEvent * event)
{
  if (event->type() == QEvent::TouchBegin) {
    Q_EMIT debugRequested();
    event->accept();
    return true;
  }

  return QWidget::event(event);
}

void FaceView::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    Q_EMIT debugRequested();
    event->accept();
    return;
  }

  QWidget::mousePressEvent(event);
}

void FaceView::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updateScaledPixmap();
}

void FaceView::loadPixmaps()
{
  const QStringList names = {
    "Abiertos.PNG",
    "Cerrando.PNG",
    "Cerrados.PNG",
    "Abiertos_llorando.PNG",
    "Cerrando_llorando.PNG",
    "Cerrados_llorando.PNG"};

  QString image_dir;
  try {
    image_dir = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("eyes_gui")) + "/images";
  } catch (const std::exception & exception) {
    image_label_->setText(QString("Package share not found: %1").arg(exception.what()));
    return;
  }

  for (const auto & name : names) {
    QPixmap pixmap(image_dir + "/" + name);
    if (pixmap.isNull()) {
      image_label_->setText(QString("Missing image: %1/%2").arg(image_dir, name));
      continue;
    }
    pixmaps_.insert(name, pixmap);
  }
}

void FaceView::scheduleNextBlink()
{
  const int delay = QRandomGenerator::global()->bounded(kMinBlinkDelayMs, kMaxBlinkDelayMs + 1);
  auto_blink_timer_.start(delay);
}

void FaceView::showBlinkFrame()
{
  if (blink_index_ >= blink_sequence_.size()) {
    is_blinking_ = false;
    showOpen();
    scheduleNextBlink();
    return;
  }

  setCurrentPixmap(blink_sequence_.at(blink_index_));
  const int duration = blink_durations_ms_.value(blink_index_, 90);
  ++blink_index_;
  QTimer::singleShot(duration, this, &FaceView::showBlinkFrame);
}

void FaceView::setCurrentPixmap(const QString & name)
{
  if (!pixmaps_.contains(name)) {
    return;
  }

  current_pixmap_ = name;
  updateScaledPixmap();
}

void FaceView::updateScaledPixmap()
{
  if (current_pixmap_.isEmpty() || !pixmaps_.contains(current_pixmap_)) {
    return;
  }

  const QPixmap & source = pixmaps_.value(current_pixmap_);
  const QSize target_size = image_label_->size();
  if (!target_size.isValid() || target_size.isEmpty()) {
    return;
  }

  image_label_->setPixmap(
    source.scaled(target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

}  // namespace eyes_gui
