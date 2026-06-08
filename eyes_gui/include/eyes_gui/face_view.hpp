#ifndef EYES_GUI__FACE_VIEW_HPP_
#define EYES_GUI__FACE_VIEW_HPP_

#include <QHash>
#include <QLabel>
#include <QList>
#include <QPixmap>
#include <QStringList>
#include <QTimer>
#include <QWidget>

namespace eyes_gui
{

class FaceView : public QWidget
{
  Q_OBJECT

public:
  explicit FaceView(QWidget * parent = nullptr);

  bool isCrying() const;

public Q_SLOTS:
  void showOpen();
  void blink();
  void setCrying(bool enabled);
  void recover();

Q_SIGNALS:
  void debugRequested();

protected:
  bool event(QEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;

private:
  void loadPixmaps();
  void scheduleNextBlink();
  void showBlinkFrame();
  void setCurrentPixmap(const QString & name);
  void updateScaledPixmap();

  QLabel * image_label_;
  QHash<QString, QPixmap> pixmaps_;
  QString current_pixmap_;
  QTimer auto_blink_timer_;
  QStringList blink_sequence_;
  QList<int> blink_durations_ms_;
  int blink_index_;
  bool is_crying_;
  bool is_blinking_;
};

}  // namespace eyes_gui

#endif  // EYES_GUI__FACE_VIEW_HPP_
