#ifndef EYES_GUI__IDLE_EVENT_FILTER_HPP_
#define EYES_GUI__IDLE_EVENT_FILTER_HPP_

#include <QObject>

namespace eyes_gui
{

class IdleEventFilter : public QObject
{
  Q_OBJECT

public:
  explicit IdleEventFilter(QObject * parent = nullptr);

  void setEnabled(bool enabled);
  bool isEnabled() const;

Q_SIGNALS:
  void activityDetected();

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  bool enabled_;
};

}  // namespace eyes_gui

#endif  // EYES_GUI__IDLE_EVENT_FILTER_HPP_
