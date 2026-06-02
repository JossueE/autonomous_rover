#include "eyes_gui/idle_event_filter.hpp"

#include <QEvent>

namespace eyes_gui
{

IdleEventFilter::IdleEventFilter(QObject * parent)
: QObject(parent),
  enabled_(false)
{
}

void IdleEventFilter::setEnabled(bool enabled)
{
  enabled_ = enabled;
}

bool IdleEventFilter::isEnabled() const
{
  return enabled_;
}

bool IdleEventFilter::eventFilter(QObject * watched, QEvent * event)
{
  if (!enabled_) {
    return QObject::eventFilter(watched, event);
  }

  switch (event->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
    case QEvent::Gesture:
    case QEvent::GestureOverride:
    case QEvent::MouseMove:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::Wheel:
    case QEvent::KeyPress:
      Q_EMIT activityDetected();
      break;
    default:
      break;
  }

  return QObject::eventFilter(watched, event);
}

}  // namespace eyes_gui
