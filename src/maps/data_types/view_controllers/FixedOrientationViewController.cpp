
#include "FixedOrientationViewController.h"

#include <OgreViewport.h>
#include <QsLog/QsLog.h>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/viewport_mouse_event.h>

namespace gcs {
FixedOrientationViewController::FixedOrientationViewController()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

FixedOrientationViewController::~FixedOrientationViewController()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void FixedOrientationViewController::reset()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    target_frame_property_->setValue(QString("map"));
    scale_property_->setFloat(60);
    x_property_->setFloat(0);
    y_property_->setFloat(0);

    /// \note Rotate -90º
    angle_property_->setFloat(-1.0f * (M_PI / 2.0));
}

void FixedOrientationViewController::handleMouseEvent(rviz::ViewportMouseEvent& event)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    bool moved = false;

    int32_t diff_x = 0;
    int32_t diff_y = 0;

    if (event.type == QEvent::MouseButtonPress) {
        dragging_ = true;
    } else if (event.type == QEvent::MouseButtonRelease) {
        dragging_ = false;
    } else if (dragging_ && event.type == QEvent::MouseMove) {
        diff_x = event.x - event.last_x;
        diff_y = event.y - event.last_y;
        moved  = true;
    }

    if (event.left() && !event.control() && !_movingWaypoint) {
        setCursor(MoveXY);
        float scale = scale_property_->getFloat();
        move(-diff_x / scale, diff_y / scale);
    } else if (event.left() && event.control() && !_movingWaypoint) {
        setCursor(Rotate2D);
        angle_property_->add(diff_x * 0.005);
        orientCamera();
    } else {
        setCursor(Default);
        // setCursor(!_movingWaypoint ? MoveXY : Default);
    }

    if (event.wheel_delta != 0) {
        setCursor(Zoom);
        int diff = event.wheel_delta;
        scale_property_->multiply(1.0 - (-diff) * 0.001);

        moved = true;
    }

    if (moved) {
        context_->queueRender();
        Q_EMIT configChanged();
    }
}

bool& FixedOrientationViewController::movingWaypoint()
{
    return _movingWaypoint;
}
} // namespace gcs

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gcs::FixedOrientationViewController, rviz::ViewController)