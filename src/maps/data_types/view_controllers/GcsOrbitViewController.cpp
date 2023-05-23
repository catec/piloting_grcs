
#include "GcsOrbitViewController.h"

#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <QsLog/QsLog.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/float_property.h>
#include <rviz/viewport_mouse_event.h>

namespace gcs {
GcsOrbitViewController::GcsOrbitViewController()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

GcsOrbitViewController::~GcsOrbitViewController()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void GcsOrbitViewController::handleMouseEvent(rviz::ViewportMouseEvent& event)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    float distance = distance_property_->getFloat();
    updateFocalShapeSize();

    int32_t diff_x = 0;
    int32_t diff_y = 0;

    if (event.type == QEvent::MouseButtonPress) {
        focal_shape_->getRootNode()->setVisible(true);
        dragging_ = true;
    } else if (event.type == QEvent::MouseButtonRelease) {
        focal_shape_->getRootNode()->setVisible(false);
        dragging_ = false;
    } else if (dragging_ && event.type == QEvent::MouseMove) {
        diff_x = event.x - event.last_x;
        diff_y = event.y - event.last_y;
    }

    // regular left-button drag
    if (event.left() && !event.shift()) {
        setCursor(Rotate3D);
        yaw(diff_x * 0.005);
        pitch(-diff_y * 0.005);
    }
    // middle or shift-left drag
    else if (event.middle() || (event.shift() && event.left())) {
        setCursor(MoveXY);
        float fovY = camera_->getFOVy().valueRadians();
        float fovX = 2.0f * std::atan(std::tan(fovY / 2.0f) * camera_->getAspectRatio());

        int width  = camera_->getViewport()->getActualWidth();
        int height = camera_->getViewport()->getActualHeight();

        move(-((float)diff_x / (float)width) * distance * std::tan(fovX / 2.0f) * 2.0f,
             ((float)diff_y / (float)height) * distance * std::tan(fovY / 2.0f) * 2.0f,
             0.0f);
    } else {
        setCursor(event.shift() ? MoveXY : Rotate3D);
    }

    if (event.wheel_delta != 0) {
        int diff = event.wheel_delta;
        if (event.shift()) {
            move(0, 0, -diff * 0.001 * distance);
        } else {
            zoom(diff * 0.001 * distance);
        }
    }

    context_->queueRender();
}

} // namespace gcs

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gcs::GcsOrbitViewController, rviz::ViewController)