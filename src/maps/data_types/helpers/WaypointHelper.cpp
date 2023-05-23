#include "WaypointHelper.h"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QsLog/QsLog.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/ogre_helpers/shape.h>

namespace gcs {
WaypointHelper::WaypointHelper(Ogre::SceneManager* manager) : rviz::Object(manager)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

WaypointHelper::~WaypointHelper()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_wpNode->getParentSceneNode()) {
        _wpNode->getParentSceneNode()->removeChild(_wpNode);
    }
    scene_manager_->destroySceneNode(_wpNode);

    if (_wpTextNode->getParentSceneNode()) {
        _wpTextNode->getParentSceneNode()->removeChild(_wpTextNode);
    }
    scene_manager_->destroySceneNode(_wpTextNode);
}

void WaypointHelper::createWaypoint(
        const std::string&       id,
        const Ogre::Vector3&     position,
        const Ogre::Quaternion&  orientation,
        const float&             scale,
        const Ogre::ColourValue& color,
        const WaypointType&      type)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _wpNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

    rviz::Shape* wp_shape = nullptr;
    float        radius   = 0.15f * scale;
    if (type == WaypointType::ACTION) {
        radius   *= 2.0f;
        wp_shape  = new rviz::Shape(rviz::Shape::Cube, scene_manager_, _wpNode);
    } else {
        wp_shape = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, _wpNode);
        _wpAxes  = new rviz::Axes(scene_manager_, _wpNode, 0.3f * scale, 0.05f * scale);
        _wpAxes->getSceneNode()->setVisible(true);
        _wpAxes->setOrientation(orientation);
        _wpAxes->setPosition(position);
    }

    wp_shape->getRootNode()->setVisible(true);
    Ogre::Vector3 dimension(radius, radius, radius);
    wp_shape->setScale(dimension);
    wp_shape->setColor(color);
    wp_shape->setPosition(position);

    Ogre::ColourValue text_color = Ogre::ColourValue::White;
    if (id.find("(*)") != std::string::npos) {
        text_color = Ogre::ColourValue::Red;
    }

    rviz::MovableText* wp_id_text = new rviz::MovableText(id, "Liberation Sans", 0.3 * scale, text_color);
    wp_id_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);
    _wpTextNode = scene_manager_->getRootSceneNode()->createChildSceneNode();
    _wpTextNode->attachObject(wp_id_text);
    _wpTextNode->setVisible(true);
    _wpTextNode->setPosition(position[0], position[1], position[2] + 0.1f * scale);
}

void WaypointHelper::setPosition(const Ogre::Vector3& position)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. Check whether this should be the parent or the child
    _wpNode->setPosition(position);
    _wpTextNode->setPosition(position[0], position[1], position[2] + 0.1f);
}

void WaypointHelper::setOrientation(const Ogre::Quaternion& orientation)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. Check whether this should be the parent or the child
    _wpNode->setOrientation(orientation);
}

void WaypointHelper::setScale(const Ogre::Vector3&)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void WaypointHelper::setColor(float, float, float, float)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void WaypointHelper::setColor(const Ogre::ColourValue&)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void WaypointHelper::setUserData(const Ogre::Any&)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

const Ogre::Vector3& WaypointHelper::getPosition()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _wpAxes->getSceneNode()->getPosition();
}

const Ogre::Quaternion& WaypointHelper::getOrientation()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _wpAxes->getSceneNode()->getOrientation();
}

void WaypointHelper::headingRotation(const float& yaw)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _wpAxes->getSceneNode()->roll(Ogre::Degree(yaw));
}

} // namespace gcs
