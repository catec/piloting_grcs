
#include "LinesHelper.h"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QsLog/QsLog.h>

#include <iostream>

#include "maps/items/Waypoint3dItem.h"

namespace gcs {
LinesHelper::LinesHelper(Ogre::SceneManager* manager) : rviz::Object(manager)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linesNode = manager->getRootSceneNode()->createChildSceneNode();

    _lineWptsMO    = manager->createManualObject();
    _lineMeasureMO = manager->createManualObject();
    _linePathMO    = manager->createManualObject();

    _linesNode->attachObject(_lineWptsMO);
    _linesNode->attachObject(_lineMeasureMO);
    _linesNode->attachObject(_linePathMO);

    _lineWptsMO->setDynamic(true);
    _lineMeasureMO->setDynamic(true);
    _linePathMO->setDynamic(true);
}

LinesHelper::~LinesHelper()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_linesNode->getParentSceneNode()) {
        _linesNode->getParentSceneNode()->removeChild(_linesNode);
    }
    scene_manager_->destroySceneNode(_linesNode);

    scene_manager_->destroyManualObject(_lineWptsMO);
    scene_manager_->destroyManualObject(_lineMeasureMO);
    scene_manager_->destroyManualObject(_linePathMO);
}

void LinesHelper::setPosition(const Ogre::Vector3& position)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linesNode->setPosition(position);
}

void LinesHelper::setOrientation(const Ogre::Quaternion& orientation)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linesNode->setOrientation(orientation);
}

void LinesHelper::setScale(const Ogre::Vector3& scale)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linesNode->setScale(scale);
}

void LinesHelper::setColor(float, float, float, float)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_WARN() << __PRETTY_FUNCTION__ << " - Color can't be changed when we use mesh\n";
}

void LinesHelper::setColor(const Ogre::ColourValue& color)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _lineWptsMO->colour(color);
}

void LinesHelper::setUserData(const Ogre::Any& data)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _lineWptsMO->getUserObjectBindings().setUserAny(data);
}

const Ogre::Vector3& LinesHelper::getPosition()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _linesNode->getPosition();
}
const Ogre::Quaternion& LinesHelper::getOrientation()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _linesNode->getOrientation();
}

void LinesHelper::addLinkerLine(const Waypoint3dItem& wpItem1, const Waypoint3dItem& wpItem2)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // _lineWptsMO->estimateVertexCount(2);
    _lineWptsMO->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

    Ogre::ColourValue color = Ogre::ColourValue(1.0f, 1.0f, 0.0f);
    if (wpItem1.inLoadedRoute() && wpItem2.inLoadedRoute()) {
        color = Ogre::ColourValue::Green;
    }

    _lineWptsMO->colour(color);

    Ogre::Vector3 xpos = Ogre::Vector3(
            wpItem1.waypoint().getPosition().getX(),
            wpItem1.waypoint().getPosition().getY(),
            wpItem1.waypoint().getPosition().getZ());
    Ogre::Vector3 ppos = Ogre::Vector3(
            wpItem2.waypoint().getPosition().getX(),
            wpItem2.waypoint().getPosition().getY(),
            wpItem2.waypoint().getPosition().getZ());

    _lineWptsMO->position(xpos.x, xpos.y, xpos.z);
    _lineWptsMO->position(ppos.x, ppos.y, ppos.z);

    _lineWptsMO->end();
    // _linesNode->setVisible(true, true);

    // context_->queueRender();
}

void LinesHelper::addMeasureLine(const Ogre::Vector3& pt1, const Ogre::Vector3& pt2)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // _lineMeasureMO->estimateVertexCount(2);
    _lineMeasureMO->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    _lineMeasureMO->colour(Ogre::ColourValue(0.15f, 0.57f, 1.0f));
    _lineMeasureMO->position(pt1.x, pt1.y, pt1.z);
    _lineMeasureMO->position(pt2.x, pt2.y, pt2.z);
    _lineMeasureMO->end();
}

void LinesHelper::addPath(const std::vector<Ogre::Vector3>& vehiclePath)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linePathMO->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    _linePathMO->colour(Ogre::ColourValue(0.48f, 0.73f, 0.98f));
    for (const auto& position : vehiclePath) {
        _linePathMO->position(position.x, position.y, position.z);
    }

    _linePathMO->end();
}

void LinesHelper::removeLinkerLines()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _lineWptsMO->clear();
    _lineWptsMO->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    _lineWptsMO->end();
}

void LinesHelper::removeMeasureLine()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _lineMeasureMO->clear();
    _lineMeasureMO->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    _lineMeasureMO->end();
}

void LinesHelper::removePathLine()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linePathMO->clear();
    _linePathMO->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    _linePathMO->end();
}

} // namespace gcs
