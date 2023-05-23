
#include "MeshHelper.h"

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QsLog/QsLog.h>
#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>

#include "dataModel/dtos/HomeDTO.h"

namespace gcs {
MeshHelper::MeshHelper(Ogre::SceneManager* manager) : rviz::Object(manager)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

MeshHelper::~MeshHelper()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_loadedHomeNode->getParentSceneNode()) {
        _loadedHomeNode->getParentSceneNode()->removeChild(_loadedHomeNode);
    }
    scene_manager_->destroySceneNode(_loadedHomeNode);

    if (_homeEntity) {
        _homeEntity->detachFromParent();
        scene_manager_->destroyEntity(_homeEntity);
    }
}

bool MeshHelper::initMeshFromPath(const std::string& path)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (rviz::loadMeshFromResource(path).isNull()) {
        return false;
    }

    _loadedHomeNode = scene_manager_->getRootSceneNode()->createChildSceneNode();
    _homeEntity     = scene_manager_->createEntity(path);
    _loadedHomeNode->attachObject(_homeEntity);
    _loadedHomeNode->setVisible(false);

    return true;
}

void MeshHelper::updateHomePosition(const HomeDTO& home)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const Ogre::Vector3 homePosition
            = Ogre::Vector3(home.getPosition().getX(), home.getPosition().getY(), home.getPosition().getZ());
    const Ogre::Quaternion homeOrientation
            = Ogre::Quaternion(Ogre::Degree(home.getYawOrientation()), Ogre::Vector3::UNIT_Z);

    _loadedHomeNode->setVisible(true);
    setPosition(homePosition);
    setOrientation(homeOrientation);
    // setScale(Ogre::Vector3(0.8f, 0.8f, 0.8f));
}

void MeshHelper::removeLoadedHome()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_loadedHomeNode) {
        return;
    }

    _loadedHomeNode->resetToInitialState();
    _loadedHomeNode->setVisible(false);
}

void MeshHelper::setPosition(const Ogre::Vector3& position)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _loadedHomeNode->setPosition(position);
    _loadedHomeNode->setVisible(true);
}

void MeshHelper::setOrientation(const Ogre::Quaternion& orientation)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _loadedHomeNode->setOrientation(orientation);
}

void MeshHelper::setScale(const Ogre::Vector3& scale)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _loadedHomeNode->setScale(scale);
}

void MeshHelper::setColor(float, float, float, float)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // Ogre::ColourValue color = Ogre::ColourValue(r, g, b, a));

    // manual_object_material_->getTechnique(0)->setAmbient(color * 0.5);
    // manual_object_material_->getTechnique(0)->setDiffuse(color);
    // manual_object_material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    // manual_object_material_->getTechnique(0)->setDepthWriteEnabled(true);

    QLOG_ERROR() << __PRETTY_FUNCTION__ << " - Color can't be changed when we use mesh\n";
}

void MeshHelper::setUserData(const Ogre::Any& data)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_homeEntity) {
        _homeEntity->getUserObjectBindings().setUserAny(data);
    }
}

void MeshHelper::setVisibility(const bool& visible)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_loadedHomeNode) {
        _loadedHomeNode->setVisible(visible);
    }
}

const Ogre::Vector3& MeshHelper::getPosition()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _loadedHomeNode->getPosition();
}

const Ogre::Quaternion& MeshHelper::getOrientation()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _loadedHomeNode->getOrientation();
}

} // namespace gcs