#pragma once

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/object.h>

#include <string>

#include "common/Types.h"

namespace Ogre {
class Vector3;
class Any;
class ManualObject;
class SceneNode;
class SceneManager;
class ColourValue;
} // namespace Ogre

namespace gcs {
class WaypointHelper : rviz::Object
{
  public:
    explicit WaypointHelper(Ogre::SceneManager* manager);
    ~WaypointHelper() override;

    void createWaypoint(
            const std::string&,
            const Ogre::Vector3&,
            const Ogre::Quaternion&,
            const float&,
            const Ogre::ColourValue&,
            const WaypointType&);

    void setPosition(const Ogre::Vector3& position) override;
    void setOrientation(const Ogre::Quaternion& orientation) override;
    void setScale(const Ogre::Vector3& scale) override;

    void setColor(const Ogre::ColourValue& color);
    void setColor(float r, float g, float b, float a) override;
    void setUserData(const Ogre::Any& data) override;

    void headingRotation(const float& yaw);

    const Ogre::Vector3&    getPosition() override;
    const Ogre::Quaternion& getOrientation() override;

  private:
    Ogre::SceneNode* _wpNode;
    Ogre::SceneNode* _wpTextNode;

    rviz::Axes* _wpAxes;
};
} // namespace gcs
