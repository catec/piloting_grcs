#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <rviz/ogre_helpers/object.h>

#include <vector>

namespace Ogre {
class Vector3;
class Quaternion;
class Any;
class ManualObject;
class SceneNode;
class SceneManager;
class ColourValue;
} // namespace Ogre

namespace gcs {
class Waypoint3dItem;

class LinesHelper : rviz::Object
{
  public:
    explicit LinesHelper(Ogre::SceneManager* manager);
    ~LinesHelper() override;

    void addLinkerLine(const Waypoint3dItem&, const Waypoint3dItem&);
    void addMeasureLine(const Ogre::Vector3&, const Ogre::Vector3&);
    void addPath(const std::vector<Ogre::Vector3>&);
    void removeLinkerLines();
    void removeMeasureLine();
    void removePathLine();

    void setPosition(const Ogre::Vector3& position) override;
    void setOrientation(const Ogre::Quaternion& orientation) override;
    void setScale(const Ogre::Vector3& scale) override;

    void setColor(const Ogre::ColourValue& color);
    void setColor(float r, float g, float b, float a) override;
    void setUserData(const Ogre::Any& data) override;

    const Ogre::Vector3&    getPosition() override;
    const Ogre::Quaternion& getOrientation() override;

  private:
    Ogre::ManualObject* _lineWptsMO;
    Ogre::ManualObject* _lineMeasureMO;
    Ogre::ManualObject* _linePathMO;

    Ogre::SceneNode* _linesNode;
};
} // namespace gcs
