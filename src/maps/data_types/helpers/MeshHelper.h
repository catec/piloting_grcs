
#pragma once

#include <rviz/ogre_helpers/object.h>

#include <string>

namespace Ogre {
class Vector3;
class Quaternion;
class Any;
class SceneNode;
class SceneManager;
class Entity;
} // namespace Ogre

namespace rviz {
class Arrow;
class Axes;
} // namespace rviz

namespace gcs {
class HomeDTO;

class MeshHelper : rviz::Object
{
  public:
    explicit MeshHelper(Ogre::SceneManager* manager);
    ~MeshHelper() override;

    bool initMeshFromPath(const std::string&);

    void updateHomePosition(const HomeDTO&);
    void removeLoadedHome();

    void setPosition(const Ogre::Vector3& position) override;
    void setOrientation(const Ogre::Quaternion& orientation) override;
    void setScale(const Ogre::Vector3& scale) override;
    void setColor(float, float, float, float) override;
    void setUserData(const Ogre::Any& data) override;
    void setVisibility(const bool& visible);

    const Ogre::Vector3&    getPosition() override;
    const Ogre::Quaternion& getOrientation() override;

  private:
    Ogre::SceneNode* _loadedHomeNode;
    Ogre::Entity*    _homeEntity;
};
} // namespace gcs
