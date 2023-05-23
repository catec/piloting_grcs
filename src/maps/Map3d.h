#pragma once

#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <QsLog/QsLog.h>
#include <common/IDTO.h>
#include <core/IAction.h>
#include <dataModel/dtos/EditionModeDTO.h>
#include <geometry_msgs/PoseStamped.h>
#include <maps/dtos/Map3dDTO.h>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <QColor>
#include <QObject>
#include <QTimer>
#include <QVector3D>

#include "CircularContainer.h"
#include "gcs_maps_export.h"
#include "maps/IMap.h"

#define NUM_POSES_PATH 100

namespace rviz {
class RenderPanel;
class VisualizationManager;
class ViewManager;
class GridDisplay;
class TFDisplay;
class ToolManager;
class Tool;
class Axes;
class Arrow;
class Shape;
} // namespace rviz

namespace Ogre {
class SceneNode;
}

namespace gcs {
class MissionDTO;
class RouteDTO;
class HomeDTO;
class RouteOptionsDTO;
class ApplicationStatusDTO;
class MeasureDTO;
class ViewModeDTO;
class CommsProgressDTO;
class QPointCloud2Display;
class QPoseDisplay;
class QMarkerDisplay;
class QMouseTool;
class FixedOrientationViewController;
class LinesHelper;
class WaypointHelper;
class MeshHelper;
class AssetParserThread;
class VisualizationConfigDTO;
class CloudEditionParametersDTO;
class RobotPositionVelocityNedDTO;
class RobotAttitudeAngVelocityDTO;

struct gcsColors {
    inline static auto lightBlue = Ogre::ColourValue(0.15f, 0.57f, 1.0f, 1.0f);
    inline static auto orange    = Ogre::ColourValue(1.0f, 0.55f, 0.0f, 1.0f);
    inline static auto lightRed  = Ogre::ColourValue(1.0f, 0.2f, 0.0f, 1.0f);
    inline static auto blue      = Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f);
    inline static auto purple    = Ogre::ColourValue(0.67f, 0.0f, 1.0f, 1.0f);
    inline static auto yellow    = Ogre::ColourValue(1.0f, 1.0f, 0.0f, 0.8f);
    inline static auto tYellow   = Ogre::ColourValue(1.0f, 1.0f, 0.0f, 0.2f);
    inline static auto tRed      = Ogre::ColourValue(1.0f, 0.0f, 0.0f, 0.2f);
    inline static auto tGreen    = Ogre::ColourValue(0.0f, 1.0f, 0.0f, 0.4f);
    inline static auto tBlue     = Ogre::ColourValue(0.0f, 0.0f, 1.0f, 0.4f);
};

class PILOTING_GRCS_MAPS_EXPORT Map3d : public QObject, public IMap
{
    Q_OBJECT

  public:
    explicit Map3d(QObject* parent = nullptr);
    virtual ~Map3d();

    void initializeRender() override;

    QWidget* getMapView() override;

  public Q_SLOTS:
    void manageCoreDTO(QSharedPointer<IDTO>) override;

    void initialize() override;
    void resetVisualizer() override;

    void loadPointCloud(const QString&, const int&);
    void updateViewInspectionTask(const QString&);

    void editPointCloud(const QSharedPointer<CloudEditionParametersDTO>&);
    void updateVisualizationConfig(const VisualizationConfigDTO&);
    void restorePointCloud();
    void hidePointCloud();
    void hideCutPlane();
    void hidePreviewAxis();

  Q_SIGNALS:
    void sendDTOToCore(ActionType actionType, QSharedPointer<IDTO> dto = nullptr) override;

  private Q_SLOTS:
    void cursorPosition(QVector3D);
    void addWaypoint(QVector3D);
    void editWaypoint(QSharedPointer<WaypointDTO>);
    void addHome(QPointF);
    void changeSelectedWaypoint(quint16);
    void removeWaypoint(quint16);
    void addRoute(QVector3D);
    void removeRoute(quint16);
    void sendCurrentRoute();
    void copyLoadedRouteToLocal();
    void addMeasuringPoint(QVector3D);
    void editHomeParameters();
    void addActionWpOverInspTask(QString);
    void addPoseWpOverInspTask(QString);

    void editWaypointHeading();

    void manageDirectionArrow(Ogre::Vector3, float, bool);
    void manageDirectionArrowVisibility();
    void updateVehiclePose();

    void showLoadedAssetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

    void sendConsoleMsg(const MsgType& type, const QString& msg);

    void modifyPreviewWpHeading(const float&);
    void modifySelectedWpHeading(const float&);

  private:
    void connectSignals();
    void createOriginAxis();
    void createPreviewAxis();

    void addInspectionTasks(const QList<InspectionTaskDTO>&);
    void addExclusionArea(const Ogre::Vector3&, const Ogre::Vector3&);
    void addSafetyArea(const Ogre::Vector3&, const float);
    void addInspectionArea(
            const std::string&, const Ogre::Vector3&, const Ogre::Quaternion&, const float, const float, const float);
    void addInspectionPoint(const std::string&, const Ogre::Vector3&, const float);

    void updateLoadedHome();

    void updateWaypoints();
    void linkWaypoints();
    void removeWaypoints();

    void updateWaypointItems(const MissionDTO&);
    void updateRouteOptions(const RouteOptionsDTO&);
    void updateApplicationStatus(const ApplicationStatusDTO&);
    void updateViewMode(const ViewModeDTO&);
    void updateEditionMode(const EditionModeDTO&);

    void addWaypointItemsByRoute(const RouteDTO&, const quint16, const bool, const bool);

    void displayGrid(const QString, const int, const float, const QColor);
    void displayPose(const Ogre::Vector3&, const Ogre::Quaternion&);

    void changeToCamera3D();
    void changeToCamera2D();

    void updateTool(rviz::Tool* tool);

    void addMeasuringPoints(const MeasureDTO&);
    void removeMeasuringPoints();

    void sendCloudDimensions(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
    void drawCuttingPlane(const float&, const Axis&);

    void startProgressBar(const QString&);
    void finishProgressBar();

    template <typename dataType>
    void resetInterestVector(dataType&);

    bool computeClickedPointNormal(const float&, const float&, const float&, Ogre::Quaternion&);

  private:
    Map3dDTO _mapDTO;

    bool _showLocalRoutes{true};
    bool _showLoadedRoutes{true};

    std::map<QString, QVariant> _camera2DParams;
    std::map<QString, QVariant> _camera3DParams;

    std::unique_ptr<rviz::RenderPanel> _render;
    rviz::VisualizationManager*        _visualizationManager;
    std::unique_ptr<rviz::ViewManager> _viewManager;

    std::unique_ptr<CircularContainer<Ogre::Vector3, NUM_POSES_PATH>> _path;

    std::unique_ptr<rviz::Axes>  _originAxe;
    std::unique_ptr<rviz::Arrow> _directionArrow;

    std::unique_ptr<rviz::Axes> _poseAxis;

    rviz::GridDisplay*           _gridDisplay;
    std::unique_ptr<rviz::Shape> _floorShape;

    QPointCloud2Display*                   _pcdDisplay;
    std::vector<float>                     _cloudDimensions;
    pcl::PointXYZRGB                       _cloudCentroid;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _assetCloud;

    rviz::ToolManager* _toolManager;
    rviz::Tool*        _cameraTool;
    QMouseTool*        _mouseTool;

    std::vector<std::unique_ptr<rviz::Shape>>    _measureSphereShapes;
    std::vector<std::unique_ptr<WaypointHelper>> _waypointsHelper;
    std::unique_ptr<LinesHelper>                 _linesHelper;
    std::unique_ptr<MeshHelper>                  _meshHelper;

    std::unordered_map<rviz::Shape*, Ogre::SceneNode*>     _exclusionAreas;
    std::unordered_map<rviz::Shape*, Ogre::SceneNode*>     _vehicleSafetyAreas;
    std::vector<std::pair<rviz::Shape*, Ogre::SceneNode*>> _inspectionTasks;

    std::unique_ptr<rviz::Shape> _cutCloudShape;
    std::unique_ptr<rviz::Axes>  _headingPreviewAxe;
    Ogre::Quaternion             _baseHeading;

    std::unique_ptr<AssetParserThread> _parserThr;

    QTimer _arrowVisibilityTimer;

    QTimer                                      _updatePoseTimer;
    QSharedPointer<RobotPositionVelocityNedDTO> _lastRobotPositionVelocityDTO{nullptr};
    QSharedPointer<RobotAttitudeAngVelocityDTO> _lastRobotAttitudeAngVelocityDTO{nullptr};
};

} // namespace gcs
