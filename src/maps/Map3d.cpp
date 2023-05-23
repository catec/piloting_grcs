
#include "maps/Map3d.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>
#include <common/Types.h>
#include <communications/MavSDK/dtos/CurrentInspItemDTO.h>
#include <core/dtos/BasicIntDTO.h>
#include <dataModel/PoseDTO.h>
#include <dataModel/RobotAttitudeAngVelocityDTO.h>
#include <dataModel/RobotPositionVelocityNedDTO.h>
#include <dataModel/RouteDTO.h>
#include <dataModel/dtos/ApplicationStatusDTO.h>
#include <dataModel/dtos/AssetDTO.h>
#include <dataModel/dtos/CloudDimensionsDTO.h>
#include <dataModel/dtos/CloudEditionParametersDTO.h>
#include <dataModel/dtos/CommsProgressDTO.h>
#include <dataModel/dtos/CommsStatusDTO.h>
#include <dataModel/dtos/CursorPositionDTO.h>
#include <dataModel/dtos/DialogDTO.h>
#include <dataModel/dtos/EditionModeDTO.h>
#include <dataModel/dtos/HomeDTO.h>
#include <dataModel/dtos/InspectionPlanDTO.h>
#include <dataModel/dtos/MeasureDTO.h>
#include <dataModel/dtos/MissionDTO.h>
#include <dataModel/dtos/MissionStatusDTO.h>
#include <dataModel/dtos/MsgConsoleDTO.h>
#include <dataModel/dtos/RouteOptionsDTO.h>
#include <dataModel/dtos/ViewModeDTO.h>
#include <dataModel/dtos/VisualizationConfigDTO.h>
#include <gui/dialogs/homeParameters/HomeParametersDialog.h>
#include <gui/dialogs/inspectionTaskActionInfo/InspectionTaskActionInfoDialog.h>
#include <gui/dialogs/waypointsOptions/WaypointsOptionsDialog.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <rviz/default_plugin/grid_display.h>
#include <rviz/default_plugin/tf_display.h>
#include <rviz/display_group.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

#include <QApplication>
#include <QTime>

#include "cloud_tools/AssetParserThread.h"
#include "cloud_tools/CloudEditionWorker.h"
#include "data_types/displays/QPointCloud2Display.h"
#include "data_types/helpers/LinesHelper.h"
#include "data_types/helpers/MeshHelper.h"
#include "data_types/helpers/WaypointHelper.h"
#include "data_types/tools/QMouseTool.h"
#include "data_types/view_controllers/FixedOrientationViewController.h"
#include "items/Waypoint3dItem.h"

namespace gcs {

Map3d::Map3d(QObject* parent) :
        QObject(parent),
        _render(std::make_unique<rviz::RenderPanel>()),
        _cutCloudShape(nullptr),
        _baseHeading(Ogre::Quaternion(1.0, 0.0, 0.0, 0.0)),
        _arrowVisibilityTimer(this)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    int   argc = 0;
    char* argv = nullptr;
    ros::init(argc, &argv, "gcs_node", ros::init_options::AnonymousName);

    _path = std::make_unique<CircularContainer<Ogre::Vector3, NUM_POSES_PATH>>();

    // clang-format off
    connect(&_arrowVisibilityTimer, &QTimer::timeout,
            this,                   &Map3d::manageDirectionArrowVisibility);

    connect(&_updatePoseTimer, &QTimer::timeout,
            this,              &Map3d::updateVehiclePose);
    // clang-format on

    _arrowVisibilityTimer.setInterval(2000); /// 2 Hz
    _updatePoseTimer.setInterval(33.333);    /// 30 Hz
}

Map3d::~Map3d()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _visualizationManager->removeAllDisplays();
    _toolManager->removeAll();

    if (_waypointsHelper.size() > 0) {
        _waypointsHelper.clear();
    }

    resetVisualizer();
    delete _mouseTool;
}

QWidget* Map3d::getMapView()
{
    return dynamic_cast<QWidget*>(_render.get());
}

void Map3d::initializeRender()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    _visualizationManager = new rviz::VisualizationManager(_render.get());

    _meshHelper                = std::make_unique<MeshHelper>(_visualizationManager->getSceneManager());
    const std::string meshPath = "package://piloting_grcs/data/meshes/landing-base-mesh.dae";
    if (!_meshHelper->initMeshFromPath(meshPath)) {
        auto dtoToCore          = createSharedDTO<MsgConsoleDTO>();
        dtoToCore->getMsgType() = MsgType::ERROR;
        dtoToCore->getMessage() = QString("Unable to load home mesh from: %1. "
                                          "Please, source devel file from the package")
                                          .arg(QString::fromStdString(meshPath));
        Q_EMIT sendDTOToCore(ActionType::UPDATE_MSG_CONSOLE, QSharedPointer<IDTO>(dtoToCore));
    }

    _render->initialize(_visualizationManager->getSceneManager(), _visualizationManager);
    _visualizationManager->initialize();
    _visualizationManager->removeAllDisplays();
    _visualizationManager->setFixedFrame("map");

    _viewManager = std::unique_ptr<rviz::ViewManager>(_visualizationManager->getViewManager());
    _viewManager->setRenderPanel(_render.get());
    _viewManager->setCurrentViewControllerType("gcs/Orbit");
    auto viewController = _viewManager->getCurrent();

    /// \note Initial parameters to Orbit view controller
    viewController->subProp("Focal Point")->setValue("0;0;0");
    viewController->subProp("Yaw")->setValue("0.78");
    viewController->subProp("Pitch")->setValue("0.78");
    viewController->subProp("Distance")->setValue("10.0");

    _toolManager = _visualizationManager->getToolManager();
    _toolManager->removeAll();

    _mouseTool  = new QMouseTool(_mapDTO);
    _cameraTool = _toolManager->addTool("rviz/MoveCamera");

    displayGrid("map", 50, 1.0f, QColor(125, 125, 125));
    createOriginAxis();

    createPreviewAxis();

    _linesHelper = std::make_unique<LinesHelper>(_visualizationManager->getSceneManager());

    _pcdDisplay = new QPointCloud2Display();
    _pcdDisplay->initialize(_visualizationManager);
    _pcdDisplay->setVisible(true);
    _visualizationManager->getRootDisplayGroup()->addDisplay(_pcdDisplay);

    _visualizationManager->startUpdate();

    connectSignals();
}

void Map3d::createOriginAxis()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_originAxe) {
        _originAxe.reset();
    }

    _originAxe = std::make_unique<rviz::Axes>(_visualizationManager->getSceneManager(), nullptr, 2.0f, 0.1f);
    _originAxe->setPosition(Ogre::Vector3(0.0, 0.0, 0.0));
    _originAxe->setOrientation(Ogre::Quaternion(1.0, 0.0, 0.0, 0.0));
    _originAxe->getSceneNode()->setVisible(_mapDTO.getVisualizationConfig().getShowGlobalAxis());
}

void Map3d::createPreviewAxis()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_headingPreviewAxe) {
        _headingPreviewAxe.reset();
    }

    const auto waypointSizeScale = _mapDTO.getVisualizationConfig().getWaypointSizeScale();
    _headingPreviewAxe           = std::make_unique<rviz::Axes>(
            _visualizationManager->getSceneManager(), nullptr, 0.3f * waypointSizeScale, 0.05f * waypointSizeScale);
    _headingPreviewAxe->setXColor(Ogre::ColourValue(1.0, 0.0, 0.0, 0.5));
    _headingPreviewAxe->setYColor(Ogre::ColourValue(0.0, 1.0, 0.0, 0.5));
    _headingPreviewAxe->setZColor(Ogre::ColourValue(0.0, 0.0, 1.0, 0.5));
    _headingPreviewAxe->getSceneNode()->setVisible(false);
}

/// \note. This method will ne unnecessary
void Map3d::initialize()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_poseAxis) {
        _poseAxis.reset();
    }

    _poseAxis = std::make_unique<rviz::Axes>(_visualizationManager->getSceneManager(), nullptr, 0.4f, 0.05f);
    _poseAxis->getSceneNode()->setVisible(true);
}

void Map3d::connectSignals()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    // clang-format off
    connect(_mouseTool, &QMouseTool::cursorPosition,
            this,       &Map3d::cursorPosition);

    connect(_mouseTool, &QMouseTool::addWaypoint,
            this,       &Map3d::addWaypoint);

    connect(_mouseTool, &QMouseTool::editWaypoint,
            this,       &Map3d::editWaypoint);
    
    connect(_mouseTool, &QMouseTool::addHome,
            this,       &Map3d::addHome);

    connect(_mouseTool, &QMouseTool::changeSelectedWaypoint,
            this,       &Map3d::changeSelectedWaypoint);

    connect(_mouseTool, &QMouseTool::removeWaypoint,
            this,       &Map3d::removeWaypoint);

    connect(_mouseTool, &QMouseTool::addRoute,
            this,       &Map3d::addRoute);

    connect(_mouseTool, &QMouseTool::removeRoute,
            this,       &Map3d::removeRoute);

    connect(_mouseTool, &QMouseTool::sendCurrentRoute,
            this,       &Map3d::sendCurrentRoute);

    connect(_mouseTool, &QMouseTool::copyLoadedRouteToLocal,
            this,       &Map3d::copyLoadedRouteToLocal);

    connect(_mouseTool, &QMouseTool::addMeasuringPoint,
            this,       &Map3d::addMeasuringPoint);

    connect(_mouseTool, &QMouseTool::editHomeParameters,
            this,       &Map3d::editHomeParameters);

    connect(_mouseTool, &QMouseTool::manageDirectionArrow,
            this,       &Map3d::manageDirectionArrow);

    connect(_mouseTool, &QMouseTool::addActionWpOverInspTask,
            this,       &Map3d::addActionWpOverInspTask);

    connect(_mouseTool, &QMouseTool::updateViewInspectionTask,
            this,       &Map3d::updateViewInspectionTask);

    connect(_mouseTool, &QMouseTool::editWaypointHeading,
            this,       &Map3d::editWaypointHeading);

    connect(_mouseTool, &QMouseTool::addPoseWpOverInspTask,
            this,       &Map3d::addPoseWpOverInspTask);
    // clang-format on
}

void Map3d::resetVisualizer()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_poseAxis) {
        _poseAxis.reset();
    }

    _path->clear();
    _linesHelper->removePathLine();

    // resetInterestVector(_exclusionAreas);
    // resetInterestVector(_vehicleSafetyAreas);
    // resetInterestVector(_inspectionTasks);
}

template <typename dataType>
void Map3d::resetInterestVector(dataType& interestVector)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (interestVector.size() > 0) {
        for (const auto& [shape, text_node] : interestVector) {
            if (text_node->getParentSceneNode()) {
                text_node->getParentSceneNode()->removeChild(text_node);
            }
            _visualizationManager->getSceneManager()->destroySceneNode(text_node);

            delete shape;
        }
        interestVector.clear();
    }
}

void Map3d::manageCoreDTO(QSharedPointer<IDTO> idto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!idto) {
        return;
    }

    switch (idto->getDTOType()) {
        case DTOType::COMMS_STATUS: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: COMMS_STATUS";

            auto dto = idto.dynamicCast<CommsStatusDTO>();
            if (dto) {
                _mapDTO.getCommsIsConnected() = dto->getIsConnected();

                /// \note. When disconnected, the pose is resetted
                if (!_mapDTO.getCommsIsConnected() && _updatePoseTimer.isActive()) {
                    _updatePoseTimer.stop();
                    _lastRobotPositionVelocityDTO.reset(new RobotPositionVelocityNedDTO);
                    _lastRobotAttitudeAngVelocityDTO.reset(new RobotAttitudeAngVelocityDTO);

                    _poseAxis.reset();
                }
            }
            break;
        }
        case DTOType::EDITION_MODE: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: EDITION_MODE";

            auto dto = idto.dynamicCast<EditionModeDTO>();
            if (dto) {
                updateEditionMode(*dto);
            }
            break;
        }
        case DTOType::VIEW_MODE: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: VIEW_MODE";

            auto dto = idto.dynamicCast<ViewModeDTO>();
            if (dto) {
                updateViewMode(*dto);
            }
            break;
        }
        case DTOType::APPLICATION_STATUS: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: APPLICATION_STATUS";

            auto dto = idto.dynamicCast<ApplicationStatusDTO>();
            if (dto) {
                updateApplicationStatus(*dto);
            }
            break;
        }
        case DTOType::INSPECTION_PLAN: {
            auto dto = idto.dynamicCast<InspectionPlanDTO>();
            if (dto) {
                _mapDTO.getInspectionTaskList() = dto->getInspectionTaskList();

                addInspectionTasks(_mapDTO.getInspectionTaskList());
            }
            break;
        }
        case DTOType::ASSET: {
            auto dto = idto.dynamicCast<AssetDTO>();
            if (dto) {
                if (!dto->getLoadedFile().getPath().isEmpty()) {
                    loadPointCloud(dto->getLoadedFile().getPath(), dto->getLoadedFile().getSize());
                }
            }
            break;
        }
        case DTOType::MISSION: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: MISSION";

            auto dto = idto.dynamicCast<MissionDTO>();
            if (dto) {
                updateWaypointItems(*dto);
            }
            break;
        }
        case DTOType::ROUTE_OPTIONS: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: ROUTE_OPTIONS";

            auto dto = idto.dynamicCast<RouteOptionsDTO>();
            if (dto) {
                updateRouteOptions(*dto);
            }
            break;
        }
        case DTOType::MEASURE: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: MEASURE";

            auto dto = idto.dynamicCast<MeasureDTO>();
            if (dto) {
                addMeasuringPoints(*dto);
            }
            break;
        }
        case DTOType::CURRENT_INSP_ITEM: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: CURRENT_INSP_ITEM";

            auto dto = idto.dynamicCast<CurrentInspItemDTO>();
            if (dto && _mapDTO.getCommsIsConnected()) {
                _mapDTO.getCurrentWpIndex() = dto->getIndex();

                removeWaypoints();
                updateWaypoints();
            }
            break;
        }
        case DTOType::ROBOT_ATTITUDE_ANGVEL: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: ROBOT_ATTITUDE_ANGVEL";

            auto dto = idto.dynamicCast<RobotAttitudeAngVelocityDTO>();
            if (dto) {
                _lastRobotAttitudeAngVelocityDTO = dto;
            }
            break;
        }
        case DTOType::ROBOT_POSITION_VELOCITY_NED: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: ROBOT_POSITION_VELOCITY";

            /// \note. When is connected and the timer is not active, the timer is started
            if (_mapDTO.getCommsIsConnected() && !_updatePoseTimer.isActive()) {
                _updatePoseTimer.start();
            }

            auto dto = idto.dynamicCast<RobotPositionVelocityNedDTO>();
            if (dto) {
                _lastRobotPositionVelocityDTO = dto;
            }
            break;
        }
        case DTOType::VISUALIZATION_CONFIG: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: VISUALIZATION_CONFIG";

            auto dto = idto.dynamicCast<VisualizationConfigDTO>();
            if (dto) {
                updateVisualizationConfig(*dto);
            }

            break;
        }
        default: {
            const auto msg = QString("DTO type is not managed: %1").arg(static_cast<uint8_t>(idto->getDTOType()));

            // QLOG_WARN() << __PRETTY_FUNCTION__ << " - " << msg;
            break;
        }
    }
}

void Map3d::updateTool(rviz::Tool* tool)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_toolManager->getDefaultTool() == tool) {
        return;
    }

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Setting the tool: " << tool->getClassId() << " as current";

    _toolManager->setCurrentTool(tool);
    _toolManager->setDefaultTool(tool);
}

void Map3d::updateEditionMode(const EditionModeDTO& currEditionMode)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (currEditionMode.getMode()) {
        updateTool(_mouseTool);
        updateViewInspectionTask("");
    } else {
        updateTool(_cameraTool);
    }

    _mapDTO.getInEditionMode() = currEditionMode.getMode();
}

void Map3d::updateApplicationStatus(const ApplicationStatusDTO& currAppStatus)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const AppStatusType lastStatus = _mapDTO.getAppStatus();
    if (currAppStatus.getStatus() != AppStatusType::Measuring && lastStatus == AppStatusType::Measuring) {
        removeMeasuringPoints();
    }

    if (currAppStatus.getStatus() == AppStatusType::EditingLoadedHome
        && _mapDTO.getLoadedHome().getIsEnabled() == false) {
        editHomeParameters();
    }

    _mapDTO.getAppStatus() = currAppStatus.getStatus();
}

void Map3d::updateViewMode(const ViewModeDTO& viewMode)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (viewMode.getMode() == ViewModeType::TwoD) {
        changeToCamera2D();
    } else if (viewMode.getMode() == ViewModeType::ThreeD) {
        changeToCamera3D();
    }

    _mapDTO.getViewMode() = viewMode.getMode();
}

void Map3d::updateRouteOptions(const RouteOptionsDTO& routeOptionsDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Q_UNUSED(routeOptionsDTO);

    const RouteOptions& routeOptions = routeOptionsDTO.getFlag();
    _showLocalRoutes                 = routeOptions & ShowLocal;
    _showLoadedRoutes                = routeOptions & ShowLoaded;

    removeWaypoints();
    updateWaypoints();
}

void Map3d::updateVisualizationConfig(const VisualizationConfigDTO& visualizConfig)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _mapDTO.getVisualizationConfig() = visualizConfig;

    /// \note. Update floor plane
    if (!_gridDisplay || !_floorShape) {
        displayGrid("map", 50, 1.0f, QColor(125, 125, 125));
    }

    if (_gridDisplay) {
        _gridDisplay->setEnabled(_mapDTO.getVisualizationConfig().getShowFloorGrid());
    }

    if (_floorShape) {
        _floorShape->getRootNode()->setVisible(_mapDTO.getVisualizationConfig().getShowFloorGrid());
    }

    /// \note. Modify sizes of wpts
    removeWaypoints();
    updateWaypoints();

    /// \note. Modify offsets to be able to select wpts in 3D view
    _mouseTool->updateWpScaleRangeSelectable(_mapDTO.getVisualizationConfig().getWaypointSizeScale());

    /// \note. Modify preview axe size
    createPreviewAxis();

    /// \note. Modify sizes points of the pointcloud
    if (_pcdDisplay) {
        _pcdDisplay->configureVisualization(_mapDTO.getVisualizationConfig());
    }

    /// \note. Modify sizes of inspection pts and transparency of the inspection locations
    if (_mapDTO.getInspectionTaskList().size() > 0) {
        addInspectionTasks(_mapDTO.getInspectionTaskList());
    }

    /// \note Modifiy visibility of origin axis
    if (_originAxe) {
        _originAxe->getSceneNode()->setVisible(_mapDTO.getVisualizationConfig().getShowGlobalAxis());
    }

    /// \note Modifiy visibility of home position
    updateLoadedHome();
}

void Map3d::updateLoadedHome()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_mapDTO.getLoadedHome().getIsEnabled()) {
        return;
    }

    const Ogre::Vector3 homePosition = Ogre::Vector3(
            _mapDTO.getLoadedHome().getPosition().getX(),
            _mapDTO.getLoadedHome().getPosition().getY(),
            _mapDTO.getLoadedHome().getPosition().getZ());
    const Ogre::Quaternion homeOrientation
            = Ogre::Quaternion(Ogre::Degree(_mapDTO.getLoadedHome().getYawOrientation()), Ogre::Vector3::UNIT_Z);
    _meshHelper->setPosition(homePosition);
    _meshHelper->setOrientation(homeOrientation);
    _meshHelper->setVisibility(_mapDTO.getVisualizationConfig().getShowHomePosition());
}

void Map3d::updateWaypointItems(const MissionDTO& missionDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    removeWaypoints();

    if (_mapDTO.getWaypointsList().size() > 0) {
        _mapDTO.getWaypointsList().clear();
    }

    for (const auto& route : missionDTO.getLocalRoutesMap().values()) {
        addWaypointItemsByRoute(
                route, missionDTO.getCurrentWaypointId(), route.getId() == missionDTO.getCurrentRouteId(), false);
    }
    addWaypointItemsByRoute(missionDTO.getLoadedRoute(), 0, false, true);

    updateWaypoints();

    _mapDTO.getLoadedHome() = missionDTO.getLoadedHome();
    if (!_mapDTO.getLoadedHome().getIsEnabled()) {
        _meshHelper->removeLoadedHome();
    }
    updateLoadedHome();
}

void Map3d::addWaypointItemsByRoute(
        const RouteDTO& route, const quint16 currentWaypointId, const bool selectedRoute, const bool loadedRoute)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note Sequence number starts with 0 because the route doesn't contain the home position.
    quint16 seqNumber = 0;
    for (auto it = route.getWpsList().cbegin(); it != route.getWpsList().cend(); ++it) {
        WaypointDTO waypoint = *it;
        auto        iter     = std::find_if(
                _mapDTO.getWaypointsList().cbegin(),
                _mapDTO.getWaypointsList().cend(),
                [&](const QSharedPointer<Waypoint3dItem> wp) {
                    /// \note This condition may need to be reviewed
                    return (waypoint.getId() != 0) && (wp->id() == waypoint.getId() && wp->inLoadedRoute());
                });

        if (iter != _mapDTO.getWaypointsList().cend()) {
            (*iter)->inSelectedRoute() = selectedRoute;
            (*iter)->inLoadedRoute()   = loadedRoute;
            if (waypoint.getId() != 0) {
                (*iter)->isSelectedWaypoint() = waypoint.getId() == currentWaypointId ? true : false;
            }
        } else {
            QSharedPointer<Waypoint3dItem> wpToAdd = QSharedPointer<Waypoint3dItem>::create();
            wpToAdd->id()                          = waypoint.getId();
            wpToAdd->seqNumber()                   = seqNumber++;
            wpToAdd->routeId()                     = route.getId();
            wpToAdd->waypoint()                    = waypoint;
            wpToAdd->inSelectedRoute()             = selectedRoute;
            wpToAdd->inLoadedRoute()               = loadedRoute;
            if (waypoint.getId() != 0) {
                wpToAdd->isSelectedWaypoint() = waypoint.getId() == currentWaypointId ? true : false;
            }

            QLOG_DEBUG() << "WP -> id:" << wpToAdd->id() << "seqNumber:" << wpToAdd->seqNumber()
                         << "routeId:" << wpToAdd->routeId() << "inLoadedRoute?" << wpToAdd->inLoadedRoute()
                         << "inSelectedRoute?" << wpToAdd->inSelectedRoute() << "isSelectedWaypoint?"
                         << wpToAdd->isSelectedWaypoint() << "taskUUID:" << wpToAdd->waypoint().getTaskUUID()
                         << "waypoint:(" << wpToAdd->waypoint().getPosition().getX() << ","
                         << wpToAdd->waypoint().getPosition().getY() << "," << wpToAdd->waypoint().getPosition().getZ()
                         << ")";

            _mapDTO.getWaypointsList().append(wpToAdd);
        }
    }
}

void Map3d::updateWaypoints()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_mapDTO.getVisibleWaypointsList().size() > 0) {
        _mapDTO.getVisibleWaypointsList().clear();
    }

    if (!_showLocalRoutes && !_showLoadedRoutes) {
        return;
    }

    QString prevInspTaskUUID;
    for (const auto& waypointItem : _mapDTO.getWaypointsList()) {
        if ((waypointItem->inLoadedRoute() && !_showLoadedRoutes)
            || (!_showLocalRoutes && !waypointItem->inLoadedRoute())) {
            continue;
        } else {
            _mapDTO.getVisibleWaypointsList().append(waypointItem);
        }

        const bool isNewInspectionTask = (prevInspTaskUUID != waypointItem->waypoint().getTaskUUID()) ? true : false;

        auto        wp       = std::make_unique<WaypointHelper>(_visualizationManager->getSceneManager());
        std::string wpText   = std::string("");
        const auto  showText = _mapDTO.getVisualizationConfig().getShowItemsText();
        if (showText) {
            if (isNewInspectionTask) {
                wpText = std::to_string(waypointItem->seqNumber()) + " ("
                       + waypointItem->waypoint().getTaskUUID().toStdString() + ")";
            } else {
                wpText = std::to_string(waypointItem->seqNumber());
            }
        }
        Ogre::ColourValue wpColor;
        if (waypointItem->inLoadedRoute() && (waypointItem->seqNumber() == _mapDTO.getCurrentWpIndex())) {
            wpColor = gcsColors::orange;
        } else if (isNewInspectionTask) {
            wpColor = gcsColors::purple;
        } else if (waypointItem->isSelectedWaypoint() && !waypointItem->inLoadedRoute() && _mapDTO.getInEditionMode()) {
            wpColor = gcsColors::lightRed;
        } else {
            wpColor = gcsColors::lightBlue;
        }

        const Ogre::Vector3 wpPosition = Ogre::Vector3(
                waypointItem->waypoint().getPosition().getX(),
                waypointItem->waypoint().getPosition().getY(),
                waypointItem->waypoint().getPosition().getZ());
        const Ogre::Quaternion wpOrientation = Ogre::Quaternion(
                waypointItem->waypoint().getOrientation().getW(),
                waypointItem->waypoint().getOrientation().getX(),
                waypointItem->waypoint().getOrientation().getY(),
                waypointItem->waypoint().getOrientation().getZ());

        const auto waypointSizeScale = _mapDTO.getVisualizationConfig().getWaypointSizeScale();
        wp->createWaypoint(
                wpText, wpPosition, wpOrientation, waypointSizeScale, wpColor, waypointItem->waypoint().getWpType());
        _waypointsHelper.push_back(std::move(wp));

        prevInspTaskUUID = waypointItem->waypoint().getTaskUUID();
    }

    linkWaypoints();
}

void Map3d::linkWaypoints()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linesHelper->removeLinkerLines();

    /// \note Not link between Waypoints not visible
    auto waypointList = _mapDTO.getVisibleWaypointsList();

    int iFirstWaypointOfRoute = -1;
    for (int i = 1; i <= waypointList.size(); i++) {
        if (i != waypointList.size() && waypointList[i - 1]->routeId() == waypointList[i]->routeId()) {
            /// Set the first Wp of this Route
            if (iFirstWaypointOfRoute == -1) {
                iFirstWaypointOfRoute = i - 1;
            }
            _linesHelper->addLinkerLine(*waypointList[i - 1], *waypointList[i]);
        }
    }
}

void Map3d::addMeasuringPoints(const MeasureDTO& dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _mapDTO.getMeasureList() = dto.getPointList();

    removeMeasuringPoints();

    const float   measurePtRadius = 0.2f;
    Ogre::Vector3 ptSize(measurePtRadius, measurePtRadius, measurePtRadius);
    for (const auto& measurePt : _mapDTO.getMeasureList()) {
        auto pt_sphere = std::make_unique<rviz::Shape>(rviz::Shape::Sphere, _visualizationManager->getSceneManager());
        pt_sphere->getRootNode()->setVisible(true);
        pt_sphere->setScale(ptSize);
        pt_sphere->setColor(gcsColors::blue);
        pt_sphere->setPosition(Ogre::Vector3(measurePt.getX(), measurePt.getY(), measurePt.getZ()));

        _measureSphereShapes.push_back(std::move(pt_sphere));
    }

    if (_mapDTO.getMeasureList().size() == 2) {
        _linesHelper->addMeasureLine(
                Ogre::Vector3(
                        _mapDTO.getMeasureList().at(0).getX(),
                        _mapDTO.getMeasureList().at(0).getY(),
                        _mapDTO.getMeasureList().at(0).getZ()),
                Ogre::Vector3(
                        _mapDTO.getMeasureList().at(1).getX(),
                        _mapDTO.getMeasureList().at(1).getY(),
                        _mapDTO.getMeasureList().at(1).getZ()));
    }
}

void Map3d::removeMeasuringPoints()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _linesHelper->removeMeasureLine();

    if (_measureSphereShapes.size() > 0) {
        _measureSphereShapes.clear();
    }
}

void Map3d::updateVehiclePose()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto lastVehiclePositionVelocityDTO  = _lastRobotPositionVelocityDTO;
    auto lastRobotAttitudeAngVelocityDTO = _lastRobotAttitudeAngVelocityDTO;
    if (!lastVehiclePositionVelocityDTO || !lastRobotAttitudeAngVelocityDTO) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Invalid received position or orientation DTO";
        return;
    }

    const auto position = Ogre::Vector3(
            lastVehiclePositionVelocityDTO->getPositionX(),
            lastVehiclePositionVelocityDTO->getPositionY(),
            lastVehiclePositionVelocityDTO->getPositionZ());
    const auto orientation = Ogre::Quaternion(
            lastRobotAttitudeAngVelocityDTO->getQuaternionW(),
            lastRobotAttitudeAngVelocityDTO->getQuaternionX(),
            lastRobotAttitudeAngVelocityDTO->getQuaternionY(),
            lastRobotAttitudeAngVelocityDTO->getQuaternionZ());
    if (!position.isNaN() && !orientation.isNaN()) {
        displayPose(position, orientation);
    }
}

void Map3d::displayPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_poseAxis) {
        _poseAxis = std::make_unique<rviz::Axes>(_visualizationManager->getSceneManager(), nullptr, 0.4f, 0.05f);
        _poseAxis->getSceneNode()->setVisible(true);
    }

    _poseAxis->setPosition(position);
    _poseAxis->setOrientation(orientation);

    _path->add(position);

    _linesHelper->removePathLine();
    _linesHelper->addPath(_path->getVector());
}

void Map3d::displayGrid(
        const QString referenceFrame, const int planCellCount, const float metersPerCell, const QColor color)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _gridDisplay = dynamic_cast<rviz::GridDisplay*>(_visualizationManager->createDisplay("rviz/Grid", "QGrid", true));
    ROS_ASSERT(_gridDisplay != nullptr);

    _gridDisplay->subProp("Reference Frame")->setValue(referenceFrame);
    _gridDisplay->subProp("Plane Cell Count")->setValue(planCellCount);
    _gridDisplay->subProp("Cell Size")->setValue(metersPerCell);
    _gridDisplay->subProp("Color")->setValue(color);
    _gridDisplay->subProp("Line Style")->setValue("Billboards");

    _gridDisplay->setEnabled(true);

    /// \note. Add floor plane to be able to add home in 3D view
    if (_floorShape) {
        _floorShape.reset();
    }

    _floorShape = std::make_unique<rviz::Shape>(rviz::Shape::Cube, _visualizationManager->getSceneManager());
    _floorShape->setScale(Ogre::Vector3(planCellCount, planCellCount, 0.01)); // (depth, width, height)
    _floorShape->setColor(Ogre::ColourValue(0.19f, 0.19f, 0.19f, 0.001f));
    _floorShape->setPosition(Ogre::Vector3(0.0f, 0.0f, 0.0f));
}

void Map3d::changeToCamera2D()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto prevController = _viewManager->getCurrent();
    if (prevController && prevController->getClassId() == QString("gcs/Orbit")) {
        _camera3DParams["Distance"]    = prevController->subProp("Distance")->getValue();
        _camera3DParams["Yaw"]         = prevController->subProp("Yaw")->getValue();
        _camera3DParams["Pitch"]       = prevController->subProp("Pitch")->getValue();
        _camera3DParams["Focal Point"] = prevController->subProp("Focal Point")->getValue();
    }

    _viewManager->setCurrentViewControllerType("gcs/TopCamera");
    auto viewController = _viewManager->getCurrent();
    /// \note. Hardcoded ROS frame_id
    viewController->subProp("Target Frame")->setValue("map");

    if (_camera2DParams.empty()) {
        viewController->reset();
    }

    for (const auto& [name, value] : _camera2DParams) {
        viewController->subProp(name)->setValue(value);
    }
}

void Map3d::changeToCamera3D()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto prevController = _viewManager->getCurrent();
    if (prevController && prevController->getClassId() == QString("gcs/TopCamera")) {
        _camera2DParams["Scale"]        = prevController->subProp("Scale")->getValue();
        _camera2DParams["X"]            = prevController->subProp("X")->getValue();
        _camera2DParams["Y"]            = prevController->subProp("Y")->getValue();
        _camera2DParams["Angle"]        = prevController->subProp("Angle")->getValue();
        _camera2DParams["Target Frame"] = prevController->subProp("Target Frame")->getValue();
    }

    _viewManager->setCurrentViewControllerType("gcs/Orbit");
    auto viewController = _viewManager->getCurrent();

    for (const auto& [name, value] : _camera3DParams) {
        viewController->subProp(name)->setValue(value);
    }
}

void Map3d::removeWaypoints()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_waypointsHelper.size() > 0) {
        _waypointsHelper.clear();
    }

    _linesHelper->removeLinkerLines();
    /// \note IDK if it's necesary after any helper modification
    /// (Queues a render. Multiple calls before a render happens will only cause a single render.)
    // _visualizationManager.get()->queueRender();
}

void Map3d::loadPointCloud(const QString& filePath, const int& assetSize)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    startProgressBar("Loading asset...");

    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("pcl::PointCloud<pcl::PointXYZRGB>::Ptr");

    _parserThr = std::make_unique<AssetParserThread>();
    _parserThr->setObjectName("ParserThread");
    _parserThr->init(filePath, assetSize);
    // clang-format off
    connect(_parserThr.get(), &AssetParserThread::loadedAssetPointCloud, 
            this,             &Map3d::showLoadedAssetPointCloud);
    // clang-format on
    _parserThr->start();
}

void Map3d::showLoadedAssetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_assetCloud) {
        _assetCloud.reset();
    }
    _assetCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*cloud, *_assetCloud);

    sendCloudDimensions(_assetCloud);
    _pcdDisplay->reset();
    _pcdDisplay->configureVisualization(_mapDTO.getVisualizationConfig());
    _pcdDisplay->attachData(_assetCloud);

    sendConsoleMsg(MsgType::INFO, "Successfully loaded asset");
    finishProgressBar();

    Q_EMIT sendDTOToCore(ActionType::LOAD_CURRENT_INSPECTION_PLAN);
}

void Map3d::sendCloudDimensions(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    _cloudDimensions.clear();
    _cloudDimensions.push_back(maxPt.x - minPt.x);
    _cloudDimensions.push_back(maxPt.y - minPt.y);
    _cloudDimensions.push_back(maxPt.z - minPt.z);

    // /// \note. Update plane floor dimension to cloud size. It's neccesary??
    // if(_floorShape)
    //     _floorShape->setScale(Ogre::Vector3(_cloudDimensions[0], _cloudDimensions[0], 0.001));

    pcl::computeCentroid(*cloud, _cloudCentroid);

    auto dtoToCore            = createSharedDTO<CloudDimensionsDTO>();
    dtoToCore->getNumPoints() = cloud->points.size();
    dtoToCore->getXmin()      = minPt.x;
    dtoToCore->getYmin()      = minPt.y;
    dtoToCore->getZmin()      = minPt.z;
    dtoToCore->getXmax()      = maxPt.x;
    dtoToCore->getYmax()      = maxPt.y;
    dtoToCore->getZmax()      = maxPt.z;

    Q_EMIT sendDTOToCore(ActionType::UPDATE_CLOUD_DIMENSIONS, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::restorePointCloud()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_assetCloud) {
        sendConsoleMsg(MsgType::WARNING, "Received asset cloud is empty!");
        return;
    }

    sendCloudDimensions(_assetCloud);
    _pcdDisplay->reset();
    _pcdDisplay->configureVisualization(_mapDTO.getVisualizationConfig());
    _pcdDisplay->attachData(_assetCloud);
}

void Map3d::hidePointCloud()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_assetCloud) {
        sendConsoleMsg(MsgType::WARNING, "Received asset cloud is empty!");
        return;
    }

    _pcdDisplay->reset();
}

void Map3d::hideCutPlane()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_cutCloudShape) {
        _cutCloudShape.reset();
    }
}

void Map3d::hidePreviewAxis()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_headingPreviewAxe) {
        _headingPreviewAxe->getSceneNode()->setVisible(false);
    }
}

void Map3d::editPointCloud(const QSharedPointer<CloudEditionParametersDTO>& editionParams)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto planeCutDir   = editionParams->getPlaneCutDirection();
    const auto planeCutValue = editionParams->getPlaneCutValue();
    const auto cutAxis       = editionParams->getPlaneCutAxis();

    if (editionParams->getApplyCut()) {
        auto                                   cloudEditor = std::make_unique<CloudEditionWorker>();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloudEditor->cutPointCloudByPlane(_assetCloud, cutAxis, planeCutDir, planeCutValue, cutted_cloud);
        if (!(cutted_cloud->points.size() > 0)) {
            sendConsoleMsg(MsgType::WARNING, "The point cloud resulting from the cut is empty");
            return;
        }
        _pcdDisplay->reset();
        _pcdDisplay->configureVisualization(_mapDTO.getVisualizationConfig());
        _pcdDisplay->attachData(cutted_cloud);
        return;
    }

    drawCuttingPlane(planeCutValue, cutAxis);
}

void Map3d::drawCuttingPlane(const float& plane_offset, const Axis& plane_axis)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Ogre::Vector3 position   = Ogre::Vector3(_cloudCentroid.x, _cloudCentroid.y, _cloudCentroid.z);
    Ogre::Vector3 dimensions = Ogre::Vector3(_cloudDimensions[0], _cloudDimensions[1], _cloudDimensions[2]);
    const float   plane_thickness{0.05f};
    const auto    axisIndex = static_cast<uint8_t>(plane_axis);
    if (axisIndex > 2) {
        sendConsoleMsg(MsgType::WARNING, "Invalis axis index in plane cut");
        return;
    }

    position[axisIndex]   = plane_offset;
    dimensions[axisIndex] = plane_thickness;

    if (_cutCloudShape) {
        _cutCloudShape.reset();
    }

    _cutCloudShape = std::make_unique<rviz::Shape>(rviz::Shape::Cube, _visualizationManager->getSceneManager());
    _cutCloudShape->setScale(dimensions); // (depth, width, height)
    _cutCloudShape->setColor(gcsColors::tBlue);
    _cutCloudShape->setPosition(position);
}

void Map3d::addInspectionTasks(const QList<InspectionTaskDTO>& inspTaskList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    resetInterestVector(_inspectionTasks);
    resetInterestVector(_vehicleSafetyAreas);

    for (const auto& inspTask : inspTaskList) {
        const auto    inspTaskType = inspTask.getLocationType();
        Ogre::Vector3 position(
                inspTask.getPosition().getX(), inspTask.getPosition().getY(), inspTask.getPosition().getZ());
        if (inspTaskType == InspectionTaskLocationType::POINT) {
            const auto inspPtSizeScale = _mapDTO.getVisualizationConfig().getInspPtSizeScale();
            addInspectionPoint(inspTask.getName().toStdString(), position, 0.5f * inspPtSizeScale);
        } else if (inspTaskType == InspectionTaskLocationType::AREA) {
            const auto orientation = Ogre::Quaternion(
                    inspTask.getOrientation().getW(),
                    inspTask.getOrientation().getX(),
                    inspTask.getOrientation().getY(),
                    inspTask.getOrientation().getZ());
            addInspectionArea(
                    inspTask.getName().toStdString(),
                    position,
                    orientation,
                    inspTask.getWidth(),
                    inspTask.getHeight(),
                    inspTask.getDepth());
        } else {
            QLOG_ERROR() << __PRETTY_FUNCTION__
                         << "Inspection task type not supported: " << static_cast<int>(inspTaskType);
        }

        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Position: (" << position[0] << "," << position[1] << "," << position[2]
                     << ")"
                     << "Type: " << ToString(inspTaskType);
    }
}

void Map3d::addExclusionArea(const Ogre::Vector3& position, const Ogre::Vector3& dimensions)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    rviz::Shape* desired_area = new rviz::Shape(rviz::Shape::Cube, _visualizationManager->getSceneManager());
    desired_area->setScale(dimensions); // (depth, width, height)
    desired_area->setColor(gcsColors::tRed);
    desired_area->setPosition(Ogre::Vector3(position[0], position[1], position[2] + (dimensions[2] / 2.0f)));
    const Ogre::Quaternion orient_z = Ogre::Quaternion(Ogre::Degree(55), Ogre::Vector3::UNIT_Z);
    desired_area->setOrientation(orient_z);

    rviz::MovableText* area_text = new rviz::MovableText(
            "exclusion_" + std::to_string(_exclusionAreas.size()), "Liberation Sans", 0.6, Ogre::ColourValue::White);
    area_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);

    Ogre::SceneNode* area_text_node
            = _visualizationManager->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    area_text_node->setPosition(
            Ogre::Vector3(position[0], position[1], position[2] + (dimensions[2] / 2.0f) + (dimensions[2] * 0.6f)));
    area_text_node->setVisible(true);
    area_text_node->attachObject(area_text);

    _exclusionAreas.insert({desired_area, area_text_node});
}

void Map3d::addSafetyArea(const Ogre::Vector3& position, const float side)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const float  area_height  = 0.05f;
    rviz::Shape* desired_area = new rviz::Shape(rviz::Shape::Cube, _visualizationManager->getSceneManager());
    desired_area->setScale(Ogre::Vector3(side, side, area_height));
    desired_area->setColor(gcsColors::tGreen);
    desired_area->setPosition(Ogre::Vector3(position[0], position[1], position[2] + (area_height / 2.0f)));

    rviz::MovableText* area_text = new rviz::MovableText(
            "safety_" + std::to_string(_vehicleSafetyAreas.size()), "Liberation Sans", 0.6, Ogre::ColourValue::White);
    area_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);

    Ogre::SceneNode* area_text_node
            = _visualizationManager->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    area_text_node->setPosition(
            Ogre::Vector3(position[0], position[1], position[2] + (area_height / 2.0f) + (area_height * 0.6f)));
    area_text_node->setVisible(true);
    area_text_node->attachObject(area_text);

    _vehicleSafetyAreas.insert({desired_area, area_text_node});
}

void Map3d::addInspectionArea(
        const std::string&      inspTaskName,
        const Ogre::Vector3&    position,
        const Ogre::Quaternion& orientation,
        const float             width,
        const float             height,
        const float             depth)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    rviz::Shape* desired_area = new rviz::Shape(rviz::Shape::Cube, _visualizationManager->getSceneManager());
    desired_area->setScale(Ogre::Vector3(width, depth, height));
    desired_area->setColor(
            Ogre::ColourValue(1.0f, 1.0f, 0.0f, _mapDTO.getVisualizationConfig().getInspLocationTransp()));
    desired_area->setPosition(Ogre::Vector3(position[0], position[1], position[2] /* + (height/2.0f)*/));
    desired_area->setOrientation(orientation);

    rviz::MovableText* area_text
            = new rviz::MovableText(inspTaskName, "Liberation Sans", 0.4, Ogre::ColourValue::White);
    area_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);

    Ogre::SceneNode* area_text_node
            = _visualizationManager->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    area_text_node->setPosition(
            Ogre::Vector3(position[0], position[1], position[2] + (height / 2.0f) + (height * 0.5f)));
    area_text_node->setVisible(true);

    if (_mapDTO.getVisualizationConfig().getShowItemsText()) {
        area_text_node->attachObject(area_text);
    }

    _inspectionTasks.push_back(std::make_pair(desired_area, area_text_node));
}

void Map3d::addInspectionPoint(const std::string& inspTaskName, const Ogre::Vector3& position, const float radius)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    rviz::Shape* desired_pt = new rviz::Shape(rviz::Shape::Sphere, _visualizationManager->getSceneManager());
    desired_pt->setScale(Ogre::Vector3(radius, radius, radius));
    desired_pt->setColor(Ogre::ColourValue(1.0f, 1.0f, 0.0f, _mapDTO.getVisualizationConfig().getInspLocationTransp()));
    desired_pt->setPosition(Ogre::Vector3(position[0], position[1], position[2]));

    rviz::MovableText* pt_text = new rviz::MovableText(inspTaskName, "Liberation Sans", 0.4, Ogre::ColourValue::White);
    pt_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);

    Ogre::SceneNode* pt_text_node
            = _visualizationManager->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    pt_text_node->setPosition(Ogre::Vector3(position[0], position[1], position[2] + 0.3f));
    pt_text_node->setVisible(false);

    if (_mapDTO.getVisualizationConfig().getShowItemsText()) {
        pt_text_node->attachObject(pt_text);
    }

    _inspectionTasks.push_back(std::make_pair(desired_pt, pt_text_node));
}

void Map3d::manageDirectionArrow(Ogre::Vector3 position, float angle, bool visible)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_directionArrow) {
        _directionArrow = std::make_unique<rviz::Arrow>(_visualizationManager->getSceneManager());
    }

    /// \note. Arrow dimensions
    _directionArrow->set(0.3f, 0.05, 0.15f, 0.1f);

    const auto orientBase = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
    const auto orient     = Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z);
    _directionArrow->setPosition(position);
    _directionArrow->setOrientation(orient * orientBase);

    _directionArrow->getSceneNode()->setVisible(visible);

    _arrowVisibilityTimer.start();
}

void Map3d::manageDirectionArrowVisibility()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _directionArrow->getSceneNode()->setVisible(false);

    if (_arrowVisibilityTimer.isActive()) {
        _arrowVisibilityTimer.stop();
    }
}

void Map3d::updateViewInspectionTask(const QString& task_uuid)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (_inspectionTasks.size() == 0) {
            throw std::runtime_error("Empty inspection tasks");
        }

        for (const auto& task : _inspectionTasks) {
            task.first->setColor(
                    Ogre::ColourValue(1.0f, 1.0f, 0.0f, _mapDTO.getVisualizationConfig().getInspLocationTransp()));
        }

        if (!task_uuid.isEmpty()) {
            const auto it = std::find_if(
                    _mapDTO.getInspectionTaskList().cbegin(),
                    _mapDTO.getInspectionTaskList().cend(),
                    [&task_uuid](const auto& task) { return task.getUUID() == task_uuid; });

            if (it != _mapDTO.getInspectionTaskList().cend()) {
                const auto task_idx = std::distance(_mapDTO.getInspectionTaskList().cbegin(), it);
                _inspectionTasks[task_idx].first->setColor(gcsColors::yellow);
            }
        }
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
    }
}

bool Map3d::computeClickedPointNormal(const float& x, const float& y, const float& z, Ogre::Quaternion& normalQuat)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    pcl::PointXYZRGB searchPoint;
    searchPoint.x = x;
    searchPoint.y = y;
    searchPoint.z = z;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(_assetCloud);
    const float        radius = 0.1;
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    float                                                curvature;
    Eigen::Vector4f                                      plane_coeffs;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.computePointNormal(*_assetCloud, pointIdxRadiusSearch, plane_coeffs, curvature);

    if (plane_coeffs.hasNaN()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Some value of normals are NaN";
        return false;
    }

    const float sideSign = plane_coeffs[0] * searchPoint.x + plane_coeffs[1] * searchPoint.y
                         + plane_coeffs[2] * searchPoint.z + plane_coeffs[3];

    auto normalVector = Ogre::Vector3(plane_coeffs[0], plane_coeffs[1], plane_coeffs[2]);

    if (sideSign > 0) {
        normalVector = -normalVector;
    }
    normalQuat = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(normalVector);

    // QLOG_INFO() << __PRETTY_FUNCTION__
    //             << "Normal vector: (" << normalVector[0] << ", " << normalVector[1] << ", " << normalVector[2] << ")"
    //             << " Curvature: " << curvature << "\n"
    //             << "Roll: " << quat.getRoll().valueDegrees()
    //             << " Pitch: " << quat.getPitch().valueDegrees()
    //             << " Yaw: " << quat.getYaw().valueDegrees();

    return true;
}

void Map3d::cursorPosition(QVector3D cursor)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dtoToCore    = createSharedDTO<CursorPositionDTO>();
    dtoToCore->getX() = static_cast<float>(cursor.x());
    dtoToCore->getY() = static_cast<float>(cursor.y());
    dtoToCore->getZ() = static_cast<float>(cursor.z());

    if (_mapDTO.getInEditionMode()) {
        Q_EMIT sendDTOToCore(ActionType::UPDATE_CURSOR_POSITION, QSharedPointer<IDTO>(dtoToCore));
    }
}

void Map3d::addWaypoint(QVector3D clickedPosition)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const bool mapInEditionMode2D = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::TwoD);

    auto quat = Ogre::Quaternion(1.0, 0.0, 0.0, 0.0);
    if (!mapInEditionMode2D) {
        computeClickedPointNormal(clickedPosition.x(), clickedPosition.y(), clickedPosition.z(), quat);
    }

    auto dtoToCore                     = createSharedDTO<PoseDTO>();
    dtoToCore->getPosition().getX()    = clickedPosition.x();
    dtoToCore->getPosition().getY()    = clickedPosition.y();
    dtoToCore->getPosition().getZ()    = mapInEditionMode2D ? _mapDTO.getDefaultAltitude() : clickedPosition.z();
    dtoToCore->getOrientation().getW() = quat.w;
    dtoToCore->getOrientation().getX() = quat.x;
    dtoToCore->getOrientation().getY() = quat.y;
    dtoToCore->getOrientation().getZ() = quat.z;

    Q_EMIT sendDTOToCore(ActionType::ADD_WAYPOINT, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::editWaypoint(QSharedPointer<WaypointDTO> waypoint)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Q_EMIT sendDTOToCore(ActionType::EDIT_WAYPOINT, QSharedPointer<IDTO>(waypoint));
}

void Map3d::addHome(QPointF clickedPosition)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dtoToCore                  = createSharedDTO<HomeDTO>();
    dtoToCore->getIsEnabled()       = true;
    dtoToCore->getPosition().getX() = static_cast<float>(clickedPosition.x());
    dtoToCore->getPosition().getY() = static_cast<float>(clickedPosition.y());
    dtoToCore->getPosition().getZ() = _mapDTO.getLoadedHome().getPosition().getZ();
    dtoToCore->getYawOrientation()  = _mapDTO.getLoadedHome().getYawOrientation();

    Q_EMIT sendDTOToCore(ActionType::UPDATE_LOADED_HOME, QSharedPointer<IDTO>(dtoToCore));

    /// \note. Change home visibility if change its position
    auto visualizationConfigDTO                            = createSharedDTO<VisualizationConfigDTO>();
    _mapDTO.getVisualizationConfig().getShowHomePosition() = true;
    *visualizationConfigDTO                                = _mapDTO.getVisualizationConfig();

    Q_EMIT sendDTOToCore(ActionType::UPDATE_VISUALIZATION_CONFIG, QSharedPointer<IDTO>(visualizationConfigDTO));
}

void Map3d::addMeasuringPoint(QVector3D clickedPosition)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dtoToCore    = createSharedDTO<Position3dDTO>();
    dtoToCore->getX() = static_cast<float>(clickedPosition.x());
    dtoToCore->getY() = static_cast<float>(clickedPosition.y());
    dtoToCore->getZ() = static_cast<float>(clickedPosition.z());

    Q_EMIT sendDTOToCore(ActionType::ADD_MEASURING_POINT, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::changeSelectedWaypoint(quint16 waypointId)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "- WaypointId:" << waypointId;

    auto dtoToCore     = createSharedDTO<WaypointDTO>();
    dtoToCore->getId() = waypointId;

    Q_EMIT sendDTOToCore(ActionType::CHANGE_SELECTED_WAYPOINT, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::removeWaypoint(quint16 waypointId)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "- WaypointId:" << waypointId;

    /// \note. Remove every ocurrences of the route
    _mapDTO.getWaypointsList().erase(
            std::remove_if(
                    _mapDTO.getWaypointsList().begin(),
                    _mapDTO.getWaypointsList().end(),
                    [&](const QSharedPointer<Waypoint3dItem> wp) { return wp->id() == waypointId; }),
            _mapDTO.getWaypointsList().end());

    auto dtoToCore     = createSharedDTO<WaypointDTO>();
    dtoToCore->getId() = waypointId;

    Q_EMIT sendDTOToCore(ActionType::REMOVE_WAYPOINT, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::addRoute(QVector3D clickedPosition)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const bool mapInEditionMode2D = (_mapDTO.getInEditionMode() && _mapDTO.getViewMode() == ViewModeType::TwoD);

    WaypointsOptionsDialog dialog(nullptr, mapInEditionMode2D);
    dialog.setDefaultAltitude(_mapDTO.getDefaultAltitude());
    dialog.setInspectionTaskList(_mapDTO.getInspectionTaskList());
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    _mapDTO.getDefaultAltitude() = dialog.getDefaultAltitude();

    auto quat = Ogre::Quaternion(1.0, 0.0, 0.0, 0.0);
    if (!mapInEditionMode2D) {
        computeClickedPointNormal(clickedPosition.x(), clickedPosition.y(), clickedPosition.z(), quat);
    }

    auto dtoToCore                     = createSharedDTO<WaypointDTO>();
    dtoToCore->getTaskUUID()           = dialog.getInspectionTaskUUID();
    dtoToCore->getPosition().getX()    = clickedPosition.x();
    dtoToCore->getPosition().getY()    = clickedPosition.y();
    dtoToCore->getPosition().getZ()    = mapInEditionMode2D ? _mapDTO.getDefaultAltitude() : clickedPosition.z();
    dtoToCore->getOrientation().getW() = quat.w;
    dtoToCore->getOrientation().getX() = quat.x;
    dtoToCore->getOrientation().getY() = quat.y;
    dtoToCore->getOrientation().getZ() = quat.z;

    Q_EMIT sendDTOToCore(ActionType::ADD_ROUTE, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::removeRoute(quint16 routeId)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "- RouteId:" << routeId;

    /// \note. Remove every ocurrences of the route
    _mapDTO.getWaypointsList().erase(
            std::remove_if(
                    _mapDTO.getWaypointsList().begin(),
                    _mapDTO.getWaypointsList().end(),
                    [&](const QSharedPointer<Waypoint3dItem> wp) { return wp->routeId() == routeId; }),
            _mapDTO.getWaypointsList().end());

    auto dtoToCore     = createSharedDTO<RouteDTO>();
    dtoToCore->getId() = routeId;

    Q_EMIT sendDTOToCore(ActionType::REMOVE_ROUTE, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::sendCurrentRoute()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. Commented action
    // Q_EMIT sendDTOToCore(ActionType::SEND_ROUTE);
}

void Map3d::copyLoadedRouteToLocal()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Q_EMIT sendDTOToCore(ActionType::COPY_LOADED_ROUTE_TO_LOCAL);
}

void Map3d::editHomeParameters()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    HomeParametersDialog dialog(dynamic_cast<QWidget*>(this));
    dialog.setHomeParameters(_mapDTO.getLoadedHome());
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    auto dtoToCore                  = createSharedDTO<HomeDTO>();
    dtoToCore->getIsEnabled()       = true;
    dtoToCore->getPosition().getX() = dialog.getPositionX();
    dtoToCore->getPosition().getY() = dialog.getPositionY();
    dtoToCore->getPosition().getZ() = dialog.getPositionZ();
    dtoToCore->getYawOrientation()  = dialog.getYawAngle();

    _mapDTO.getLoadedHome() = *dtoToCore;

    Q_EMIT sendDTOToCore(ActionType::UPDATE_LOADED_HOME, QSharedPointer<IDTO>(dtoToCore));

    /// \note. Change home visibility if change its position
    auto visualizationConfigDTO                            = createSharedDTO<VisualizationConfigDTO>();
    _mapDTO.getVisualizationConfig().getShowHomePosition() = true;
    *visualizationConfigDTO                                = _mapDTO.getVisualizationConfig();

    Q_EMIT sendDTOToCore(ActionType::UPDATE_VISUALIZATION_CONFIG, QSharedPointer<IDTO>(visualizationConfigDTO));
}

void Map3d::addActionWpOverInspTask(QString inspectionTaskUUID)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    InspectionTaskActionInfoDialog dialog(dynamic_cast<QWidget*>(this));

    const auto it = std::find_if(
            _mapDTO.getInspectionTaskList().cbegin(),
            _mapDTO.getInspectionTaskList().cend(),
            [&inspectionTaskUUID](const auto& task) { return task.getUUID() == inspectionTaskUUID; });

    if (it == _mapDTO.getInspectionTaskList().cend()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - "
                    << "Inspection Task with UUID: " << inspectionTaskUUID << " not found";
        return;
    }

    const auto selectedInspectionTask = *it;
    dialog.setInspTaskActionName(selectedInspectionTask.getType().getName());
    dialog.setInspTaskActionUUID(selectedInspectionTask.getType().getUUID());
    dialog.setInspTaskActionNotes(selectedInspectionTask.getType().getDescription());
    dialog.setInspTaskActionProperties(selectedInspectionTask.getType().getProperties());
    if (dialog.exec() == QDialog::Accepted) {
        auto dtoToCore                  = createSharedDTO<WaypointDTO>();
        dtoToCore->getTaskUUID()        = selectedInspectionTask.getUUID();
        dtoToCore->getWpType()          = WaypointType::ACTION;
        dtoToCore->getPosition().getX() = selectedInspectionTask.getPosition().getX();
        dtoToCore->getPosition().getY() = selectedInspectionTask.getPosition().getY();
        dtoToCore->getPosition().getZ() = selectedInspectionTask.getPosition().getZ();

        if (selectedInspectionTask.getLocationType() == InspectionTaskLocationType::AREA) {
            dtoToCore->getOrientation() = selectedInspectionTask.getOrientation();
        }

        Q_EMIT sendDTOToCore(ActionType::ADD_WAYPOINT, QSharedPointer<IDTO>(dtoToCore));
    }

    /// \note. Reset inspection task highlight
    updateViewInspectionTask("");
}

void Map3d::addPoseWpOverInspTask(QString inspectionTaskUUID)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto it = std::find_if(
            _mapDTO.getInspectionTaskList().cbegin(),
            _mapDTO.getInspectionTaskList().cend(),
            [&inspectionTaskUUID](const auto& task) { return task.getUUID() == inspectionTaskUUID; });

    if (it == _mapDTO.getInspectionTaskList().cend()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - "
                    << "Inspection Task with UUID: " << inspectionTaskUUID << " not found";
        return;
    }

    QVector3D position(it->getPosition().getX(), it->getPosition().getY(), it->getPosition().getZ());
    addWaypoint(position);
}

void Map3d::editWaypointHeading()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dtoToCore             = createSharedDTO<DialogDTO>();
    dtoToCore->getDialogType() = DialogType::WaypointHeadingEdition;
    Q_EMIT sendDTOToCore(ActionType::SHOW_DIALOG, QSharedPointer<IDTO>(dtoToCore));

    const auto it = std::find_if(
            _mapDTO.getWaypointsList().cbegin(),
            _mapDTO.getWaypointsList().cend(),
            [&](const QSharedPointer<Waypoint3dItem> wp) { return wp->isSelectedWaypoint(); });
    if (it == _mapDTO.getWaypointsList().cend()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - "
                    << "Waypoint not found";
        return;
    }

    const auto index  = std::distance(_mapDTO.getWaypointsList().cbegin(), it);
    const auto currWp = _waypointsHelper.at(index).get();

    if (!_headingPreviewAxe) {
        return;
    }

    _headingPreviewAxe->setPosition(currWp->getPosition());
    _headingPreviewAxe->setOrientation(currWp->getOrientation());
    _baseHeading = _headingPreviewAxe->getOrientation();
    _headingPreviewAxe->getSceneNode()->setVisible(true);
}

void Map3d::modifyPreviewWpHeading(const float& increment)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_headingPreviewAxe) {
        return;
    }

    _headingPreviewAxe->setOrientation(_baseHeading);
    _headingPreviewAxe->getSceneNode()->roll(Ogre::Degree(increment));
    _headingPreviewAxe->getSceneNode()->setVisible(true);
}

void Map3d::modifySelectedWpHeading(const float& heading)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    const auto it = std::find_if(
            _mapDTO.getWaypointsList().cbegin(),
            _mapDTO.getWaypointsList().cend(),
            [&](const QSharedPointer<Waypoint3dItem> wp) { return wp->isSelectedWaypoint(); });
    if (it == _mapDTO.getWaypointsList().cend()) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - "
                    << "No waypoint selected";
        return;
    }

    auto dtoToCore = createSharedDTO<WaypointDTO>();
    *dtoToCore     = (*it)->waypoint();

    if (!_headingPreviewAxe) {
        return;
    }

    _headingPreviewAxe->setOrientation(_baseHeading);
    _headingPreviewAxe->getSceneNode()->roll(Ogre::Degree(heading));
    _baseHeading = _headingPreviewAxe->getOrientation();

    dtoToCore->getPosition().getX()    = _headingPreviewAxe->getPosition().x;
    dtoToCore->getPosition().getY()    = _headingPreviewAxe->getPosition().y;
    dtoToCore->getPosition().getZ()    = _headingPreviewAxe->getPosition().z;
    dtoToCore->getOrientation().getX() = _headingPreviewAxe->getOrientation().x;
    dtoToCore->getOrientation().getY() = _headingPreviewAxe->getOrientation().y;
    dtoToCore->getOrientation().getZ() = _headingPreviewAxe->getOrientation().z;
    dtoToCore->getOrientation().getW() = _headingPreviewAxe->getOrientation().w;

    Q_EMIT sendDTOToCore(ActionType::EDIT_WAYPOINT, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::startProgressBar(const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dtoToCore                    = createSharedDTO<CommsProgressDTO>();
    dtoToCore->getCommsProgressType() = CommsProgressType::START;
    dtoToCore->getCommsProgressInfo() = msg;
    sendDTOToCore(ActionType::UPDATE_COMMS_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::finishProgressBar()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note. Wait a bit, for quick transactions where there is no time to have launched the START
    QTime dieTime = QTime::currentTime().addMSecs(50);
    while (QTime::currentTime() < dieTime) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }

    auto dtoToCore                    = createSharedDTO<CommsProgressDTO>();
    dtoToCore->getCommsProgressType() = CommsProgressType::STOP;
    sendDTOToCore(ActionType::UPDATE_COMMS_PROGRESS, QSharedPointer<IDTO>(dtoToCore));
}

void Map3d::sendConsoleMsg(const MsgType& type, const QString& msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
    msgConsoleDTO->getMsgType() = type;
    msgConsoleDTO->getMessage() = msg;
    Q_EMIT sendDTOToCore(ActionType::UPDATE_MSG_CONSOLE, QSharedPointer<IDTO>(msgConsoleDTO));
}

} // namespace gcs
