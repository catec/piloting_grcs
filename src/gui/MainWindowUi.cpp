#include "MainWindowUi.h"

#include <QsLog/QsLog.h>
#include <common/CommonDefines.h>
#include <communications/MavSDK/dtos/AlarmListDTO.h>
#include <communications/MavSDK/dtos/AlarmStatusDTO.h>
#include <communications/MavSDK/dtos/HLActionListDTO.h>
#include <config.h>
#include <core/dtos/BasicBoolDTO.h>
#include <core/dtos/BasicFloatDTO.h>
#include <core/dtos/BasicIntDTO.h>
#include <core/dtos/BasicStringDTO.h>
#include <core/dtos/BasicStringListDTO.h>
#include <dataModel/dtos/ApplicationStatusDTO.h>
#include <dataModel/dtos/AssetProgressDTO.h>
#include <dataModel/dtos/CloudDimensionsDTO.h>
#include <dataModel/dtos/CloudEditionParametersDTO.h>
#include <dataModel/dtos/CommsLinkDDHLDTO.h>
#include <dataModel/dtos/CommsLinkMavsdkDTO.h>
#include <dataModel/dtos/CommsProgressDTO.h>
#include <dataModel/dtos/CommsStatusDTO.h>
#include <dataModel/dtos/CurrentMissionResultDTO.h>
#include <dataModel/dtos/CursorPositionDTO.h>
#include <dataModel/dtos/DialogDTO.h>
#include <dataModel/dtos/EditionModeDTO.h>
#include <dataModel/dtos/InspectionPlanDTO.h>
#include <dataModel/dtos/MeasureDTO.h>
#include <dataModel/dtos/MissionDTO.h>
#include <dataModel/dtos/MissionStatusDTO.h>
#include <dataModel/dtos/MsgConsolePopupDTO.h>
#include <dataModel/dtos/RouteOptionsDTO.h>
#include <dataModel/dtos/SiteListDTO.h>
#include <dataModel/dtos/SiteTypeDTO.h>
#include <dataModel/dtos/ViewModeDTO.h>
#include <dataModel/dtos/VisualizationConfigDTO.h>
#include <dialogs/checkList/CheckListDialog.h>
#include <dialogs/checkListInfo/CheckListInfoDialog.h>
#include <dialogs/commsOptionsDDHL/CommsOptionsDDHLDialog.h>
#include <dialogs/commsOptionsMavsdk/CommsOptionsMavsdkDialog.h>
#include <dialogs/connectionConfirm/ConnectionConfirmationDialog.h>
#include <dialogs/downloadAssetFile/DownloadAssetFileDialog.h>
#include <dialogs/downloadDataDDHL/DownloadDataDDHLDialog.h>
#include <dialogs/manageAutoHeading/ManageAutoHeadingDialog.h>
#include <dialogs/selectInspecionPlanFromSite/SelectInspecionPlanFromSiteDialog.h>
#include <dialogs/uploadDDHLManager/UploadDDHLManagerDialog.h>
#include <dialogs/visualizationConfig/VisualizationConfigDialog.h>
#include <dialogs/waypointHeadingEditor/WaypointHeadingEditorDialog.h>

#include <QCloseEvent>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QLabel>
#include <QMessageBox>
#include <QSettings>
#include <QStatusBar>
#include <iostream>

#include "ui_MainWindowUi.h"

namespace gcs {
MainWindowUi::MainWindowUi(QWidget* parent) :
        QMainWindow(parent),
        _ui(std::make_unique<Ui::MainWindowUi>()),
        _robotControlDW(tr("Robots Control"), this),
        _waypointsEditorDW(tr("Waypoints Editor"), this),
        _robotAlarmsDW(tr("Robot Alarms"), this),
        _consoleDW(tr("Console"), this),
        _inspectionPlanDW(tr("Inspection Plan"), this),
        _missionTasksDW(tr("Mission"), this),
        _telemetryTreeDW(tr("Telemetry"), this),
        _highLevelActionsDW(tr("High Level Actions"), this),
        _robotsControlWidget(this),
        _robotAlarmsWidget(this),
        _telemetryTreeWidget(_guiDTO, this),
        _consoleDisplayWidget(_guiDTO, this),
        _waypointEditorWidget(this),
        _inspectionPlanWidget(this),
        _missionWidget(_guiDTO, this),
        _highLevelActionsWidget(this),
        _commsProgressDialog(this),
        _cloudEditorDialog(this),
        _assetDownloadprogressDialog(trUtf8("Asset file download"), trUtf8("Cancel"), 0, 100, this)
{
    QLOG_TRACE() << "MainWindowUi::MainWindowUi()";

    _ui->setupUi(this);

    /// \note This is done by not allowing hide the toolbars
    setContextMenuPolicy(Qt::PreventContextMenu);

    /// \note LeftDockWidgetArea
    setTelemetryTreeDock(Qt::LeftDockWidgetArea);
    setInspectionPlanDock(Qt::LeftDockWidgetArea);

    /// \note RightDockWidgetArea
    setRobotAlarmsDock(Qt::RightDockWidgetArea);
    setRobotControlDock(Qt::RightDockWidgetArea);
    setMissionDock(Qt::RightDockWidgetArea);
    setHighLevelActionsDock(Qt::RightDockWidgetArea);

    /// \note BottomDockWidgetArea
    setConsoleDock(Qt::BottomDockWidgetArea);
    setWaypointsEditorDock(Qt::BottomDockWidgetArea);
    setAssetProgressDialog();

    loadGeometry();
    loadMavsdkCommsOptions();
    loadTargetUUID();

    connectActions();

    updateMissionName();
    updateMissionButtons();
    updateConnectionStatus();
    updateEditionModeWidgets();
    updateAppStatusButtons();
    updateRouteOptionsButtons();
    updateViewModeButtons();

    statusBar()->showMessage("");

    auto version = QString("| v%1 ").arg(QString(PROJECT_VERSION));

    auto versionLabel = new QLabel(this);
    versionLabel->setText(version);
    statusBar()->addPermanentWidget(versionLabel);
}

MainWindowUi::~MainWindowUi()
{
    QLOG_TRACE() << "MainWindowUi::~MainWindowUi()";
}

void MainWindowUi::setupInit(QWidget* mapView)
{
    _ui->mapW->layout()->addWidget(mapView);

    loadMissionResultsFromFolder();
    loadXMLDataFromFolder();
    loadDDHLOptions();

    /// \note. Initialize autoheading mode value
    auto dto       = createSharedDTO<BasicBoolDTO>();
    dto->getBool() = _guiDTO.getAutoHeadingMode();
    Q_EMIT sendDTOToCore(ActionType::MANAGE_AUTOHEADING_WPTS, QSharedPointer<IDTO>(dto));

    show();

    loadGeometry();
    loadVisualizationOptions();
}

void MainWindowUi::manageCoreDTO(QSharedPointer<IDTO> idto)
{
    if (!idto) {
        return;
    }

    switch (idto->getDTOType()) {
        case DTOType::MSG_CONSOLE:
        case DTOType::MSG_CONSOLE_POPUP: {
            auto dto = idto.dynamicCast<MsgConsoleDTO>();
            if (dto) {
                showMessage(dto);
            }

            break;
        }
        case DTOType::DIALOG: {
            auto dto = idto.dynamicCast<DialogDTO>();
            if (dto) {
                if (dto->getDialogType() == DialogType::WaypointHeadingEdition) {
                    WaypointHeadingEditorDialog dialog(this);

                    // clang-format off
                    connect(&dialog, &WaypointHeadingEditorDialog::modifyPreviewWp,
                            this,    &MainWindowUi::modifyPreviewWpHeading);
     
                    connect(&dialog, &WaypointHeadingEditorDialog::modifyCurrentWp,
                            this,    &MainWindowUi::modifyCurrentWpHeading);
                    // clang-format on

                    dialog.exec();

                    /// \note. Hide the axis
                    Q_EMIT sendDTOToCore(ActionType::HIDE_PREVIEW_AXIS);
                }
            }

            break;
        }
        case DTOType::MISSION: {
            auto dto = idto.dynamicCast<MissionDTO>();
            if (dto) {
                _guiDTO.getHasRoutes()  = !dto->getLocalRoutesMap().isEmpty();
                _guiDTO.getLoadedHome() = dto->getLoadedHome();

                if (_guiDTO.getLoadedRoute() != dto->getLoadedRoute()) {
                    _guiDTO.getLoadedRoute() = dto->getLoadedRoute();
                }

                _robotsControlWidget.updateLoadedRoute(_guiDTO.getLoadedRoute());
                _waypointEditorWidget.updateMission(*dto);
                updateMissionButtons();
            }

            break;
        }
        case DTOType::MISSION_STATUS: {
            auto dto = idto.dynamicCast<MissionStatusDTO>();
            if (dto) {
                _guiDTO.getMissionStatus() = dto->getStatus();

                updateMissionName();
                updateMissionButtons();
            }

            break;
        }
        case DTOType::CURRENT_MISSION_RESULT: {
            auto dto = idto.dynamicCast<CurrentMissionResultDTO>();
            if (dto) {
                _guiDTO.getMissionName()           = dto->getName();
                _guiDTO.getMissionCreationDate()   = dto->getCreationTimeStamp();
                _guiDTO.getMissionLastUpdateDate() = dto->getLastUpdateTimeStamp();
                _guiDTO.getMissionDescription()    = dto->getDescription();
                _guiDTO.getCheckList()             = dto->getCheckListDTO();

                _missionWidget.setMissionTasks(dto->getMissionTaskList());

                _consoleDisplayWidget.updateLogging(dto->getConsoleLogFilePath());
                _telemetryTreeWidget.updateLogging(dto->getTelemetryLogFilePath());

                updateMissionName();
            }

            break;
        }
        case DTOType::MISSION_RESULTS: {
            auto dto = idto.dynamicCast<MissionResultsDTO>();
            if (dto) {
                _guiDTO.getMissionResults() = dto;
            }
            break;
        }
        case DTOType::APPLICATION_STATUS: {
            auto dto = idto.dynamicCast<ApplicationStatusDTO>();
            if (dto) {
                _guiDTO.getAppStatus() = dto->getStatus();

                updateAppStatusButtons();

                /// \note. Show home position when the application is in EditingLoadedHome status
                ///        and update visualization configuration.
                if (_guiDTO.getAppStatus() == AppStatusType::EditingLoadedHome) {
                    _guiDTO.getVisualizationConfig().getShowHomePosition() = true;
                    auto dto                                               = createSharedDTO<VisualizationConfigDTO>();
                    *dto                                                   = _guiDTO.getVisualizationConfig();

                    Q_EMIT sendDTOToCore(ActionType::UPDATE_VISUALIZATION_CONFIG, QSharedPointer<IDTO>(dto));
                }
            }

            break;
        }
        case DTOType::EDITION_MODE: {
            auto dto = idto.dynamicCast<EditionModeDTO>();
            if (dto) {
                _ui->editModeAct->setChecked(dto->getMode());

                updateEditionModeWidgets();
                updateMissionButtons();
            }

            break;
        }
        case DTOType::VIEW_MODE: {
            QLOG_DEBUG() << __PRETTY_FUNCTION__ << " - DTOType: VIEW_MODE";

            auto dto = idto.dynamicCast<ViewModeDTO>();
            if (dto) {
                _guiDTO.getViewMode() = dto->getMode();

                updateViewModeButtons();
            }
            break;
        }
        case DTOType::ROUTE_OPTIONS: {
            auto dto = idto.dynamicCast<RouteOptionsDTO>();
            if (dto) {
                _guiDTO.getRouteOptions() = dto->getFlag();

                updateRouteOptionsButtons();
            }

            break;
        }
        case DTOType::COMMS_PROGRESS: {
            auto dto = idto.dynamicCast<CommsProgressDTO>();
            if (dto) {
                switch (dto->getCommsProgressType()) {
                    case CommsProgressType::START: {
                        if (!_commsProgressDialog.isVisible()) {
                            _commsProgressDialog.setInfo(dto->getCommsProgressInfo());
                            _commsProgressDialog.show();
                        }

                        break;
                    }
                    case CommsProgressType::STOP: {
                        if (_commsProgressDialog.isVisible()) {
                            _commsProgressDialog.close();
                        }

                        break;
                    }
                    default: {
                        QLOG_WARN() << "MainWindowUi::manageCoreDTO() -"
                                    << QString("Unknown comms progress type: %1")
                                               .arg(static_cast<uint8_t>(dto->getCommsProgressType()));
                        ;

                        break;
                    }
                }
            }

            break;
        }
        case DTOType::CURSOR_POSITION: {
            auto dto = idto.dynamicCast<CursorPositionDTO>();
            if (dto) {
                statusBar()->showMessage(QString("Position X: %1, Position Y: %2, Position Z: %3")
                                                 .arg(static_cast<double>(dto->getX()), 0, 'f', 3)
                                                 .arg(static_cast<double>(dto->getY()), 0, 'f', 3)
                                                 .arg(static_cast<double>(dto->getZ()), 0, 'f', 3));
            }

            break;
        }
        case DTOType::MEASURE: {
            auto dto = idto.dynamicCast<MeasureDTO>();
            if (dto) {
                const auto& measureList = dto->getPointList();
                if (measureList.size() == 2) {
                    const auto distance = calculateMetersDistance(
                            measureList[0].getX(),
                            measureList[0].getY(),
                            measureList[0].getZ(),
                            measureList[1].getX(),
                            measureList[1].getY(),
                            measureList[1].getZ());

                    displayMsgBox(
                            MsgType::INFO,
                            tr("Distance calculated"),
                            QString("The distance between points is: %1 m")
                                    .arg(static_cast<double>(distance), 0, 'f', 4));
                }
            }

            break;
        }
        case DTOType::COMMS_STATUS: {
            auto dto = idto.dynamicCast<CommsStatusDTO>();
            if (dto) {
                if (_guiDTO.getCommsIsConnected() != dto->getIsConnected()) {
                    _guiDTO.getCommsIsConnected() = dto->getIsConnected();

                    updateConnectionStatus();
                    updateMissionButtons();
                }
            }

            break;
        }
        case DTOType::ASSET_PROGRESS: {
            auto assetProgressDTO = idto.dynamicCast<AssetProgressDTO>();
            if (assetProgressDTO) {
                updateAssetProgress(*assetProgressDTO);
            }

            break;
        }
        case DTOType::ALARM_LIST: {
            auto dto = idto.dynamicCast<AlarmListDTO>();
            if (dto) {
                _robotAlarmsWidget.setAlarmList(*dto);
            }
            break;
        }
        case DTOType::ALARM_STATUS: {
            auto dto = idto.dynamicCast<AlarmStatusDTO>();
            if (dto) {
                _robotAlarmsWidget.update(*dto);
            }
            break;
        }
        case DTOType::HLACTION_LIST: {
            auto dto = idto.dynamicCast<HLActionListDTO>();
            if (dto) {
                _highLevelActionsWidget.receiveHLActionList(*dto);
            }
            break;
        }
        case DTOType::CURRENT_INSP_ITEM: {
            auto dto = idto.dynamicCast<CurrentInspItemDTO>();
            if (dto) {
                _telemetryTreeWidget.update(dto);
            }

            break;
        }
        case DTOType::REACHED_INSP_ITEM: {
            auto dto = idto.dynamicCast<ReachedInspItemDTO>();
            if (dto) {
                _telemetryTreeWidget.update(dto);
            }

            break;
        }
        case DTOType::INSPECTION_PLAN: {
            auto dto = idto.dynamicCast<InspectionPlanDTO>();
            if (dto) {
                _guiDTO.getInspectionPlan() = *dto;

                _inspectionPlanWidget.setInspectionPlan(dto);
                _waypointEditorWidget.updateInspectionTaskList(dto->getInspectionTaskList());
            }
            break;
        }
        case DTOType::SITE_LIST: {
            auto dto = idto.dynamicCast<SiteListDTO>();
            if (dto) {
                showInspectionPlanFromSiteDialog(*dto);
            }
            break;
        }
        case DTOType::DOWNLOADED_FILE_LIST: {
            auto dto = idto.dynamicCast<DownloadedFileListDTO>();
            if (dto) {
                selectFileAssetToDownload(*dto);
            }

            break;
        }
        case DTOType::ROBOT_POSITION_VELOCITY_NED:
        case DTOType::ROBOT_ATTITUDE_ANGVEL: {
            auto dto = idto.dynamicCast<RobotDataBaseDTO>();
            if (dto) {
                _telemetryTreeWidget.update(dto);
            }
            break;
        }
        case DTOType::CLOUD_DIMENSIONS: {
            auto dto = idto.dynamicCast<CloudDimensionsDTO>();
            if (dto) {
                _cloudEditorDialog.updateDimensions(dto);
            }
            break;
        }
        case DTOType::VISUALIZATION_CONFIG: {
            auto dto = idto.dynamicCast<VisualizationConfigDTO>();
            if (dto) {
                _guiDTO.getVisualizationConfig() = *dto;
            }
            break;
        }
        default: {
            break;
        }
    }
}

void MainWindowUi::closeEvent(QCloseEvent* event)
{
    if (!checkIfCloseMission()) {
        event->ignore();
        return;
    }

    writeGeometry();

    event->accept();

    Q_EMIT sendDTOToCore(ActionType::CLOSE_MISSION);
}

void MainWindowUi::closeMission()
{
    if (!checkIfCloseMission()) {
        return;
    }

    Q_EMIT sendDTOToCore(ActionType::CLOSE_MISSION);
}

void MainWindowUi::connectToRobot()
{
    ConnectionConfirmationDialog dialog(this);
    dialog.setCommsOptions(_guiDTO.getCommsMavsdkOptions());
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    auto dto = createSharedDTO<CommsLinkMavsdkDTO>();
    *dto     = _guiDTO.getCommsMavsdkOptions();
    Q_EMIT sendDTOToCore(ActionType::COMMS_CONNECT, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::disconnectFromRobot()
{
    const auto response = QMessageBox::question(
            this,
            tr("Disconnection confirmation"),
            tr("Are you sure that you want to disconnect from the robot system?"));

    if (response == QMessageBox::Yes) {
        Q_EMIT sendDTOToCore(ActionType::COMMS_DISCONNECT);
    }
}

void MainWindowUi::updateEditionMode(bool checked)
{
    _ui->editModeAct->setChecked(!checked);

    auto dto       = createSharedDTO<EditionModeDTO>();
    dto->getMode() = checked;
    Q_EMIT sendDTOToCore(ActionType::CHANGE_EDITION_MODE, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::updateViewMode(bool checked)
{
    auto mode = ViewModeType::ThreeD;

    auto actionButton = dynamic_cast<QAction*>(QObject::sender());
    if (actionButton) {
        actionButton->setChecked(!checked);
    }

    if (checked) {
        if (actionButton == _ui->mapTwoDViewAct) {
            mode = ViewModeType::TwoD;
        } else if (actionButton == _ui->mapThreeDViewAct) {
            mode = ViewModeType::ThreeD;
        } else {
            const QString msg("Unknown action button");
            QLOG_ERROR() << "MainWindowUi::updateViewMode() -" << msg;
            throw std::runtime_error(msg.toStdString());
        }
    }

    auto dto       = createSharedDTO<ViewModeDTO>();
    dto->getMode() = mode;
    Q_EMIT sendDTOToCore(ActionType::CHANGE_VIEW_MODE, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::updateViewModeButtons()
{
    const auto viewMode = _guiDTO.getViewMode();

    _ui->mapTwoDViewAct->setChecked(viewMode == ViewModeType::TwoD);
    _ui->mapThreeDViewAct->setChecked(viewMode == ViewModeType::ThreeD);
}

void MainWindowUi::updateAppStatus(bool checked)
{
    auto status = AppStatusType::Panning;

    auto actionButton = dynamic_cast<QAction*>(QObject::sender());
    if (actionButton) {
        actionButton->setChecked(!checked);
    }

    if (checked) {
        if (actionButton == _ui->creatingRouteAct) {
            status = AppStatusType::CreatingRoute;
        } else if (actionButton == _ui->editingRouteAct) {
            status = AppStatusType::EditingRoute;
        } else if (actionButton == _ui->editingLoadedHomeAct) {
            status = AppStatusType::EditingLoadedHome;
        } else if (actionButton == _ui->measuringDistancesAct) {
            status = AppStatusType::Measuring;
        } else {
            const QString msg("Unknown action button");
            QLOG_ERROR() << "MainWindowUi::updateAppStatus() -" << msg;
            throw std::runtime_error(msg.toStdString());
        }
    }

    auto dto         = createSharedDTO<ApplicationStatusDTO>();
    dto->getStatus() = status;
    Q_EMIT sendDTOToCore(ActionType::CHANGE_APPLICATION_STATUS, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::updateRouteOptions(bool checked)
{
    RouteOptions routeOptions = NoShow;
    routeOptions              = _ui->showLocalRoutesAct->isChecked() ? routeOptions |= ShowLocal : routeOptions;
    routeOptions              = _ui->showLoadedRoutesAct->isChecked() ? routeOptions |= ShowLoaded : routeOptions;

    auto actionButton = dynamic_cast<QAction*>(QObject::sender());
    if (actionButton) {
        actionButton->setChecked(!checked);
    }

    auto dto       = createSharedDTO<RouteOptionsDTO>();
    dto->getFlag() = routeOptions;
    Q_EMIT sendDTOToCore(ActionType::CHANGE_ROUTE_OPTIONS, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::changeCurrentWaypoint(QSharedPointer<BasicWaypointDTO> dto)
{
    Q_EMIT sendDTOToCore(ActionType::CHANGE_SELECTED_WAYPOINT, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::editWaypointFeatures(QSharedPointer<WaypointDTO> dto)
{
    Q_EMIT sendDTOToCore(ActionType::EDIT_WAYPOINT, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::editPointCloud(QSharedPointer<CloudEditionParametersDTO> dto)
{
    Q_EMIT sendDTOToCore(ActionType::EDIT_POINTCLOUD, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::restorePointCloud()
{
    Q_EMIT sendDTOToCore(ActionType::RESTORE_POINTCLOUD);
}

void MainWindowUi::hideCutPlane()
{
    if (_ui->cutCloudAct->isChecked()) {
        _ui->cutCloudAct->setChecked(false);
    }

    Q_EMIT sendDTOToCore(ActionType::HIDE_CUT_PLANE);
}

void MainWindowUi::hidePointCloud()
{
    Q_EMIT sendDTOToCore(ActionType::HIDE_POINTCLOUD);
}

// PILOTING actions calls
void MainWindowUi::loadMissionResultsFromFolder()
{
    const auto missionResultPath = QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/missions/").arg(PROJECT_VERSION_MAJOR).arg(PROJECT_VERSION_MINOR));

    auto dto         = createSharedDTO<BasicStringDTO>();
    dto->getString() = missionResultPath;
    Q_EMIT sendDTOToCore(ActionType::LOAD_MISSION_RESULT_LIST_FROM_FOLDER, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::loadXMLDataFromFolder()
{
    const auto default_path = QString::fromStdString(PROJECT_SOURCE_PATH + std::string("data/xml"));
    const auto ros_path     = QString::fromStdString("/opt/ros/melodic/share/piloting_grcs/xml");

    QString xml_path = QString();
    if (QDir(default_path).exists()) {
        xml_path = default_path;
    } else if (QDir(ros_path).exists()) {
        xml_path = ros_path;
    } else {
        auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
        msgConsoleDTO->getMsgType() = MsgType::WARNING;
        msgConsoleDTO->getMessage() = QString("Unable to load MAVSDK XML from folder");
        showMessage(msgConsoleDTO);
        return;
    }

    auto dto         = createSharedDTO<BasicStringDTO>();
    dto->getString() = xml_path;
    Q_EMIT sendDTOToCore(ActionType::LOAD_XML_DATA_FROM_FOLDER, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::sendCommand(QSharedPointer<CommandLongDTO> dto)
{
    Q_EMIT sendDTOToCore(ActionType::SEND_COMMAND, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::uploadWaypointList()
{
    if (showConfirmationDialog(
                tr("Send Waypoints List to vehicle"),
                tr("Are you sure that you want to upload this waypoint list to robotic system?"))) {
        Q_EMIT sendDTOToCore(ActionType::UPLOAD_WAYPOINTS_LIST);
    }
}

void MainWindowUi::downloadWaypointList()
{
    if (showConfirmationDialog(
                tr("Download Waypoints List"), tr("Are you sure that you want to download the waypoints list?"))) {
        Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_WAYPOINTS_LIST);
    }
}

void MainWindowUi::downloadCheckList()
{
    Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_CHECKLIST);
}

void MainWindowUi::downloadAlarmsList()
{
    Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_ALARMS_LIST);
}

void MainWindowUi::downloadHighLevelActions()
{
    Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_HLACTION_LIST);
}
void MainWindowUi::sendHome()
{
    if (!_guiDTO.getLoadedHome().getIsEnabled()) {
        displayMsgBox(MsgType::WARNING, tr("Send Home"), tr("Home is not loaded, please load it before sending it."));
        return;
    }

    auto dto = createSharedDTO<HomeDTO>();
    *dto     = _guiDTO.getLoadedHome();
    Q_EMIT sendDTOToCore(ActionType::SEND_HOME, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::setCurrentWaypointItem(quint16 task_idx)
{
    if (showConfirmationDialog(
                tr("Set Current Waypoint Item"),
                tr("Are you sure that you want to set the following waypoint item as current?: ")
                        + QString::number(task_idx))) {
        auto dto      = createSharedDTO<BasicIntDTO>();
        dto->getInt() = static_cast<int>(task_idx);
        Q_EMIT sendDTOToCore(ActionType::SET_CURRENT_WAYPOINT_ITEM, QSharedPointer<IDTO>(dto));
    }
}

void MainWindowUi::updateViewInspectionTask(QString task_to_view_uuid)
{
    auto dto         = createSharedDTO<BasicStringDTO>();
    dto->getString() = task_to_view_uuid;
    Q_EMIT sendDTOToCore(ActionType::UPDATE_VIEW_INSPECTION_TASK, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::updateMissionTask(QSharedPointer<MissionTaskDTO> missionTaskDTO)
{
    Q_EMIT sendDTOToCore(ActionType::UPDATE_MISSION_TASK, QSharedPointer<IDTO>(missionTaskDTO));
}

void MainWindowUi::waypointsEditorLocationChanged(Qt::DockWidgetArea area)
{
    if (area == Qt::LeftDockWidgetArea || area == Qt::RightDockWidgetArea) {
        _waypointEditorWidget.setVerticalLayout();
    } else if (area == Qt::TopDockWidgetArea || area == Qt::BottomDockWidgetArea) {
        _waypointEditorWidget.setHorizontalLayout();
    }
}

void MainWindowUi::showMessage(QSharedPointer<MsgConsoleDTO> dto)
{
    if (!dto) {
        return;
    }

    _consoleDisplayWidget.addMessage(dto->getMsgType(), dto->getMessage());

    auto popup = dto.dynamicCast<MsgConsolePopupDTO>();
    if (popup) {
        displayMsgBox(popup->getMsgType(), popup->getTitle(), popup->getMessage());
    }
}

void MainWindowUi::displayMsgBox(const MsgType& msgType, const QString& title, const QString& msg)
{
    auto msgBox = new QMessageBox(this);
    msgBox->setModal(true);
    msgBox->setAttribute(Qt::WA_DeleteOnClose);
    msgBox->setWindowTitle(title);
    msgBox->setText(msg);
    switch (msgType) {
        case MsgType::INFO: {
            msgBox->setIcon(QMessageBox::Information);
            break;
        }
        case MsgType::WARNING: {
            msgBox->setIcon(QMessageBox::Warning);
            break;
        }
        case MsgType::ERROR: {
            msgBox->setIcon(QMessageBox::Critical);
            break;
        }
        default: {
            QLOG_WARN() << __PRETTY_FUNCTION__ << " - "
                        << QString("Unknown message type: %1").arg(static_cast<uint8_t>(msgType));
            break;
        }
    }
    msgBox->show();
}

void MainWindowUi::connectActions()
{
    // clang-format off
   connect(_ui->quitAct,                &QAction::triggered,
           this,                        &MainWindowUi::close);

   connect(_ui->loadInspectionPlanAct,  &QAction::triggered,
           this,                        &MainWindowUi::loadInspectionPlan);

   connect(_ui->saveCurrWptsListAct,    &QAction::triggered,
           this,                        &MainWindowUi::saveCurrentWaypointList);

   connect(_ui->saveAsCurrWptsListAct,  &QAction::triggered,
           this,                        &MainWindowUi::saveAsCurrentWaypointList);

   connect(_ui->loadAssocWptsListAct,   &QAction::triggered,
           this,                        &MainWindowUi::loadAssociatedWaypointList);

   connect(_ui->commsOptionsAct,        &QAction::triggered,
           this,                        &MainWindowUi::showCommsOptions);

   connect(_ui->editModeAct,            &QAction::triggered,
           this,                        &MainWindowUi::updateEditionMode);
   connect(_ui->mapTwoDViewAct,         &QAction::triggered,
           this,                        &MainWindowUi::updateViewMode);
   connect(_ui->mapThreeDViewAct,       &QAction::triggered,
           this,                        &MainWindowUi::updateViewMode);

   connect(_ui->creatingRouteAct,       &QAction::triggered,
           this,                        &MainWindowUi::updateAppStatus);
   connect(_ui->editingRouteAct,        &QAction::triggered,
           this,                        &MainWindowUi::updateAppStatus);
   connect(_ui->editingLoadedHomeAct,   &QAction::triggered,
           this,                        &MainWindowUi::updateAppStatus);
   connect(_ui->measuringDistancesAct,  &QAction::triggered,
           this,                        &MainWindowUi::updateAppStatus);

   connect(_ui->configDDHLAct,          &QAction::triggered,
           this,                        &MainWindowUi::showDDHLOptions);
   connect(_ui->downloadDDHLAct,        &QAction::triggered,
           this,                        &MainWindowUi::downloadFromDDHL);
   connect(_ui->uploadDDHLAct,          &QAction::triggered,
           this,                        &MainWindowUi::uploadToDDHL);

   connect(_ui->connectAct,             &QAction::triggered,
           this,                        &MainWindowUi::connectToRobot);
   connect(_ui->disconnectAct,          &QAction::triggered,
           this,                        &MainWindowUi::disconnectFromRobot);

   connect(_ui->showLocalRoutesAct,     &QAction::triggered,
           this,                        &MainWindowUi::updateRouteOptions);
   connect(_ui->showLoadedRoutesAct,    &QAction::triggered,
           this,                        &MainWindowUi::updateRouteOptions);

   connect(_ui->checklistAct,           &QAction::triggered,
           this,                        &MainWindowUi::showCheckList);

   connect(_ui->configVisualizationAct, &QAction::triggered,
           this,                        &MainWindowUi::showVisualizacionConfig);

   connect(_ui->enableAutoHeadingAct,   &QAction::triggered,
           this,                        &MainWindowUi::manageAutoHeading);
    // clang-format on

    connectActionsCloudEditor();
}

void MainWindowUi::connectActionsCloudEditor()
{
    // clang-format off
   connect(_ui->cutCloudAct,     &QAction::triggered,
           &_cloudEditorDialog,  &CloudEditorDialog::setVisible);

   connect(&_cloudEditorDialog,  &CloudEditorDialog::editPointCloud,
           this,                 &MainWindowUi::editPointCloud);

   connect(&_cloudEditorDialog,  &CloudEditorDialog::restorePointCloud,
           this,                 &MainWindowUi::restorePointCloud);

   connect(&_cloudEditorDialog,  &CloudEditorDialog::hideCutPlane,
           this,                 &MainWindowUi::hideCutPlane);
   
   connect(&_cloudEditorDialog,  &CloudEditorDialog::hidePointCloud,
           this,                 &MainWindowUi::hidePointCloud);
    // clang-format on
}

void MainWindowUi::loadGeometry()
{
    restoreViewState();

    QSettings settings;
    settings.beginGroup("MainWindowUi");

    restoreGeometry(settings.value("geometry").toByteArray());
    /// sario porque no se recolocaba correctamente la ventana
    const auto pos = settings.value("pos").toPoint();
    if (!pos.isNull()) {
        move(pos);
    }

    settings.endGroup();
}

void MainWindowUi::writeGeometry()
{
    saveViewState();

    QSettings settings;
    settings.beginGroup("MainWindowUi");

    settings.setValue("geometry", saveGeometry());
    settings.setValue("pos", pos());

    settings.endGroup();
}

void MainWindowUi::restoreViewState()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi");

    const QString key = _waypointsEditorDW.isVisible() ? "editModeState" : "flightModeState";
    restoreState(settings.value(key).toByteArray());

    settings.endGroup();
}

void MainWindowUi::saveViewState()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi");

    const QString key = _waypointsEditorDW.isVisible() ? "editModeState" : "flightModeState";
    settings.setValue(key, saveState());

    settings.endGroup();
}

void MainWindowUi::setAssetProgressDialog()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _assetDownloadprogressDialog.reset();

    _assetDownloadprogressDialog.setModal(true);
    _assetDownloadprogressDialog.overrideWindowFlags(
            static_cast<Qt::WindowFlags>(Qt::Window | Qt::Dialog | Qt::FramelessWindowHint));
    // clang-format off
    connect(&_assetDownloadprogressDialog, &QProgressDialog::canceled,
            this,                          &MainWindowUi::cancelAssetDownloadProgress);
    // clang-format on
}

void MainWindowUi::setWaypointsEditorDock(const Qt::DockWidgetArea area)
{
    _waypointsEditorDW.setObjectName("waypointsEditorDW");
    _waypointsEditorDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _waypointsEditorDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    _waypointEditorWidget.setParent(&_waypointsEditorDW);
    _waypointsEditorDW.setWidget(&_waypointEditorWidget);
    addDockWidget(area, &_waypointsEditorDW);

    // clang-format off
    connect(&_waypointEditorWidget,  &WaypointsEditorWidget::changeCurrentWaypoint,
            this,                    &MainWindowUi::changeCurrentWaypoint);
    connect(&_waypointEditorWidget,  &WaypointsEditorWidget::editWaypointFeatures,
            this,                    &MainWindowUi::editWaypointFeatures);
 
    connect(&_waypointsEditorDW,     &QDockWidget::dockLocationChanged,
            this,                    &MainWindowUi::waypointsEditorLocationChanged);
    // clang-format on

    _waypointsEditorDW.setVisible(_ui->editModeAct->isChecked());
}

void MainWindowUi::setRobotAlarmsDock(const Qt::DockWidgetArea area)
{
    _robotAlarmsDW.setObjectName("robotAlarmsDW");
    _robotAlarmsDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _robotAlarmsDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    _robotAlarmsWidget.setParent(&_robotAlarmsDW);
    _robotAlarmsDW.setWidget(&_robotAlarmsWidget);
    addDockWidget(area, &_robotAlarmsDW);
}

void MainWindowUi::setRobotControlDock(const Qt::DockWidgetArea area)
{
    _robotControlDW.setObjectName("robotControlDW");
    _robotControlDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _robotControlDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    _robotsControlWidget.setParent(&_robotControlDW);
    _robotControlDW.setWidget(&_robotsControlWidget);
    addDockWidget(area, &_robotControlDW);

    // clang-format off
   connect(&_robotsControlWidget,   &RobotControlWidget::uploadWaypointList,
           this,                    &MainWindowUi::uploadWaypointList);
   connect(&_robotsControlWidget,   &RobotControlWidget::downloadWaypointList,
           this,                    &MainWindowUi::downloadWaypointList);
   connect(&_robotsControlWidget,   &RobotControlWidget::setCurrentWaypointItem,
           this,                    &MainWindowUi::setCurrentWaypointItem);
   connect(&_robotsControlWidget,   &RobotControlWidget::downloadCheckList,
           this,                    &MainWindowUi::downloadCheckList);
   connect(&_robotsControlWidget,   &RobotControlWidget::downloadAlarmsList,
           this,                    &MainWindowUi::downloadAlarmsList);
   connect(&_robotsControlWidget,   &RobotControlWidget::downloadHighLevelActions,
           this,                    &MainWindowUi::downloadHighLevelActions);
   connect(&_robotsControlWidget,   &RobotControlWidget::sendHome,
           this,                    &MainWindowUi::sendHome);
    // clang-format on
}

void MainWindowUi::setConsoleDock(const Qt::DockWidgetArea area)
{
    _consoleDW.setObjectName("consoleDW");
    _consoleDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _consoleDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    _consoleDisplayWidget.setParent(&_consoleDW);
    _consoleDW.setWidget(&_consoleDisplayWidget);
    addDockWidget(area, &_consoleDW);
}

void MainWindowUi::setInspectionPlanDock(const Qt::DockWidgetArea area)
{
    _inspectionPlanDW.setObjectName("inspectionPlanDW");
    _inspectionPlanDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _inspectionPlanDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    _inspectionPlanWidget.setParent(&_inspectionPlanDW);
    _inspectionPlanDW.setWidget(&_inspectionPlanWidget);
    addDockWidget(area, &_inspectionPlanDW);

    // clang-format off
    connect(&_inspectionPlanWidget,  &InspectionPlanWidget::updateViewInspectionTask,
            this,                    &MainWindowUi::updateViewInspectionTask);
    // clang-format on
}

void MainWindowUi::setMissionDock(const Qt::DockWidgetArea area)
{
    _missionTasksDW.setObjectName("missionTasksDW");
    _missionTasksDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _missionTasksDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    _missionWidget.setParent(&_missionTasksDW);
    _missionTasksDW.setWidget(&_missionWidget);
    addDockWidget(area, &_missionTasksDW);

    // clang-format off
    connect(&_missionWidget,  &MissionTasksWidget::updateMissionTask,
            this,              &MainWindowUi::updateMissionTask);
    connect(&_missionWidget,  &MissionTasksWidget::updateMissionResult,
            this,             &MainWindowUi::updateMissionResult);
    // clang-format on
}

void MainWindowUi::setTelemetryTreeDock(const Qt::DockWidgetArea area)
{
    _telemetryTreeDW.setObjectName("telemetryTreeDW");
    _telemetryTreeDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _telemetryTreeDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    _telemetryTreeWidget.setParent(&_telemetryTreeDW);
    _telemetryTreeDW.setWidget(&_telemetryTreeWidget);
    addDockWidget(area, &_telemetryTreeDW);

    _telemetryTreeWidget.setUpdateInterval(1000); /// 1 Hz
}

void MainWindowUi::setHighLevelActionsDock(const Qt::DockWidgetArea area)
{
    _highLevelActionsDW.setObjectName("highLevelActionsDW");
    _highLevelActionsDW.setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    _highLevelActionsDW.setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    _highLevelActionsWidget.setParent(&_highLevelActionsDW);
    _highLevelActionsDW.setWidget(&_highLevelActionsWidget);
    addDockWidget(area, &_highLevelActionsDW);

    // clang-format off
    connect(&_highLevelActionsWidget, &HighLevelActionsWidget::sendCommand, 
            this,                     &MainWindowUi::sendCommand);
    // clang-format on
}

bool MainWindowUi::checkIfCloseMission()
{
    if (_guiDTO.getCommsIsConnected()) {
        displayMsgBox(
                MsgType::WARNING,
                tr("Close Mission"),
                tr("The communications must be disconnected before closing the mission."));
        return false;
    }

    const auto response = QMessageBox::question(
            this,
            tr("Close Application"),
            tr("Are you sure you want to close the gRCS?"),
            QMessageBox::Yes,
            QMessageBox::No);

    return response == QMessageBox::Yes;
}

void MainWindowUi::updateMissionResult(QSharedPointer<CurrentMissionResultDTO> dto)
{
    Q_EMIT sendDTOToCore(ActionType::UPDATE_CURRENT_MISSION_RESULT, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::updateConnectionStatus()
{
    const auto isConnected = _guiDTO.getCommsIsConnected();

    _robotsControlWidget.updateCommsStatus(isConnected);
    _robotAlarmsWidget.updateCommsStatus(isConnected);
    _telemetryTreeWidget.updateCommsStatus(isConnected);
    _missionWidget.updateCommsStatus(isConnected);

    _ui->disconnectAct->setEnabled(isConnected);
    _ui->connectAct->setDisabled(isConnected);
    _ui->commsOptionsAct->setDisabled(isConnected);

    if (isConnected) {
        Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_WAYPOINTS_LIST);
    } else {
        Q_EMIT sendDTOToCore(ActionType::REMOVE_LOADED_ROUTE);

        _highLevelActionsWidget.resetHLActionList();
        _robotAlarmsWidget.resetAlarmList();
        _guiDTO.getCheckList().getCheckItemsList().clear();
    }
}

void MainWindowUi::updateAppStatusButtons()
{
    const auto appStatus = _guiDTO.getAppStatus();

    _ui->creatingRouteAct->setChecked(appStatus == AppStatusType::CreatingRoute);
    _ui->editingRouteAct->setChecked(appStatus == AppStatusType::EditingRoute);
    _ui->editingLoadedHomeAct->setChecked(appStatus == AppStatusType::EditingLoadedHome);
    _ui->measuringDistancesAct->setChecked(appStatus == AppStatusType::Measuring);
}

void MainWindowUi::updateMissionName()
{
    QString windowTitle("PILOTING General Robot Control Station - ");
    if (_guiDTO.getMissionStatus() == MissionStatusType::NoMission) {
        windowTitle.append("NoMission");
    } else {
        windowTitle.append(_guiDTO.getMissionName());
        if (_guiDTO.getMissionStatus() == MissionStatusType::WithChanges) {
            windowTitle.append("*");
        }
    }
    setWindowTitle(windowTitle);
}

void MainWindowUi::updateMissionButtons()
{
    const auto hasMission  = _guiDTO.getMissionStatus() != MissionStatusType::NoMission;
    const auto hasRoutes   = _guiDTO.getHasRoutes();
    const auto isConnected = _guiDTO.getCommsIsConnected();
    const auto isEditMode  = _ui->editModeAct->isChecked();
    // const auto hasInspPlan = !_guiDTO.getInspectionPlan().getInspectionTaskList().isEmpty();
    const auto hasInspPlan = !_guiDTO.getInspectionPlan().getUUID().isEmpty();

    _ui->saveCurrWptsListAct->setEnabled(hasInspPlan && hasRoutes);
    _ui->saveAsCurrWptsListAct->setEnabled(hasInspPlan && hasRoutes);
    _ui->loadAssocWptsListAct->setEnabled(hasInspPlan);

    _ui->enableAutoHeadingAct->setEnabled(hasInspPlan);

    _ui->editionBar->setEnabled(hasMission && isEditMode);
    _ui->editingRouteAct->setEnabled(hasRoutes);

    _ui->connectionBar->setEnabled(hasMission);
    _ui->checkBar->setEnabled(isConnected && hasMission);

    _ui->visualBar->setEnabled(hasMission);
    _ui->showLocalRoutesAct->setEnabled(hasRoutes);
    _ui->showLoadedRoutesAct->setEnabled(isConnected);

    _waypointEditorWidget.updateStatus(isEditMode && hasMission && hasRoutes);

    /// \note. Would be neccesary add more conditions
    _ui->cutCloudAct->setEnabled(hasInspPlan);
    _cloudEditorDialog.updateStatus(hasInspPlan, hasInspPlan);

    _ui->configVisualizationAct->setEnabled(hasInspPlan);
}

void MainWindowUi::updateEditionModeWidgets()
{
    /// \note Save previous view state
    saveViewState();

    if (_ui->editModeAct->isChecked()) {
        _robotControlDW.setVisible(false);
        _robotAlarmsDW.setVisible(false);
        _inspectionPlanDW.setVisible(false);
        _telemetryTreeDW.setVisible(false);
        _highLevelActionsDW.setVisible(false);
        _missionTasksDW.setVisible(false);
        _consoleDW.setVisible(false);

        _waypointsEditorDW.setVisible(true);
    } else {
        _waypointsEditorDW.setVisible(false);

        _robotControlDW.setVisible(true);
        _robotAlarmsDW.setVisible(true);
        _inspectionPlanDW.setVisible(true);
        _telemetryTreeDW.setVisible(true);
        _highLevelActionsDW.setVisible(true);
        _missionTasksDW.setVisible(true);
        _consoleDW.setVisible(true);
    }

    /// \note Restore current view state
    restoreViewState();
}

void MainWindowUi::updateRouteOptionsButtons()
{
    const auto routeOptions = _guiDTO.getRouteOptions();

    _ui->showLocalRoutesAct->setChecked(routeOptions & ShowLocal);
    _ui->showLoadedRoutesAct->setChecked(routeOptions & ShowLoaded);
}

void MainWindowUi::showCommsOptions()
{
    CommsOptionsMavsdkDialog dialog(this);
    dialog.setHeartbeatInterval(_guiDTO.getCommsMavsdkOptions().getHeartBeatInterval());
    dialog.setLocalSystemId(_guiDTO.getCommsMavsdkOptions().getLocalSystemId());
    dialog.setLocalComponentId(_guiDTO.getCommsMavsdkOptions().getLocalComponentId());
    dialog.setLocalIP(_guiDTO.getCommsMavsdkOptions().getLocalIP());
    dialog.setLocalPort(_guiDTO.getCommsMavsdkOptions().getLocalPort());
    dialog.setTargetIP(_guiDTO.getCommsMavsdkOptions().getTargetIP());
    dialog.setTargetPort(_guiDTO.getCommsMavsdkOptions().getTargetPort());
    dialog.setTargetSystemId(_guiDTO.getCommsMavsdkOptions().getTargetSystemId());

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    _guiDTO.getCommsMavsdkOptions().getHeartBeatInterval() = dialog.getHeartbeatInterval();
    _guiDTO.getCommsMavsdkOptions().getLocalSystemId()     = dialog.getLocalSystemId();
    _guiDTO.getCommsMavsdkOptions().getLocalComponentId()  = dialog.getLocalComponentId();
    _guiDTO.getCommsMavsdkOptions().getLocalIP()           = dialog.getLocalIP();
    _guiDTO.getCommsMavsdkOptions().getLocalPort()         = dialog.getLocalPort();
    _guiDTO.getCommsMavsdkOptions().getTargetIP()          = dialog.getTargetIP();
    _guiDTO.getCommsMavsdkOptions().getTargetPort()        = dialog.getTargetPort();
    _guiDTO.getCommsMavsdkOptions().getTargetSystemId()    = dialog.getTargetSystemId();

    writeMavsdkCommsOptions();
}

void MainWindowUi::showCheckList()
{
    const auto checkList = _guiDTO.getCheckList();

    CheckListDialog dialog(this);
    dialog.setCheckList(checkList);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    auto dto = createSharedDTO<CheckListDTO>();
    *dto     = dialog.getCheckList();

    CheckListInfoDialog infoDialog(this);
    infoDialog.setCheckList(*dto);
    if (infoDialog.exec() != QDialog::Accepted) {
        return;
    }

    Q_EMIT sendDTOToCore(ActionType::UPDATE_CHECKLIST, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::showVisualizacionConfig()
{
    VisualizationConfigDialog dialog(this);
    dialog.setShowFloorGrid(_guiDTO.getVisualizationConfig().getShowFloorGrid());
    dialog.setShowItemsText(_guiDTO.getVisualizationConfig().getShowItemsText());
    dialog.setShowGlobalAxis(_guiDTO.getVisualizationConfig().getShowGlobalAxis());
    dialog.setShowHomePosition(_guiDTO.getVisualizationConfig().getShowHomePosition());
    dialog.setWaypointSizeScale(_guiDTO.getVisualizationConfig().getWaypointSizeScale());
    dialog.setCloudPtsSize(_guiDTO.getVisualizationConfig().getPointCloudPtsSize());
    dialog.setInspectionPtScale(_guiDTO.getVisualizationConfig().getInspPtSizeScale());
    dialog.setInspLocationTransp(_guiDTO.getVisualizationConfig().getInspLocationTransp());
    dialog.setPointCloudColorStyle(_guiDTO.getVisualizationConfig().getPointCloudColorStyle());
    dialog.setPointCloudColor(_guiDTO.getVisualizationConfig().getPointCloudColor());

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    _guiDTO.getVisualizationConfig().getShowFloorGrid()        = dialog.getShowFloorGrid();
    _guiDTO.getVisualizationConfig().getShowItemsText()        = dialog.getShowItemsText();
    _guiDTO.getVisualizationConfig().getShowGlobalAxis()       = dialog.getShowGlobalAxis();
    _guiDTO.getVisualizationConfig().getShowHomePosition()     = dialog.getShowHomePosition();
    _guiDTO.getVisualizationConfig().getWaypointSizeScale()    = dialog.getWaypointSizeScale();
    _guiDTO.getVisualizationConfig().getPointCloudPtsSize()    = dialog.getCloudPtsSize();
    _guiDTO.getVisualizationConfig().getInspPtSizeScale()      = dialog.getInspPtSizeScale();
    _guiDTO.getVisualizationConfig().getInspLocationTransp()   = dialog.getInspLocationTransp();
    _guiDTO.getVisualizationConfig().getPointCloudColorStyle() = dialog.getPointCloudColorStyle();
    _guiDTO.getVisualizationConfig().getPointCloudColor()      = dialog.getPointCloudColor();

    writeVisualizationOptions();

    auto dto = createSharedDTO<VisualizationConfigDTO>();
    *dto     = _guiDTO.getVisualizationConfig();

    Q_EMIT sendDTOToCore(ActionType::UPDATE_VISUALIZATION_CONFIG, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::loadMavsdkCommsOptions()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/MavsdkOptions");

    _guiDTO.getCommsMavsdkOptions().getHeartBeatInterval() = settings.value("heart_beat_interval", 1000).toUInt();
    _guiDTO.getCommsMavsdkOptions().getLocalSystemId()     = settings.value("local_system_id", 31).toUInt();
    _guiDTO.getCommsMavsdkOptions().getLocalComponentId()  = settings.value("local_component_id", 26).toUInt();
    _guiDTO.getCommsMavsdkOptions().getLocalIP()           = settings.value("local_ip", "127.0.0.1").toString();
    _guiDTO.getCommsMavsdkOptions().getLocalPort()         = settings.value("local_port", 14540).toUInt();
    _guiDTO.getCommsMavsdkOptions().getTargetIP()          = settings.value("target_ip", "127.0.0.1").toString();
    _guiDTO.getCommsMavsdkOptions().getTargetPort()        = settings.value("target_port", 14541).toUInt();
    _guiDTO.getCommsMavsdkOptions().getTargetSystemId()    = settings.value("target_system_id", 30).toUInt();

    settings.endGroup();
}

void MainWindowUi::writeMavsdkCommsOptions()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/MavsdkOptions");

    settings.setValue("heart_beat_interval", _guiDTO.getCommsMavsdkOptions().getHeartBeatInterval());
    settings.setValue("local_system_id", _guiDTO.getCommsMavsdkOptions().getLocalSystemId());
    settings.setValue("local_component_id", _guiDTO.getCommsMavsdkOptions().getLocalComponentId());
    settings.setValue("local_ip", _guiDTO.getCommsMavsdkOptions().getLocalIP());
    settings.setValue("local_port", _guiDTO.getCommsMavsdkOptions().getLocalPort());
    settings.setValue("target_ip", _guiDTO.getCommsMavsdkOptions().getTargetIP());
    settings.setValue("target_port", _guiDTO.getCommsMavsdkOptions().getTargetPort());
    settings.setValue("target_system_id", _guiDTO.getCommsMavsdkOptions().getTargetSystemId());

    settings.endGroup();
}

void MainWindowUi::loadDDHLOptions()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/DDHLOptions");

    _guiDTO.getCommsDDHLOptions().getEndpointMode()
            = ToDDHLEndpointMode(settings.value("endpoint_mode", "url").toString());
    _guiDTO.getCommsDDHLOptions().getTargetUrl()
            = settings.value("target_url", "https://piloting-ddhl.inlecomsystems.com/apis").toString();
    _guiDTO.getCommsDDHLOptions().getTargetIP()   = settings.value("target_ip", "168.119.15.247").toString();
    _guiDTO.getCommsDDHLOptions().getTargetPort() = settings.value("target_port", 3040).toUInt();
    _guiDTO.getCommsDDHLOptions().getUsername()   = settings.value("user_name", "user").toString();

    settings.endGroup();

    auto dto = createSharedDTO<CommsLinkDDHLDTO>();
    *dto     = _guiDTO.getCommsDDHLOptions();
    Q_EMIT sendDTOToCore(ActionType::UPDATE_COMMS_LINK_DDHL, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::writeDDHLOptions()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/DDHLOptions");

    settings.setValue("endpoint_mode", ToString(_guiDTO.getCommsDDHLOptions().getEndpointMode()));
    settings.setValue("target_url", _guiDTO.getCommsDDHLOptions().getTargetUrl());
    settings.setValue("target_ip", _guiDTO.getCommsDDHLOptions().getTargetIP());
    settings.setValue("target_port", _guiDTO.getCommsDDHLOptions().getTargetPort());
    settings.setValue("user_name", _guiDTO.getCommsDDHLOptions().getUsername());

    settings.endGroup();
}

void MainWindowUi::loadVisualizationOptions()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/VisualizationOptions");

    _guiDTO.getVisualizationConfig().getShowFloorGrid()        = settings.value("show_grid", true).toBool();
    _guiDTO.getVisualizationConfig().getShowItemsText()        = settings.value("show_items_text", true).toBool();
    _guiDTO.getVisualizationConfig().getShowGlobalAxis()       = settings.value("show_global_axis", true).toBool();
    _guiDTO.getVisualizationConfig().getShowHomePosition()     = settings.value("show_home_position", true).toBool();
    _guiDTO.getVisualizationConfig().getWaypointSizeScale()    = settings.value("wpts_scale", 1.0).toFloat();
    _guiDTO.getVisualizationConfig().getInspPtSizeScale()      = settings.value("insp_pt_scale", 1.0).toFloat();
    _guiDTO.getVisualizationConfig().getInspLocationTransp()   = settings.value("insp_loc_transp", 0.2).toFloat();
    _guiDTO.getVisualizationConfig().getPointCloudPtsSize()    = settings.value("cloud_pts_size", 0.05).toFloat();
    _guiDTO.getVisualizationConfig().getPointCloudColorStyle() = settings.value("cloud_pts_color_style", "RGB8");
    _guiDTO.getVisualizationConfig().getPointCloudColor()      = settings.value("cloud_pts_color").value<QColor>();

    settings.endGroup();

    auto dto = createSharedDTO<VisualizationConfigDTO>();
    *dto     = _guiDTO.getVisualizationConfig();

    Q_EMIT sendDTOToCore(ActionType::UPDATE_VISUALIZATION_CONFIG, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::writeVisualizationOptions()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/VisualizationOptions");

    settings.setValue("show_grid", _guiDTO.getVisualizationConfig().getShowFloorGrid());
    settings.setValue("show_items_text", _guiDTO.getVisualizationConfig().getShowItemsText());
    settings.setValue("show_global_axis", _guiDTO.getVisualizationConfig().getShowGlobalAxis());
    settings.setValue("show_home_position", _guiDTO.getVisualizationConfig().getShowHomePosition());
    settings.setValue("wpts_scale", _guiDTO.getVisualizationConfig().getWaypointSizeScale());
    settings.setValue("insp_pt_scale", _guiDTO.getVisualizationConfig().getInspPtSizeScale());
    settings.setValue("insp_loc_transp", _guiDTO.getVisualizationConfig().getInspLocationTransp());
    settings.setValue("cloud_pts_size", _guiDTO.getVisualizationConfig().getPointCloudPtsSize());
    settings.setValue("cloud_pts_color_style", _guiDTO.getVisualizationConfig().getPointCloudColorStyle());
    settings.setValue("cloud_pts_color", _guiDTO.getVisualizationConfig().getPointCloudColor());

    settings.endGroup();
}

void MainWindowUi::loadTargetUUID()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/TargetUUID");

    _guiDTO.getTargetInspectionPlanUUID() = settings.value("target_inspection_plan_uuid", "").toString();

    settings.endGroup();
}

void MainWindowUi::writeTargetUUID()
{
    QSettings settings;
    settings.beginGroup("MainWindowUi/TargetUUID");

    settings.setValue("target_inspection_plan_uuid", _guiDTO.getTargetInspectionPlanUUID());

    settings.endGroup();
}

void MainWindowUi::showDDHLOptions()
{
    CommsOptionsDDHLDialog dialog(this);
    dialog.setEndpointMode(_guiDTO.getCommsDDHLOptions().getEndpointMode());
    dialog.setTargetUrl(_guiDTO.getCommsDDHLOptions().getTargetUrl());
    dialog.setTargetIP(_guiDTO.getCommsDDHLOptions().getTargetIP());
    dialog.setTargetPort(_guiDTO.getCommsDDHLOptions().getTargetPort());
    dialog.setUsername(_guiDTO.getCommsDDHLOptions().getUsername());
    dialog.setPassword(_guiDTO.getCommsDDHLOptions().getPassword());

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    _guiDTO.getCommsDDHLOptions().getEndpointMode() = dialog.getEndpointMode();
    _guiDTO.getCommsDDHLOptions().getTargetUrl()    = dialog.getTargetUrl();
    _guiDTO.getCommsDDHLOptions().getTargetIP()     = dialog.getTargetIP();
    _guiDTO.getCommsDDHLOptions().getTargetPort()   = dialog.getTargetPort();
    _guiDTO.getCommsDDHLOptions().getUsername()     = dialog.getUsername();
    _guiDTO.getCommsDDHLOptions().getPassword()     = dialog.getPassword();

    auto dto = createSharedDTO<CommsLinkDDHLDTO>();
    *dto     = _guiDTO.getCommsDDHLOptions();
    Q_EMIT sendDTOToCore(ActionType::UPDATE_COMMS_LINK_DDHL, QSharedPointer<IDTO>(dto));
    writeDDHLOptions();
}

void MainWindowUi::downloadFromDDHL()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    DownloadDataDDHLDialog dialog(this);
    dialog.setDDHLCommsOptions(_guiDTO.getCommsDDHLOptions());
    dialog.setTargetInspectionPlanUUID(_guiDTO.getTargetInspectionPlanUUID());

    // clang-format off
    connect(&dialog, &DownloadDataDDHLDialog::downloadInspectionPlanFromUUID, 
            this, [&]() 
            {
                _guiDTO.getDownloadAssetIsCanceled()  = false;
                _guiDTO.getTargetInspectionPlanUUID() = dialog.getInspectionPlanUUID();

                auto dto         = createSharedDTO<BasicStringDTO>();
                dto->getString() = _guiDTO.getTargetInspectionPlanUUID();
                Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_INSP_PLAN_FROM_DDHL, QSharedPointer<IDTO>(dto));

                writeTargetUUID();

            });

    connect(&dialog, &DownloadDataDDHLDialog::downloadInspectionPlansFromSites, 
            this, [&](const SiteType& infrastructureType) 
            {
                auto dto         = createSharedDTO<SiteTypeDTO>();
                dto->getSiteType() = infrastructureType;
                Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_SITE_LIST_FROM_DDHL, QSharedPointer<IDTO>(dto));
            });
    // clang-format on

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
}

void MainWindowUi::uploadToDDHL()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    UploadDDHLManagerDialog dialog(this);
    auto                    missionResults = _guiDTO.getMissionResults();
    if (missionResults) {
        dialog.setMissionResultList(missionResults->getList());
    }

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    if (dialog.getMissionsToUpload().isEmpty()) {
        return;
    }

    auto dto             = createSharedDTO<BasicStringListDTO>();
    dto->getStringList() = dialog.getMissionsToUpload();
    Q_EMIT sendDTOToCore(ActionType::UPLOAD_MISSION_RESULT_LIST_TO_DDHL, QSharedPointer<IDTO>(dto));
}

bool MainWindowUi::showConfirmationDialog(const QString& title, const QString& text)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dialog = new QMessageBox(this);
    dialog->setWindowModality(Qt::ApplicationModal);
    dialog->setWindowTitle(title);
    dialog->setText(text);
    dialog->setStandardButtons(QMessageBox::Yes | QMessageBox::No);

    return dialog->exec() == QMessageBox::Yes;
}

void MainWindowUi::showInspectionPlanFromSiteDialog(const SiteListDTO& siteList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    SelectInspecionPlanFromSiteDialog dialog(this);
    dialog.setSiteList(siteList);

    // clang-format off
    connect(&dialog, &SelectInspecionPlanFromSiteDialog::selectedInspectionPlan, 
            this, [&](const QString& selectedPlanUUID)
            {
                _guiDTO.getTargetInspectionPlanUUID() = selectedPlanUUID ;
                auto dto         = createSharedDTO<BasicStringDTO>();
                dto->getString() = _guiDTO.getTargetInspectionPlanUUID();
                Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_INSP_PLAN_FROM_DDHL, QSharedPointer<IDTO>(dto));

                writeTargetUUID(); 
            });
    // clang-format on

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
}

void MainWindowUi::loadInspectionPlan()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString inspectionPlanPath = QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/inspectionPlans").arg(PROJECT_VERSION_MAJOR).arg(PROJECT_VERSION_MINOR));

    if (!QDir(inspectionPlanPath).exists()) {
        auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
        msgConsoleDTO->getMsgType() = MsgType::INFO;
        msgConsoleDTO->getMessage() = QString("There isn't inspection plan folder to load");
        showMessage(msgConsoleDTO);
        return;
    }

    const QString jsonFilePath = QFileDialog::getOpenFileName(
            this,
            tr("Select inspection plan to load"),
            inspectionPlanPath,
            tr("Json File (*.json)"),
            nullptr,
            QFileDialog::DontUseNativeDialog);
    if (jsonFilePath != "") {
        auto dto         = createSharedDTO<BasicStringDTO>();
        dto->getString() = jsonFilePath;
        Q_EMIT sendDTOToCore(ActionType::LOAD_INSPECTION_PLAN_FROM_LOCAL, QSharedPointer<IDTO>(dto));
        Q_EMIT sendDTOToCore(ActionType::NEW_MISSION);
    }
}

void MainWindowUi::updateAssetProgress(const AssetProgressDTO& assetProgressDTO)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (!_assetDownloadprogressDialog.isVisible()) {
        if (_guiDTO.getDownloadAssetIsCanceled()) {
            return;
        }
        _assetDownloadprogressDialog.show();
    }

    const auto percProgress = (static_cast<float>(assetProgressDTO.getBytesRead())
                               / static_cast<float>(assetProgressDTO.getTotalBytes()))
                            * 100.0;

    _assetDownloadprogressDialog.setLabelText(
            QString("Downloading asset file: \"%1\"").arg(assetProgressDTO.getAssetFileName()));
    _assetDownloadprogressDialog.setValue(percProgress);

    if (percProgress == static_cast<quint32>(_assetDownloadprogressDialog.maximum())) {
        const auto fileList = _guiDTO.getInspectionPlan().getAsset().getDownloadedFileList().getDownloadedFileList();

        if (fileList.size() > 0) {
            const auto firstFileSize = fileList.first().getSize();

            auto msgConsoleDTO = createSharedDTO<MsgConsoleDTO>();
            if (firstFileSize >= assetProgressDTO.getBytesRead()) {
                msgConsoleDTO->getMsgType() = MsgType::INFO;
                msgConsoleDTO->getMessage() = tr("Asset File downloaded correctly");
            } else {
                msgConsoleDTO->getMsgType() = MsgType::WARNING;
                msgConsoleDTO->getMessage() = tr("Error downloading asset file");
            }

            showMessage(msgConsoleDTO);
            _assetDownloadprogressDialog.reset();
        }
    }
}

void MainWindowUi::cancelAssetDownloadProgress()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _guiDTO.getDownloadAssetIsCanceled() = true;
    _assetDownloadprogressDialog.cancel();

    Q_EMIT sendDTOToCore(ActionType::CANCEL_ASSET_DOWNLOAD);
}

void MainWindowUi::saveCurrentWaypointList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    Q_EMIT sendDTOToCore(ActionType::SAVE_CURRENT_WPTS_LIST);
}

void MainWindowUi::saveAsCurrentWaypointList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString waypointListsPath = QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/inspectionPlans/inspectionPlan_%3/waypoint_lists")
                    .arg(PROJECT_VERSION_MAJOR)
                    .arg(PROJECT_VERSION_MINOR)
                    .arg(_guiDTO.getInspectionPlan().getUUID()));

    if (!QDir(waypointListsPath).exists()) {
        QDir(waypointListsPath).mkpath(".");
    }

    QString missionPath = QFileDialog::getSaveFileName(
            this,
            tr("Save Waypoint List As"),
            waypointListsPath,
            tr("Waypoint Lists (*.json)"),
            0,
            QFileDialog::DontUseNativeDialog);

    if (!missionPath.isEmpty()) {
        if (QString::compare("json", QFileInfo(missionPath).suffix()) != 0) {
            missionPath.append(".json");
        }

        QLOG_INFO() << missionPath;

        auto basicStringDTO         = createSharedDTO<BasicStringDTO>();
        basicStringDTO->getString() = missionPath;
        Q_EMIT sendDTOToCore(ActionType::SAVE_AS_CURRENT_WPTS_LIST, QSharedPointer<IDTO>(basicStringDTO));
    }
}

void MainWindowUi::loadAssociatedWaypointList()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QString waypointListsPath = QDir::home().absoluteFilePath(
            QString(".pilotingGrcs/v%1.%2/inspectionPlans/inspectionPlan_%3/waypoint_lists")
                    .arg(PROJECT_VERSION_MAJOR)
                    .arg(PROJECT_VERSION_MINOR)
                    .arg(_guiDTO.getInspectionPlan().getUUID()));

    if (!QDir(waypointListsPath).exists()) {
        auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
        msgConsoleDTO->getMsgType() = MsgType::INFO;
        msgConsoleDTO->getMessage() = QString("There isn't waypoints lists folder to load");
        showMessage(msgConsoleDTO);
        return;
    }

    const QString wptsListFilePath = QFileDialog::getOpenFileName(
            this,
            tr("Select waypoint list"),
            waypointListsPath,
            tr("Json File (*.json)"),
            nullptr,
            QFileDialog::DontUseNativeDialog);
    if (wptsListFilePath != "") {
        auto dto         = createSharedDTO<BasicStringDTO>();
        dto->getString() = wptsListFilePath;
        Q_EMIT sendDTOToCore(ActionType::LOAD_ASSOCIATED_WPTS_LIST, QSharedPointer<IDTO>(dto));
    }
}

void MainWindowUi::modifyPreviewWpHeading(float increment)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dto        = createSharedDTO<BasicFloatDTO>();
    dto->getFloat() = increment;
    Q_EMIT sendDTOToCore(ActionType::MODIFY_PREVIEW_WP_HEADING, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::modifyCurrentWpHeading(float heading)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto dto        = createSharedDTO<BasicFloatDTO>();
    dto->getFloat() = heading;
    Q_EMIT sendDTOToCore(ActionType::MODIFY_CURR_WP_HEADING, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::selectFileAssetToDownload(const DownloadedFileListDTO& downloadedFileList)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _guiDTO.getInspectionPlan().getAsset().getDownloadedFileList() = downloadedFileList;

    DownloadAssetFileDialog dialog(this);
    dialog.setDownloadedFileList(downloadedFileList.getDownloadedFileList());

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    auto fileSelectedDTO = dialog.getSelectedFile();
    auto dto             = createSharedDTO<FileDTO>();
    *dto                 = fileSelectedDTO;
    Q_EMIT sendDTOToCore(ActionType::DOWNLOAD_ASSET_FROM_DDHL, QSharedPointer<IDTO>(dto));
}

void MainWindowUi::manageAutoHeading()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    ManageAutoHeadingDialog dialog(this);
    dialog.setCurrentAutoHeadingState(_guiDTO.getAutoHeadingMode());

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    _guiDTO.getAutoHeadingMode() = dialog.getAutoHeadingState();

    auto dto       = createSharedDTO<BasicBoolDTO>();
    dto->getBool() = _guiDTO.getAutoHeadingMode();
    Q_EMIT sendDTOToCore(ActionType::MANAGE_AUTOHEADING_WPTS, QSharedPointer<IDTO>(dto));
}
} // namespace gcs
