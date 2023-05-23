#pragma once

#include <common/IDTO.h>
#include <core/IAction.h>

#include <QDockWidget>
#include <QMainWindow>
#include <QProgressDialog>

#include "dialogs/cloudEditor/CloudEditorDialog.h"
#include "dialogs/commsProgress/CommsProgressDialog.h"
#include "dtos/GuiDTO.h"
#include "gcs_gui_export.h"
#include "widgets/consoleDisplay/ConsoleDisplayWidget.h"
#include "widgets/highLevelActions/HighLevelActionsWidget.h"
#include "widgets/inspectionPlan/InspectionPlanWidget.h"
#include "widgets/missionTasks/MissionTasksWidget.h"
#include "widgets/robotAlarms/RobotAlarmsWidget.h"
#include "widgets/robotControl/RobotControlWidget.h"
#include "widgets/telemetryTree/TelemetryTreeWidget.h"
#include "widgets/waypointsEditor/WaypointsEditorWidget.h"

namespace Ui {
class MainWindowUi;
}

namespace gcs {

class BasicWaypointDTO;
class MissionTaskDTO;
class CurrentMissionResultDTO;
class MsgConsoleDTO;
class CloudEditionParametersDTO;
class AssetProgressDTO;
class DownloadedFileListDTO;
class SiteListDTO;

class PILOTING_GRCS_GUI_EXPORT MainWindowUi : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindowUi(QWidget* parent = nullptr);
    virtual ~MainWindowUi();

    void setupInit(QWidget* mapView);

  public Q_SLOTS:
    void manageCoreDTO(QSharedPointer<IDTO>);

  private Q_SLOTS:
    void closeEvent(QCloseEvent*);

    void closeMission();

    void connectToRobot();
    void disconnectFromRobot();

    void showCommsOptions();

    void updateViewMode(bool);
    void updateEditionMode(bool);
    void updateAppStatus(bool);
    void updateRouteOptions(bool);

    void changeCurrentWaypoint(QSharedPointer<BasicWaypointDTO>);
    void editWaypointFeatures(QSharedPointer<WaypointDTO>);
    void editPointCloud(QSharedPointer<CloudEditionParametersDTO>);
    void restorePointCloud();
    void hideCutPlane();
    void hidePointCloud();

    // PILOTING actions
    void showDDHLOptions();
    void downloadFromDDHL();
    void uploadToDDHL();
    void showCheckList();
    void showVisualizacionConfig();
    void sendCommand(QSharedPointer<CommandLongDTO>);
    void uploadWaypointList();
    void downloadWaypointList();
    void downloadCheckList();
    void downloadAlarmsList();
    void downloadHighLevelActions();
    void sendHome();
    void setCurrentWaypointItem(quint16);
    void updateViewInspectionTask(QString);
    void updateMissionTask(QSharedPointer<MissionTaskDTO>);
    void updateMissionResult(QSharedPointer<CurrentMissionResultDTO>);

    void waypointsEditorLocationChanged(Qt::DockWidgetArea);

    void showMessage(QSharedPointer<MsgConsoleDTO>);
    void displayMsgBox(const MsgType&, const QString&, const QString&);

    void loadInspectionPlan();
    void saveCurrentWaypointList();
    void saveAsCurrentWaypointList();
    void loadAssociatedWaypointList();

    void modifyPreviewWpHeading(float);
    void modifyCurrentWpHeading(float);

    void manageAutoHeading();

  private:
    void connectActions();
    void connectActionsCloudEditor();

    void loadGeometry();
    void writeGeometry();

    void restoreViewState();
    void saveViewState();

    void loadMavsdkCommsOptions();
    void writeMavsdkCommsOptions();

    void loadDDHLOptions();
    void writeDDHLOptions();

    void loadVisualizationOptions();
    void writeVisualizationOptions();

    void loadTargetUUID();
    void writeTargetUUID();

    void setRobotControlDock(const Qt::DockWidgetArea);
    void setWaypointsEditorDock(const Qt::DockWidgetArea);
    void setRobotAlarmsDock(const Qt::DockWidgetArea);
    void setInspectionPlanDock(const Qt::DockWidgetArea);
    void setMissionDock(const Qt::DockWidgetArea);
    void setTelemetryTreeDock(const Qt::DockWidgetArea);
    void setConsoleDock(const Qt::DockWidgetArea);
    void setHighLevelActionsDock(const Qt::DockWidgetArea);
    void setAssetProgressDialog();

    bool checkIfCloseMission();

    void selectFileAssetToDownload(const DownloadedFileListDTO&);

    void updateConnectionStatus();
    void updateAppStatusButtons();
    void updateViewModeButtons();
    void updateEditionModeWidgets();
    void updateRouteOptionsButtons();
    void updateMissionName();
    void updateMissionButtons();
    void updateAssetProgress(const AssetProgressDTO&);

    void loadMissionResultsFromFolder();
    void loadXMLDataFromFolder();

    bool showConfirmationDialog(const QString&, const QString&);
    void showInspectionPlanFromSiteDialog(const SiteListDTO&);
    void cancelAssetDownloadProgress();

  private:
    std::unique_ptr<Ui::MainWindowUi> _ui;

    GuiDTO _guiDTO;

    QDockWidget _robotControlDW;
    QDockWidget _waypointsEditorDW;
    QDockWidget _robotAlarmsDW;
    QDockWidget _consoleDW;
    QDockWidget _inspectionPlanDW;
    QDockWidget _missionTasksDW;
    QDockWidget _telemetryTreeDW;
    QDockWidget _highLevelActionsDW;

    RobotControlWidget     _robotsControlWidget;
    RobotAlarmsWidget      _robotAlarmsWidget;
    TelemetryTreeWidget    _telemetryTreeWidget;
    ConsoleDisplayWidget   _consoleDisplayWidget;
    WaypointsEditorWidget  _waypointEditorWidget;
    InspectionPlanWidget   _inspectionPlanWidget;
    MissionTasksWidget     _missionWidget;
    HighLevelActionsWidget _highLevelActionsWidget;

    CommsProgressDialog _commsProgressDialog;
    CloudEditorDialog   _cloudEditorDialog;
    QProgressDialog     _assetDownloadprogressDialog;

  Q_SIGNALS:
    void sendDTOToCore(ActionType actionType, QSharedPointer<IDTO> dto = nullptr);
};

} // namespace gcs
