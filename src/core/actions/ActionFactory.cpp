#include "ActionFactory.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <communications/ICommunications.h>
#include <dataModel/DataModel.h>
#include <maps/IMap.h>

#include "comms/CommsConnectAction.h"
#include "comms/CommsDisconnectAction.h"
#include "comms/DownloadAlarmsListAction.h"
#include "comms/DownloadCheckListAction.h"
#include "comms/DownloadHLActionsAction.h"
#include "comms/DownloadWaypointsListAction.h"
#include "comms/ReceiveAlarmListAction.h"
#include "comms/ReceiveCheckListAction.h"
#include "comms/ReceiveHLActionsAction.h"
#include "comms/RemoveLoadedHomeAction.h"
#include "comms/RemoveLoadedRouteAction.h"
#include "comms/SendCommandAction.h"
#include "comms/SendHomeAction.h"
#include "comms/SetCurrentWaypointItemAction.h"
#include "comms/UpdateAlarmStatusAction.h"
#include "comms/UpdateCheckListAction.h"
#include "comms/UpdateCommsProgressAction.h"
#include "comms/UpdateCommsStatusAction.h"
#include "comms/UpdateCurrentInspItemAction.h"
#include "comms/UpdateLoadedHomeAction.h"
#include "comms/UpdateLoadedRouteAction.h"
#include "comms/UpdateLoadedRouteFromCurrentRouteAction.h"
#include "comms/UpdateReachedInspItemAction.h"
#include "comms/UpdateRobotAttitudeAngVelocityAction.h"
#include "comms/UpdateRobotPositionVelocityAction.h"
#include "comms/UploadWaypointsListAction.h"
#include "commsDDHL/CancelAssetDownloadAction.h"
#include "commsDDHL/DownloadAssetFromDDHLAction.h"
#include "commsDDHL/DownloadInspPlanFromDDHLAction.h"
#include "commsDDHL/DownloadSiteListFromDDHLAction.h"
#include "commsDDHL/LoadCurrentInspectionPlanAction.h"
#include "commsDDHL/LoadInspectionPlanFromLocalAction.h"
#include "commsDDHL/UpdateAssetPathAction.h"
#include "commsDDHL/UpdateAssetProgressAction.h"
#include "commsDDHL/UpdateInspectionPlanAction.h"
#include "commsDDHL/UpdateSiteListAction.h"
#include "commsDDHL/UploadMissionResultListToDDHLAction.h"
#include "gui/ChangeApplicationStatusAction.h"
#include "gui/ChangeEditionModeAction.h"
#include "gui/ChangeRouteOptionsAction.h"
#include "gui/ChangeViewModeAction.h"
#include "gui/LoadXmlDataFromFolderAction.h"
#include "gui/ShowDialogAction.h"
#include "gui/UpdateCommsLinkDDHLAction.h"
#include "gui/UpdateCurrentMissionResultAction.h"
#include "gui/UpdateCursorPositionAction.h"
#include "gui/UpdateMissionTaskAction.h"
#include "gui/UpdateMsgConsoleAction.h"
#include "maps/AddMeasuringPointAction.h"
#include "maps/EditPointCloudAction.h"
#include "maps/HideCutPlaneAction.h"
#include "maps/HidePointCloudAction.h"
#include "maps/HidePreviewAxisAction.h"
#include "maps/ModifyCurrentWpHeadingAction.h"
#include "maps/ModifyPreviewWpHeadingAction.h"
#include "maps/RemoveMeasuringPointsAction.h"
#include "maps/RestorePointCloudAction.h"
#include "maps/UpdateCloudDimensionsAction.h"
#include "maps/UpdateViewInspectionTaskAction.h"
#include "maps/UpdateVisualizationConfigAction.h"
#include "mission/CloseMissionAction.h"
#include "mission/LoadAssociatedWaypointListAction.h"
#include "mission/LoadMissionResultListFromFolderAction.h"
#include "mission/NewMissionAction.h"
#include "mission/RemoveMissionResultAction.h"
#include "mission/SaveAsCurrentWaypointListAction.h"
#include "mission/SaveCurrentWaypointListAction.h"
#include "route/AddRouteAction.h"
#include "route/AddWaypointAction.h"
#include "route/ChangeSelectedWaypointAction.h"
#include "route/CopyLoadedRouteToLocalAction.h"
#include "route/EditWaypointAction.h"
#include "route/ManageAutoHeadingWptsAction.h"
#include "route/RemoveRouteAction.h"
#include "route/RemoveWaypointAction.h"

namespace gcs {

inline void checkIsNull(ICommunications* ptr)
{
    if (!ptr) {
        const QString msg("Comms is NULL");
        QLOG_ERROR() << "ActionFactory::create() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }
}

inline void checkIsNull(DDHLComms* ptr)
{
    if (!ptr) {
        const QString msg("Comms DDHL is NULL");
        QLOG_ERROR() << "ActionFactory::create() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }
}

inline void checkIsNull(IMap* ptr)
{
    if (!ptr) {
        const QString msg = QString("Map is NULL");
        QLOG_ERROR() << "ActionFactory::create() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }
}

std::unique_ptr<IAction> ActionFactory::create(
        const ActionType& type, DataModel& dataModel, ICommunications* comms, DDHLComms* commsDDHL, IMap* map)
{
    QLOG_TRACE() << "ActionFactory::create()";

    std::unique_ptr<IAction> instance;

    switch (type) {
        case ActionType::NEW_MISSION: {
            instance = std::make_unique<NewMissionAction>(dataModel);
            break;
        }
        case ActionType::CLOSE_MISSION: {
            instance = std::make_unique<CloseMissionAction>(dataModel);
            break;
        }
        case ActionType::ADD_ROUTE: {
            instance = std::make_unique<AddRouteAction>(dataModel);
            break;
        }
        case ActionType::REMOVE_ROUTE: {
            instance = std::make_unique<RemoveRouteAction>(dataModel);
            break;
        }
        case ActionType::ADD_WAYPOINT: {
            instance = std::make_unique<AddWaypointAction>(dataModel);
            break;
        }
        case ActionType::REMOVE_WAYPOINT: {
            instance = std::make_unique<RemoveWaypointAction>(dataModel);
            break;
        }
        case ActionType::EDIT_WAYPOINT: {
            instance = std::make_unique<EditWaypointAction>(dataModel);
            break;
        }
        case ActionType::CHANGE_SELECTED_WAYPOINT: {
            instance = std::make_unique<ChangeSelectedWaypointAction>(dataModel);
            break;
        }
        case ActionType::COPY_LOADED_ROUTE_TO_LOCAL: {
            instance = std::make_unique<CopyLoadedRouteToLocalAction>(dataModel);
            break;
        }
        case ActionType::MANAGE_AUTOHEADING_WPTS: {
            instance = std::make_unique<ManageAutoHeadingWptsAction>(dataModel);
            break;
        }
        case ActionType::CHANGE_APPLICATION_STATUS: {
            instance = std::make_unique<ChangeApplicationStatusAction>(dataModel);
            break;
        }
        case ActionType::CHANGE_EDITION_MODE: {
            instance = std::make_unique<ChangeEditionModeAction>(dataModel);
            break;
        }
        case ActionType::CHANGE_VIEW_MODE: {
            instance = std::make_unique<ChangeViewModeAction>(dataModel);
            break;
        }
        case ActionType::CHANGE_ROUTE_OPTIONS: {
            instance = std::make_unique<ChangeRouteOptionsAction>(dataModel);
            break;
        }
        case ActionType::SHOW_DIALOG: {
            instance = std::make_unique<ShowDialogAction>();
            break;
        }
        case ActionType::LOAD_MISSION_RESULT_LIST_FROM_FOLDER: {
            instance = std::make_unique<LoadMissionResultListFromFolderAction>(dataModel);
            break;
        }
        case ActionType::LOAD_XML_DATA_FROM_FOLDER: {
            instance = std::make_unique<LoadXmlDataFromFolderAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_CURSOR_POSITION: {
            instance = std::make_unique<UpdateCursorPositionAction>();
            break;
        }
        case ActionType::UPDATE_MSG_CONSOLE: {
            instance = std::make_unique<UpdateMsgConsoleAction>();
            break;
        }
        case ActionType::ADD_MEASURING_POINT: {
            instance = std::make_unique<AddMeasuringPointAction>(dataModel);
            break;
        }
        case ActionType::REMOVE_MEASURING_POINTS: {
            instance = std::make_unique<RemoveMeasuringPointsAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_CURRENT_MISSION_RESULT: {
            instance = std::make_unique<UpdateCurrentMissionResultAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_MISSION_TASK: {
            instance = std::make_unique<UpdateMissionTaskAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_LOADED_HOME: {
            instance = std::make_unique<UpdateLoadedHomeAction>(dataModel);
            break;
        }
        case ActionType::REMOVE_LOADED_ROUTE: {
            instance = std::make_unique<RemoveLoadedRouteAction>(dataModel);
            break;
        }
        case ActionType::REMOVE_LOADED_HOME: {
            instance = std::make_unique<RemoveLoadedHomeAction>(dataModel);
            break;
        }
        case ActionType::COMMS_CONNECT: {
            checkIsNull(comms);
            checkIsNull(map);

            instance = std::make_unique<CommsConnectAction>(*comms, *map);
            break;
        }
        case ActionType::COMMS_DISCONNECT: {
            checkIsNull(comms);
            checkIsNull(map);

            instance = std::make_unique<CommsDisconnectAction>(*comms, *map);
            break;
        }
        case ActionType::UPDATE_COMMS_STATUS: {
            instance = std::make_unique<UpdateCommsStatusAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_COMMS_PROGRESS: {
            instance = std::make_unique<UpdateCommsProgressAction>();
            break;
        }
        case ActionType::UPDATE_LOADED_ROUTE: {
            instance = std::make_unique<UpdateLoadedRouteAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_LOADED_ROUTE_FROM_CURRENT_ROUTE: {
            instance = std::make_unique<UpdateLoadedRouteFromCurrentRouteAction>(dataModel);
            break;
        }
        case ActionType::SEND_COMMAND: {
            checkIsNull(comms);

            instance = std::make_unique<SendCommandAction>(*comms);
            break;
        }
        case ActionType::SEND_HOME: {
            checkIsNull(comms);

            instance = std::make_unique<SendHomeAction>(*comms);
            break;
        }
        case ActionType::UPLOAD_WAYPOINTS_LIST: {
            checkIsNull(comms);

            instance = std::make_unique<UploadWaypointsListAction>(dataModel, *comms);
            break;
        }
        case ActionType::DOWNLOAD_WAYPOINTS_LIST: {
            checkIsNull(comms);

            instance = std::make_unique<DownloadWaypointsListAction>(*comms);
            break;
        }
        case ActionType::DOWNLOAD_CHECKLIST: {
            checkIsNull(comms);

            instance = std::make_unique<DownloadCheckListAction>(*comms);
            break;
        }
        case ActionType::UPDATE_CHECKLIST: {
            instance = std::make_unique<UpdateCheckListAction>(dataModel);
            break;
        }
        case ActionType::DOWNLOAD_ALARMS_LIST: {
            checkIsNull(comms);

            instance = std::make_unique<DownloadAlarmsListAction>(*comms);
            break;
        }
        case ActionType::DOWNLOAD_HLACTION_LIST: {
            checkIsNull(comms);

            instance = std::make_unique<DownloadHLActionsAction>(*comms);
            break;
        }
        case ActionType::SET_CURRENT_WAYPOINT_ITEM: {
            checkIsNull(comms);

            instance = std::make_unique<SetCurrentWaypointItemAction>(*comms);
            break;
        }
        case ActionType::RECEIVE_CHECKLIST: {
            instance = std::make_unique<ReceiveCheckListAction>(dataModel);
            break;
        }
        case ActionType::RECEIVE_ALARMS_LIST: {
            instance = std::make_unique<ReceiveAlarmListAction>();
            break;
        }
        case ActionType::RECEIVE_HLACTION_LIST: {
            instance = std::make_unique<ReceiveHLActionsAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_CURRENT_INSP_ITEM: {
            instance = std::make_unique<UpdateCurrentInspItemAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_REACHED_INSP_ITEM: {
            instance = std::make_unique<UpdateReachedInspItemAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_POSITION_VELOCITY_NED: {
            instance = std::make_unique<UpdateRobotPositionVelocityAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_ATTITUDE_ANGVELOCITY: {
            instance = std::make_unique<UpdateRobotAttitudeAngVelocityAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_ASSET_PROGRESS: {
            instance = std::make_unique<UpdateAssetProgressAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_ALARM_STATUS: {
            instance = std::make_unique<UpdateAlarmStatusAction>();
            break;
        }
        case ActionType::UPDATE_VIEW_INSPECTION_TASK: {
            checkIsNull(map);

            instance = std::make_unique<UpdateViewInspectionTaskAction>(*map);
            break;
        }
        case ActionType::UPDATE_CLOUD_DIMENSIONS: {
            instance = std::make_unique<UpdateCloudDimensionsAction>();
            break;
        }
        case ActionType::UPDATE_VISUALIZATION_CONFIG: {
            instance = std::make_unique<UpdateVisualizationConfigAction>();
            break;
        }
        case ActionType::EDIT_POINTCLOUD: {
            checkIsNull(map);

            instance = std::make_unique<EditPointCloudAction>(*map);
            break;
        }
        case ActionType::RESTORE_POINTCLOUD: {
            checkIsNull(map);

            instance = std::make_unique<RestorePointCloudAction>(*map);
            break;
        }
        case ActionType::HIDE_POINTCLOUD: {
            checkIsNull(map);

            instance = std::make_unique<HidePointCloudAction>(*map);
            break;
        }
        case ActionType::HIDE_CUT_PLANE: {
            checkIsNull(map);

            instance = std::make_unique<HideCutPlaneAction>(*map);
            break;
        }
        case ActionType::HIDE_PREVIEW_AXIS: {
            checkIsNull(map);

            instance = std::make_unique<HidePreviewAxisAction>(*map);
            break;
        }
        case ActionType::MODIFY_PREVIEW_WP_HEADING: {
            checkIsNull(map);

            instance = std::make_unique<ModifyPreviewWpHeadingAction>(*map);
            break;
        }
        case ActionType::MODIFY_CURR_WP_HEADING: {
            checkIsNull(map);

            instance = std::make_unique<ModifyCurrentWpHeadingAction>(*map);
            break;
        }
        case ActionType::DOWNLOAD_INSP_PLAN_FROM_DDHL: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<DownloadInspPlanFromDDHLAction>(*commsDDHL, dataModel);
            break;
        }
        case ActionType::DOWNLOAD_SITE_LIST_FROM_DDHL: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<DownloadSiteListFromDDHLAction>(*commsDDHL, dataModel);
            break;
        }
        case ActionType::DOWNLOAD_ASSET_FROM_DDHL: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<DownloadAssetFromDDHLAction>(*commsDDHL, dataModel);
            break;
        }
        case ActionType::UPDATE_COMMS_LINK_DDHL: {
            instance = std::make_unique<UpdateCommsLinkDDHLAction>(dataModel);
            break;
        }
        case ActionType::CANCEL_ASSET_DOWNLOAD: {
            checkIsNull(commsDDHL);
            instance = std::make_unique<CancelAssetDownloadAction>(*commsDDHL);
            break;
        }
        case ActionType::UPLOAD_MISSION_RESULT_LIST_TO_DDHL: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<UploadMissionResultListToDDHLAction>(*commsDDHL, dataModel);
            break;
        }
        case ActionType::LOAD_INSPECTION_PLAN_FROM_LOCAL: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<LoadInspectionPlanFromLocalAction>(*commsDDHL, dataModel);
            break;
        }
        case ActionType::REMOVE_MISSION_RESULT: {
            instance = std::make_unique<RemoveMissionResultAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_INSPECTION_PLAN: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<UpdateInspectionPlanAction>(*commsDDHL, dataModel);
            break;
        }
        case ActionType::LOAD_CURRENT_INSPECTION_PLAN: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<LoadCurrentInspectionPlanAction>(dataModel);
            break;
        }
        case ActionType::UPDATE_SITE_LIST: {
            checkIsNull(commsDDHL);

            instance = std::make_unique<UpdateSiteListAction>();
            break;
        }
        case ActionType::UPDATE_ASSET_PATH: {
            instance = std::make_unique<UpdateAssetPathAction>(dataModel);
            break;
        }
        case ActionType::SAVE_CURRENT_WPTS_LIST: {
            instance = std::make_unique<SaveCurrentWaypointListAction>(dataModel);
            break;
        }
        case ActionType::SAVE_AS_CURRENT_WPTS_LIST: {
            instance = std::make_unique<SaveAsCurrentWaypointListAction>(dataModel);
            break;
        }
        case ActionType::LOAD_ASSOCIATED_WPTS_LIST: {
            instance = std::make_unique<LoadAssociatedWaypointListAction>(dataModel);
            break;
        }
        default: {
            const auto msg = QString("Unknown action type: %1").arg(static_cast<uint8_t>(type));
            QLOG_ERROR() << "ActionFactory::create() -" << msg;
            throw std::runtime_error(msg.toStdString());
        }
    }

    return instance;
}

} // namespace gcs
