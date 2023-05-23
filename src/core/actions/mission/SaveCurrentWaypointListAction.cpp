
#include "SaveCurrentWaypointListAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
SaveCurrentWaypointListAction::SaveCurrentWaypointListAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _missionBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

SaveCurrentWaypointListAction::~SaveCurrentWaypointListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType SaveCurrentWaypointListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::SAVE_CURRENT_WPTS_LIST;
}

void SaveCurrentWaypointListAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        /// \note. This is a temporary solution.
        _missionBusiness.saveMission(_dataModel.getInspectionPlanDTO().getUUID());
        _missionStatusBusiness.changeStatus(MissionStatusType::Saved);

        sendInfoMsg(QString("Successfully saved waypoint list associated to the Inspection Plan: %1")
                            .arg(_dataModel.getInspectionPlanDTO().getUUID()));

        auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
        *missionStatusDTO     = _dataModel.getMissionStatusDTO();
        Q_EMIT sendDTO(missionStatusDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();
        sendWarningMsg(tr("Save current waypoint list error - ") + tr(e.what()));
    }
}
} // namespace gcs
