
#include "SaveAsCurrentWaypointListAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/DataModel.h>

namespace gcs {
SaveAsCurrentWaypointListAction::SaveAsCurrentWaypointListAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _missionBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

SaveAsCurrentWaypointListAction::~SaveAsCurrentWaypointListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType SaveAsCurrentWaypointListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::SAVE_AS_CURRENT_WPTS_LIST;
}

void SaveAsCurrentWaypointListAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        /// \note. This is a temporary solution.
        _missionBusiness.saveMission(_dataModel.getInspectionPlanDTO().getUUID(), parsedDTO->getString());
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
