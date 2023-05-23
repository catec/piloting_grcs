
#include "LoadAssociatedWaypointListAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/DataModel.h>

#include <QFileInfo>

namespace gcs {
LoadAssociatedWaypointListAction::LoadAssociatedWaypointListAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _missionBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

LoadAssociatedWaypointListAction::~LoadAssociatedWaypointListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType LoadAssociatedWaypointListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::LOAD_ASSOCIATED_WPTS_LIST;
}

void LoadAssociatedWaypointListAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        /// \note. Has been commented because in case the wpts file is corrupted, the mission will be deleted.
        // _missionBusiness.closeMission();

        /// \note. The UUID is used to ensure that the waypoint list is associated with the Inspection Plan.
        _missionBusiness.openMission(parsedDTO->getString(), _dataModel.getInspectionPlanDTO().getUUID());
        _missionStatusBusiness.changeStatus(MissionStatusType::Saved);

        auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
        *missionStatusDTO     = _dataModel.getMissionStatusDTO();
        Q_EMIT sendDTO(missionStatusDTO);

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);

        QFileInfo fi(parsedDTO->getString());
        sendInfoMsg("Successfully loaded associated waypoint list with name: " + fi.fileName());
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();
        sendWarningMsg(tr("Open waypoint list error - ") + tr(e.what()));
    }
}
} // namespace gcs