#include "CloseMissionAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

CloseMissionAction::CloseMissionAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _missionStatusBusiness(dataModel.getMissionStatusDTO()),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "CloseMissionAction::CloseMissionAction()";
}

CloseMissionAction::~CloseMissionAction()
{
    QLOG_TRACE() << "CloseMissionAction::~CloseMissionAction()";
}

ActionType CloseMissionAction::getActionType()
{
    QLOG_TRACE() << "CloseMissionAction::getActionType()";

    return ActionType::CLOSE_MISSION;
}

void CloseMissionAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "CloseMissionAction::execute()";

    _missionStatusBusiness.changeStatus(MissionStatusType::NoMission);
    _currentMissionResultBusiness.closeMission();

    auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
    *missionStatusDTO     = _dataModel.getMissionStatusDTO();
    Q_EMIT sendDTO(missionStatusDTO);

    //   _appStatusBusiness.changeStatus(AppStatusType::PanningThreeDView);
    //   auto appStatusDTO = createSharedDTO<ApplicationStatusDTO>();
    //   *appStatusDTO = _dataModel.getApplicationStatusDTO();
    //   Q_EMIT sendDTO(appStatusDTO);

    auto missionDTO = createSharedDTO<MissionDTO>();
    *missionDTO     = _dataModel.getMissionDTO();
    Q_EMIT sendDTO(missionDTO);

    auto currentMissionResultDTO = createSharedDTO<CurrentMissionResultDTO>();
    *currentMissionResultDTO     = _dataModel.getCurrentMissionResultDTO();
    Q_EMIT sendDTO(currentMissionResultDTO);
}

} // namespace gcs
