#include "NewMissionAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

NewMissionAction::NewMissionAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _missionBusiness(dataModel.getMissionDTO()),
        _missionStatusBusiness(dataModel.getMissionStatusDTO()),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())

{
    QLOG_TRACE() << "NewMissionAction::NewMissionAction()";
}

NewMissionAction::~NewMissionAction()
{
    QLOG_TRACE() << "NewMissionAction::~NewMissionAction()";
}

ActionType NewMissionAction::getActionType()
{
    QLOG_TRACE() << "NewMissionAction::getActionType()";

    return ActionType::NEW_MISSION;
}

void NewMissionAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "NewMissionAction::execute()";

    try {
        _currentMissionResultBusiness.newMission(
                _dataModel.getMissionResultsDTO().getFolderPath(), _dataModel.getInspectionPlanDTO().getUUID());
        _missionBusiness.closeMission();
        _missionBusiness.newMission(_dataModel.getCurrentMissionResultDTO().getName());
        _missionStatusBusiness.changeStatus(MissionStatusType::WithChanges);

        auto missionStatusDTO = createSharedDTO<MissionStatusDTO>();
        *missionStatusDTO     = _dataModel.getMissionStatusDTO();
        Q_EMIT sendDTO(missionStatusDTO);

        auto missionDTO = createSharedDTO<MissionDTO>();
        *missionDTO     = _dataModel.getMissionDTO();
        Q_EMIT sendDTO(missionDTO);

        auto currentMissionResultDTO = createSharedDTO<CurrentMissionResultDTO>();
        *currentMissionResultDTO     = _dataModel.getCurrentMissionResultDTO();
        Q_EMIT sendDTO(currentMissionResultDTO);

        sendInfoMsg(tr("Synchronization ID created: %1").arg(_dataModel.getCurrentMissionResultDTO().getSyncId()));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "NewMissionAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("New mission error - ") + tr(e.what()));
    }
}

} // namespace gcs
