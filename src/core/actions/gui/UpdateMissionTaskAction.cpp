
#include "UpdateMissionTaskAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

UpdateMissionTaskAction::UpdateMissionTaskAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateMissionTaskAction::~UpdateMissionTaskAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateMissionTaskAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_MISSION_TASK;
}

void UpdateMissionTaskAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<MissionTaskDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be MissionTaskDTO");
        }

        _currentMissionResultBusiness.updateMissionTask(*parsedDTO);
        _dataModel.getCurrentMissionResultDTO().getLastUpdateTimeStamp() = QDateTime::currentDateTime();

        //   QLOG_DEBUG() << __PRETTY_FUNCTION__
        //                << "Mission task updated->"
        //                << "name: " << _dataModel.getCurrentMissionResultDTO().getName()
        //                << "Creation date: " <<
        //                _dataModel.getCurrentMissionResultDTO().getCreationTimeStamp().toString("yyyy-MM-dd hh:mm:ss")
        //                << "description: " << _dataModel.getCurrentMissionResultDTO().getDescription()
        //                << "mission taks size: " <<
        //                _dataModel.getCurrentMissionResultDTO().getMissionTaskList().size();

        _currentMissionResultBusiness.saveMissionResultToJson();

        auto currentMissionResultDTO = createSharedDTO<CurrentMissionResultDTO>();
        *currentMissionResultDTO     = _dataModel.getCurrentMissionResultDTO();
        Q_EMIT sendDTO(currentMissionResultDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Error:" << e.what();

        sendWarningMsg(tr("Update mission result error - ") + tr(e.what()));
    }
}

} // namespace gcs
