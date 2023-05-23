
#include "UpdateCurrentMissionResultAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

UpdateCurrentMissionResultAction::UpdateCurrentMissionResultAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateCurrentMissionResultAction::~UpdateCurrentMissionResultAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateCurrentMissionResultAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_CURRENT_MISSION_RESULT;
}

void UpdateCurrentMissionResultAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<CurrentMissionResultDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CurrentMissionResultDTO");
        }

        auto& currentMissionRes                    = _dataModel.getCurrentMissionResultDTO();
        currentMissionRes.getName()                = parsedDTO->getName();
        currentMissionRes.getDescription()         = parsedDTO->getDescription();
        currentMissionRes.getLastUpdateTimeStamp() = QDateTime::currentDateTime();

        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Current mission result updated ->"
                     << "Name:" << currentMissionRes.getName()
                     << "Creation date:" << currentMissionRes.getCreationTimeStamp().toString("yyyy-MM-dd hh:mm:ss")
                     << "LastUpdate date:" << currentMissionRes.getLastUpdateTimeStamp().toString("yyyy-MM-dd hh:mm:ss")
                     << "Description:" << currentMissionRes.getDescription()
                     << "Tasks:" << currentMissionRes.getMissionTaskList().size();

        _currentMissionResultBusiness.saveMissionResultToJson();

        auto currentMissionResultDTO = createSharedDTO<CurrentMissionResultDTO>();
        *currentMissionResultDTO     = _dataModel.getCurrentMissionResultDTO();
        Q_EMIT sendDTO(currentMissionResultDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "Error:" << e.what();

        sendWarningMsg(tr("Update current mission result error - ") + tr(e.what()));
    }
}

} // namespace gcs
