#include "RemoveMissionResultAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

RemoveMissionResultAction::RemoveMissionResultAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _missionResultsBusiness(dataModel.getMissionResultsDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

RemoveMissionResultAction::~RemoveMissionResultAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType RemoveMissionResultAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::REMOVE_MISSION_RESULT;
}

void RemoveMissionResultAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<MissionResultDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be MissionResultDTO");
        }

        _missionResultsBusiness.removeMissionResult(*parsedDTO);

        auto missionResultsDTO = createSharedDTO<MissionResultsDTO>();
        *missionResultsDTO     = _dataModel.getMissionResultsDTO();
        Q_EMIT sendDTO(missionResultsDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << "- Error:" << e.what();

        sendWarningMsg(tr("Remove Mission Result error - ") + tr(e.what()));
    }
}

} // namespace gcs
