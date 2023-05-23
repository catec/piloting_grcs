#include "UpdateCheckListAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
UpdateCheckListAction::UpdateCheckListAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateCheckListAction::~UpdateCheckListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateCheckListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_CHECKLIST;
}

void UpdateCheckListAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateCheckListAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<CheckListDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CheckListDTO");
        }

        _currentMissionResultBusiness.saveCheckListToFile(*parsedDTO);

        auto currentMissionResultDTO = createSharedDTO<CurrentMissionResultDTO>();
        *currentMissionResultDTO     = _dataModel.getCurrentMissionResultDTO();

        Q_EMIT sendDTO(currentMissionResultDTO);

    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateCheckListAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update check list error - ") + tr(e.what()));
    }
}

} // namespace gcs
