#include "ReceiveCheckListAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

ReceiveCheckListAction::ReceiveCheckListAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _currentMissionResultBusiness(dataModel.getCurrentMissionResultDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ReceiveCheckListAction::~ReceiveCheckListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType ReceiveCheckListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::RECEIVE_CHECKLIST;
}

void ReceiveCheckListAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ReceiveCheckListAction::execute()";

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
        QLOG_WARN() << "ReceiveCheckListAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Receive CheckList error - ") + tr(e.what()));
    }
}

} // namespace gcs
