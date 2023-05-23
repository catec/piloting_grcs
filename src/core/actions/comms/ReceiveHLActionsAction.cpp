
#include "ReceiveHLActionsAction.h"

#include <QsLog/QsLog.h>
#include <communications/MavSDK/dtos/HLActionListDTO.h>
#include <dataModel/DataModel.h>

namespace gcs {

ReceiveHLActionsAction::ReceiveHLActionsAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _xmlDataBusiness(dataModel.getSupportedCommandListDTO())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ReceiveHLActionsAction::~ReceiveHLActionsAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType ReceiveHLActionsAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::RECEIVE_HLACTION_LIST;
}

void ReceiveHLActionsAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ReceiveHLActionsAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<HLActionListDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be HLActionListDTO");
        }

        _xmlDataBusiness.associateHLActionWithSupportedCommand(*parsedDTO);

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ReceiveHLActionsAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Receive HLActionList error - ") + tr(e.what()));
    }
}

} // namespace gcs
