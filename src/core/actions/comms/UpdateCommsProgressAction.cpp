#include "UpdateCommsProgressAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/CommsProgressDTO.h>

namespace gcs {

UpdateCommsProgressAction::UpdateCommsProgressAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << "UpdateCommsProgressAction::UpdateCommsProgressAction()";
}

UpdateCommsProgressAction::~UpdateCommsProgressAction()
{
    QLOG_TRACE() << "UpdateCommsProgressAction::~UpdateCommsProgressAction()";
}

ActionType UpdateCommsProgressAction::getActionType()
{
    QLOG_TRACE() << "UpdateCommsProgressAction::getActionType()";

    return ActionType::UPDATE_COMMS_PROGRESS;
}

void UpdateCommsProgressAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateCommsProgressAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<CommsProgressDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CommsProgressDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateCommsProgressAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update comms progress error - ") + tr(e.what()));
    }
}

} // namespace gcs
