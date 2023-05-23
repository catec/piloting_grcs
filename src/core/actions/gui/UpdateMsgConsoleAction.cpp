#include "UpdateMsgConsoleAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/MsgConsoleDTO.h>

namespace gcs {

UpdateMsgConsoleAction::UpdateMsgConsoleAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << "UpdateMsgConsoleAction::UpdateMsgConsoleAction()";
}

UpdateMsgConsoleAction::~UpdateMsgConsoleAction()
{
    QLOG_TRACE() << "UpdateMsgConsoleAction::~UpdateMsgConsoleAction()";
}

ActionType UpdateMsgConsoleAction::getActionType()
{
    QLOG_TRACE() << "UpdateMsgConsoleAction::getActionType()";

    return ActionType::UPDATE_MSG_CONSOLE;
}

void UpdateMsgConsoleAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateMsgConsoleAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<MsgConsoleDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be MsgConsoleDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateMsgConsoleAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update msg console error - ") + tr(e.what()));
    }
}

} // namespace gcs
