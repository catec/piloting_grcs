#include "UpdateCursorPositionAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/CursorPositionDTO.h>

namespace gcs {

UpdateCursorPositionAction::UpdateCursorPositionAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << "UpdateCursorPositionAction::UpdateCursorPositionAction()";
}

UpdateCursorPositionAction::~UpdateCursorPositionAction()
{
    QLOG_TRACE() << "UpdateCursorPositionAction::~UpdateCursorPositionAction()";
}

ActionType UpdateCursorPositionAction::getActionType()
{
    QLOG_TRACE() << "UpdateCursorPositionAction::getActionType()";

    return ActionType::UPDATE_CURSOR_POSITION;
}

void UpdateCursorPositionAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateCursorPositionAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<CursorPositionDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CursorPositionDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateCursorPositionAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update cursor position error - ") + tr(e.what()));
    }
}

} // namespace gcs
