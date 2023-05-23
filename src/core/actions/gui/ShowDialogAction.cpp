#include "ShowDialogAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/DialogDTO.h>

namespace gcs {

ShowDialogAction::ShowDialogAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << "ShowDialogAction::ShowDialogAction()";
}

ShowDialogAction::~ShowDialogAction()
{
    QLOG_TRACE() << "ShowDialogAction::~ShowDialogAction()";
}

ActionType ShowDialogAction::getActionType()
{
    QLOG_TRACE() << "ShowDialogAction::getActionType()";

    return ActionType::SHOW_DIALOG;
}

void ShowDialogAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ShowDialogAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<DialogDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be DialogDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ShowDialogAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Show dialog error - ") + tr(e.what()));
    }
}

} // namespace gcs
