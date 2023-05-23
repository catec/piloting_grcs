
#include "UpdateAlarmStatusAction.h"

#include <QsLog/QsLog.h>
#include <communications/MavSDK/dtos/AlarmStatusDTO.h>

namespace gcs {

UpdateAlarmStatusAction::UpdateAlarmStatusAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateAlarmStatusAction::~UpdateAlarmStatusAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateAlarmStatusAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_ALARM_STATUS;
}

void UpdateAlarmStatusAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateAlarmStatusAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<AlarmStatusDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be AlarmListDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateAlarmStatusAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update Alarms status error - ") + tr(e.what()));
    }
}

} // namespace gcs
