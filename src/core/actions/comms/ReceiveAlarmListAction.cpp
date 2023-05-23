
#include "ReceiveAlarmListAction.h"

#include <QsLog/QsLog.h>
#include <communications/MavSDK/dtos/AlarmListDTO.h>

namespace gcs {

ReceiveAlarmListAction::ReceiveAlarmListAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ReceiveAlarmListAction::~ReceiveAlarmListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType ReceiveAlarmListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::RECEIVE_ALARMS_LIST;
}

void ReceiveAlarmListAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ReceiveAlarmListAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<AlarmListDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be AlarmListDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ReceiveAlarmListAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Receive alarms list error - ") + tr(e.what()));
    }
}

} // namespace gcs
