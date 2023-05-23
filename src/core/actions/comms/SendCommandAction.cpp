
#include "SendCommandAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>
#include <communications/MavSDK/dtos/CommandLongDTO.h>

#include <QMetaObject>

namespace gcs {

SendCommandAction::SendCommandAction(ICommunications& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

SendCommandAction::~SendCommandAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType SendCommandAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::SEND_COMMAND;
}

void SendCommandAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "SendCommandAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<CommandLongDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CommandLongDTO");
        }

        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(
                _commsObj,
                "sendCommand",
                Qt::BlockingQueuedConnection,
                Q_ARG(QSharedPointer<CommandLongDTO>, parsedDTO));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "SendCommandAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Send command error: - ") + tr(e.what()));
    }
}

} // namespace gcs
