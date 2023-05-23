
#include "SendHomeAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>
#include <dataModel/dtos/HomeDTO.h>

#include <QMetaObject>

namespace gcs {

SendHomeAction::SendHomeAction(ICommunications& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

SendHomeAction::~SendHomeAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType SendHomeAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::SEND_HOME;
}

void SendHomeAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "SendHomeAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<HomeDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be HomeDTO");
        }

        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(
                _commsObj, "sendHome", Qt::BlockingQueuedConnection, Q_ARG(QSharedPointer<HomeDTO>, parsedDTO));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "SendHomeAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Send home error: - ") + tr(e.what()));
    }
}

} // namespace gcs
