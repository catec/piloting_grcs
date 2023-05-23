#include "CommsConnectAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>
#include <dataModel/dtos/CommsLinkDTO.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

CommsConnectAction::CommsConnectAction(ICommunications& comms, IMap& map, QObject* parent) :
        BasicAction(parent),
        _comms(comms),
        _commsObj(dynamic_cast<QObject*>(&_comms)),
        _map(map),
        _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << "CommsConnectAction::CommsConnectAction()";
}

CommsConnectAction::~CommsConnectAction()
{
    QLOG_TRACE() << "CommsConnectAction::~CommsConnectAction()";
}

ActionType CommsConnectAction::getActionType()
{
    QLOG_TRACE() << "CommsConnectAction::getActionType()";

    return ActionType::COMMS_CONNECT;
}

void CommsConnectAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "CommsConnectAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<CommsLinkDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CommsLinkDTO");
        }

        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(
                _commsObj, "connectTo", Qt::BlockingQueuedConnection, Q_ARG(QSharedPointer<CommsLinkDTO>, parsedDTO));

        if (_comms.isConnected()) {
            QMetaObject::invokeMethod(_mapObj, "initialize", Qt::BlockingQueuedConnection);
        }
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "CommsConnectAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Comms connection error - ") + tr(e.what()));
    }
}

} // namespace gcs
