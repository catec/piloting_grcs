#include "CommsDisconnectAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

CommsDisconnectAction::CommsDisconnectAction(ICommunications& comms, IMap& map, QObject* parent) :
        BasicAction(parent),
        _comms(comms),
        _commsObj(dynamic_cast<QObject*>(&_comms)),
        _map(map),
        _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << "CommsDisconnectAction::CommsDisconnectAction()";
}

CommsDisconnectAction::~CommsDisconnectAction()
{
    QLOG_TRACE() << "CommsDisconnectAction::~CommsDisconnectAction()";
}

ActionType CommsDisconnectAction::getActionType()
{
    QLOG_TRACE() << "CommsDisconnectAction::getActionType()";

    return ActionType::COMMS_DISCONNECT;
}

void CommsDisconnectAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "CommsDisconnectAction::execute()";

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(_commsObj, "disconnectFrom", Qt::BlockingQueuedConnection);

        if (!_comms.isConnected()) {
            QMetaObject::invokeMethod(_mapObj, "resetVisualizer", Qt::BlockingQueuedConnection);
        }
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "CommsDisconnectAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Comms disconnect error - ") + tr(e.what()));
    }
}

} // namespace gcs
