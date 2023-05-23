
#include "SetCurrentWaypointItemAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>
#include <core/dtos/BasicIntDTO.h>

#include <QMetaObject>

namespace gcs {

SetCurrentWaypointItemAction::SetCurrentWaypointItemAction(ICommunications& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << "SetCurrentWaypointItemAction::SetCurrentWaypointItemAction()";
}

SetCurrentWaypointItemAction::~SetCurrentWaypointItemAction()
{
    QLOG_TRACE() << "SetCurrentWaypointItemAction::~SetCurrentWaypointItemAction()";
}

ActionType SetCurrentWaypointItemAction::getActionType()
{
    QLOG_TRACE() << "SetCurrentWaypointItemAction::getActionType()";

    return ActionType::SET_CURRENT_WAYPOINT_ITEM;
}

void SetCurrentWaypointItemAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "SetCurrentWaypointItemAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<BasicIntDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicIntDTO");
        }

        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(
                _commsObj,
                "setCurrentWaypointItem",
                Qt::BlockingQueuedConnection,
                Q_ARG(quint16, static_cast<quint16>(parsedDTO->getInt())));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "SetCurrentWaypointItemAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Set current waypoint item error - ") + tr(e.what()));
    }
}

} // namespace gcs
