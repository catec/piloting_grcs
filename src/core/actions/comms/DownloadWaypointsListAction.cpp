
#include "DownloadWaypointsListAction.h"

#include <QsLog/QsLog.h>
#include <communications/ICommunications.h>

#include <QMetaObject>

namespace gcs {

DownloadWaypointsListAction::DownloadWaypointsListAction(ICommunications& comms, QObject* parent) :
        BasicAction(parent), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

DownloadWaypointsListAction::~DownloadWaypointsListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType DownloadWaypointsListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::DOWNLOAD_WAYPOINTS_LIST;
}

void DownloadWaypointsListAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "DownloadWaypointsListAction::execute()";

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        QMetaObject::invokeMethod(_commsObj, "downloadWaypointsList", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "DownloadWaypointsListAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Download waypoint list error - ") + tr(e.what()));
    }
}

} // namespace gcs
