
#include "RestorePointCloudAction.h"

#include <QsLog/QsLog.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

RestorePointCloudAction::RestorePointCloudAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

RestorePointCloudAction::~RestorePointCloudAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType RestorePointCloudAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::RESTORE_POINTCLOUD;
}

void RestorePointCloudAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(_mapObj, "restorePointCloud", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Restore pointcloud error - ") + tr(e.what()));
    }
}

} // namespace gcs
