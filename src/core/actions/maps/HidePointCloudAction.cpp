
#include "HidePointCloudAction.h"

#include <QsLog/QsLog.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

HidePointCloudAction::HidePointCloudAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

HidePointCloudAction::~HidePointCloudAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType HidePointCloudAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::HIDE_POINTCLOUD;
}

void HidePointCloudAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(_mapObj, "hidePointCloud", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Hide pointcloud error - ") + tr(e.what()));
    }
}

} // namespace gcs
