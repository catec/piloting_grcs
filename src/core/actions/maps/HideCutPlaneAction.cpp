
#include "HideCutPlaneAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/CloudEditionParametersDTO.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

HideCutPlaneAction::HideCutPlaneAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

HideCutPlaneAction::~HideCutPlaneAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType HideCutPlaneAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::HIDE_CUT_PLANE;
}

void HideCutPlaneAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(_mapObj, "hideCutPlane", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Restore pointcloud error - ") + tr(e.what()));
    }
}

} // namespace gcs
