
#include "HidePreviewAxisAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/CloudEditionParametersDTO.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

HidePreviewAxisAction::HidePreviewAxisAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

HidePreviewAxisAction::~HidePreviewAxisAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType HidePreviewAxisAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::HIDE_PREVIEW_AXIS;
}

void HidePreviewAxisAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(_mapObj, "hidePreviewAxis", Qt::BlockingQueuedConnection);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Hide preview axis error - ") + tr(e.what()));
    }
}

} // namespace gcs
