
#include "UpdateViewInspectionTaskAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicStringDTO.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

UpdateViewInspectionTaskAction::UpdateViewInspectionTaskAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << "UpdateViewInspectionTaskAction::UpdateViewInspectionTaskAction()";
}

UpdateViewInspectionTaskAction::~UpdateViewInspectionTaskAction()
{
    QLOG_TRACE() << "UpdateViewInspectionTaskAction::~UpdateViewInspectionTaskAction()";
}

ActionType UpdateViewInspectionTaskAction::getActionType()
{
    QLOG_TRACE() << "UpdateViewInspectionTaskAction::getActionType()";

    return ActionType::UPDATE_VIEW_INSPECTION_TASK;
}

void UpdateViewInspectionTaskAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateViewInspectionTaskAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(
                _mapObj,
                "updateViewInspectionTask",
                Qt::BlockingQueuedConnection,
                Q_ARG(QString, parsedDTO->getString()));

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateViewInspectionTaskAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update map adapter error - ") + tr(e.what()));
    }
}

} // namespace gcs
