
#include "EditPointCloudAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/CloudEditionParametersDTO.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

EditPointCloudAction::EditPointCloudAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

EditPointCloudAction::~EditPointCloudAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType EditPointCloudAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::EDIT_POINTCLOUD;
}

void EditPointCloudAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<CloudEditionParametersDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CloudEditionParametersDTO");
        }

        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(
                _mapObj,
                "editPointCloud",
                Qt::BlockingQueuedConnection,
                Q_ARG(QSharedPointer<CloudEditionParametersDTO>, parsedDTO));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Edit pointcloud error - ") + tr(e.what()));
    }
}

} // namespace gcs
