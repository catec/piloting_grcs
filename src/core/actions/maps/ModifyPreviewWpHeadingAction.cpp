
#include "ModifyPreviewWpHeadingAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicFloatDTO.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

ModifyPreviewWpHeadingAction::ModifyPreviewWpHeadingAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ModifyPreviewWpHeadingAction::~ModifyPreviewWpHeadingAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType ModifyPreviewWpHeadingAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::MODIFY_PREVIEW_WP_HEADING;
}

void ModifyPreviewWpHeadingAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicFloatDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicFloatDTO");
        }

        if (!_mapObj) {
            throw std::runtime_error("Map object is NULL");
        }

        QMetaObject::invokeMethod(
                _mapObj, "modifyPreviewWpHeading", Qt::BlockingQueuedConnection, Q_ARG(float, parsedDTO->getFloat()));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Modify preview wp heading error - ") + tr(e.what()));
    }
}

} // namespace gcs
