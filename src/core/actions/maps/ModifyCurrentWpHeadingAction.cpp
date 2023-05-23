
#include "ModifyCurrentWpHeadingAction.h"

#include <QsLog/QsLog.h>
#include <core/dtos/BasicFloatDTO.h>
#include <maps/IMap.h>

#include <QMetaObject>

namespace gcs {

ModifyCurrentWpHeadingAction::ModifyCurrentWpHeadingAction(IMap& map, QObject* parent) :
        BasicAction(parent), _map(map), _mapObj(dynamic_cast<QObject*>(&_map))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ModifyCurrentWpHeadingAction::~ModifyCurrentWpHeadingAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType ModifyCurrentWpHeadingAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::MODIFY_CURR_WP_HEADING;
}

void ModifyCurrentWpHeadingAction::execute(QSharedPointer<IDTO> dto)
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
                _mapObj, "modifySelectedWpHeading", Qt::BlockingQueuedConnection, Q_ARG(float, parsedDTO->getFloat()));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Modify current wp heading error - ") + tr(e.what()));
    }
}

} // namespace gcs
