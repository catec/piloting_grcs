#include "UpdateCloudDimensionsAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/CloudDimensionsDTO.h>

namespace gcs {

UpdateCloudDimensionsAction::UpdateCloudDimensionsAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateCloudDimensionsAction::~UpdateCloudDimensionsAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateCloudDimensionsAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_CLOUD_DIMENSIONS;
}

void UpdateCloudDimensionsAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<CloudDimensionsDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CloudDimensionsDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update cloud dimensions error - ") + tr(e.what()));
    }
}

} // namespace gcs
