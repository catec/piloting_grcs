#include "UpdateVisualizationConfigAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/VisualizationConfigDTO.h>

namespace gcs {

UpdateVisualizationConfigAction::UpdateVisualizationConfigAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateVisualizationConfigAction::~UpdateVisualizationConfigAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateVisualizationConfigAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_VISUALIZATION_CONFIG;
}

void UpdateVisualizationConfigAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<VisualizationConfigDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be VisualizationConfigDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update cloud dimensions error - ") + tr(e.what()));
    }
}

} // namespace gcs
