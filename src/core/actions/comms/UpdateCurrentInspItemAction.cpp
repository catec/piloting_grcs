#include "UpdateCurrentInspItemAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
UpdateCurrentInspItemAction::UpdateCurrentInspItemAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateCurrentInspItemAction::~UpdateCurrentInspItemAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateCurrentInspItemAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_CURRENT_INSP_ITEM;
}

void UpdateCurrentInspItemAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateCurrentInspItemAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<CurrentInspItemDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CurrentInspItemDTO");
        }

        _dataModel.getCurrentInspItemDTO() = *parsedDTO;

        auto currentInspItemDTO = createSharedDTO<CurrentInspItemDTO>();
        *currentInspItemDTO     = _dataModel.getCurrentInspItemDTO();
        Q_EMIT sendDTO(currentInspItemDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateCurrentInspItemAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update current insp item error - ") + tr(e.what()));
    }
}

} // namespace gcs
