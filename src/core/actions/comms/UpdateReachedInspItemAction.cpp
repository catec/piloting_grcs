#include "UpdateReachedInspItemAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {
UpdateReachedInspItemAction::UpdateReachedInspItemAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateReachedInspItemAction::~UpdateReachedInspItemAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateReachedInspItemAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_REACHED_INSP_ITEM;
}

void UpdateReachedInspItemAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateReachedInspItemAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<ReachedInspItemDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be ReachedInspItemDTO");
        }

        _dataModel.getReachedInspItemDTO() = *parsedDTO;

        auto reachedInspItemDTO = createSharedDTO<ReachedInspItemDTO>();
        *reachedInspItemDTO     = _dataModel.getReachedInspItemDTO();
        Q_EMIT sendDTO(reachedInspItemDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateReachedInspItemAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update Reached Insp Item error - ") + tr(e.what()));
    }
}

} // namespace gcs
