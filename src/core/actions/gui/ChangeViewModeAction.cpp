
#include "ChangeViewModeAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

ChangeViewModeAction::ChangeViewModeAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _viewModeBusiness(dataModel.getViewModeDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "ChangeViewModeAction::ChangeViewModeAction()";
}

ChangeViewModeAction::~ChangeViewModeAction()
{
    QLOG_TRACE() << "ChangeViewModeAction::~ChangeViewModeAction()";
}

ActionType ChangeViewModeAction::getActionType()
{
    QLOG_TRACE() << "ChangeViewModeAction::getActionType()";

    return ActionType::CHANGE_VIEW_MODE;
}

void ChangeViewModeAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ChangeViewModeAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<ViewModeDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be ViewModeDTO");
        }

        _viewModeBusiness.changeMode(parsedDTO->getMode());

        auto viewModeDTO = createSharedDTO<ViewModeDTO>();
        *viewModeDTO     = _dataModel.getViewModeDTO();
        Q_EMIT sendDTO(viewModeDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ChangeViewModeAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Change view mode error - ") + tr(e.what()));
    }
}

} // namespace gcs
