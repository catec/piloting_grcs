#include "ChangeEditionModeAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

ChangeEditionModeAction::ChangeEditionModeAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _editionModeBusiness(dataModel.getEditionModeDTO()),
        _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "ChangeEditionModeAction::ChangeEditionModeAction()";
}

ChangeEditionModeAction::~ChangeEditionModeAction()
{
    QLOG_TRACE() << "ChangeEditionModeAction::~ChangeEditionModeAction()";
}

ActionType ChangeEditionModeAction::getActionType()
{
    QLOG_TRACE() << "ChangeEditionModeAction::getActionType()";

    return ActionType::CHANGE_EDITION_MODE;
}

void ChangeEditionModeAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ChangeEditionModeAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<EditionModeDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be EditionModeDTO");
        }

        _appStatusBusiness.changeStatus(AppStatusType::Panning);
        auto appStatusDTO = createSharedDTO<ApplicationStatusDTO>();
        *appStatusDTO     = _dataModel.getApplicationStatusDTO();
        Q_EMIT sendDTO(appStatusDTO);

        _editionModeBusiness.changeMode(parsedDTO->getMode());
        auto editionModeDTO = createSharedDTO<EditionModeDTO>();
        *editionModeDTO     = _dataModel.getEditionModeDTO();
        Q_EMIT sendDTO(editionModeDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ChangeEditionModeAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Change edition mode error - ") + tr(e.what()));
    }
}

} // namespace gcs
