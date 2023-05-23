#include "ChangeApplicationStatusAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

ChangeApplicationStatusAction::ChangeApplicationStatusAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _appStatusBusiness(dataModel.getApplicationStatusDTO())
{
    QLOG_TRACE() << "ChangeApplicationStatusAction::ChangeApplicationStatusAction()";
}

ChangeApplicationStatusAction::~ChangeApplicationStatusAction()
{
    QLOG_TRACE() << "ChangeApplicationStatusAction::~ChangeApplicationStatusAction()";
}

ActionType ChangeApplicationStatusAction::getActionType()
{
    QLOG_TRACE() << "ChangeApplicationStatusAction::getActionType()";

    return ActionType::CHANGE_APPLICATION_STATUS;
}

void ChangeApplicationStatusAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ChangeApplicationStatusAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<ApplicationStatusDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be ApplicationStatusDTO");
        }

        _appStatusBusiness.changeStatus(parsedDTO->getStatus());

        auto appStatusDTO = createSharedDTO<ApplicationStatusDTO>();
        *appStatusDTO     = _dataModel.getApplicationStatusDTO();
        Q_EMIT sendDTO(appStatusDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "ChangeApplicationStatusAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Change application status error - ") + tr(e.what()));
    }
}

} // namespace gcs
