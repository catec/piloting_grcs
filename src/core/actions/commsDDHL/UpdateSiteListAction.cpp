#include "UpdateSiteListAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/SiteListDTO.h>

namespace gcs {

UpdateSiteListAction::UpdateSiteListAction(QObject* parent) : BasicAction(parent)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateSiteListAction::~UpdateSiteListAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateSiteListAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_SITE_LIST;
}

void UpdateSiteListAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<SiteListDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be SiteListDTO");
        }

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update site list - ") + tr(e.what()));
    }
}

} // namespace gcs
