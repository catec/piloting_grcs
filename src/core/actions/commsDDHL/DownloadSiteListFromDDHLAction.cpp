
#include "DownloadSiteListFromDDHLAction.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <dataModel/DataModel.h>
#include <dataModel/dtos/SiteTypeDTO.h>

#include <QMetaObject>

namespace gcs {

DownloadSiteListFromDDHLAction::DownloadSiteListFromDDHLAction(
        DDHLComms& comms, DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

DownloadSiteListFromDDHLAction::~DownloadSiteListFromDDHLAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType DownloadSiteListFromDDHLAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::DOWNLOAD_SITE_LIST_FROM_DDHL;
}

void DownloadSiteListFromDDHLAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        auto parsedDTO = dto.dynamicCast<SiteTypeDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be SiteTypeDTO");
        }

        QMetaObject::invokeMethod(
                _commsObj,
                "downloadSiteListFromDDHL",
                Qt::BlockingQueuedConnection,
                Q_ARG(const CommsLinkDDHLDTO&, _dataModel.getCommsLinkDDHLDTO()),
                Q_ARG(const SiteType&, parsedDTO->getSiteType()));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Download inspection plans sites from DDHL error - ") + tr(e.what()));
    }
}

} // namespace gcs
