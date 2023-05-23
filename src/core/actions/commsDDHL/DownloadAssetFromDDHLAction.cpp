
#include "DownloadAssetFromDDHLAction.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <dataModel/DataModel.h>

#include <QMetaObject>

namespace gcs {

DownloadAssetFromDDHLAction::DownloadAssetFromDDHLAction(DDHLComms& comms, DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _comms(comms), _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

DownloadAssetFromDDHLAction::~DownloadAssetFromDDHLAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType DownloadAssetFromDDHLAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::DOWNLOAD_ASSET_FROM_DDHL;
}

void DownloadAssetFromDDHLAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        if (!_commsObj) {
            throw std::runtime_error("Comms object is NULL");
        }

        auto parsedDTO = dto.dynamicCast<FileDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be FileDTO");
        }

        _dataModel.getInspectionPlanDTO().getAsset().getLoadedFile() = *parsedDTO;

        const auto url      = parsedDTO->getUrl();
        const auto assetDTO = _dataModel.getInspectionPlanDTO().getAsset();

        sendInfoMsg(QString("Asset file with name: \"%1\" and UUID: \"%2\" is going to be downloaded from DDHL")
                            .arg(assetDTO.getName(), assetDTO.getUUID()));

        QMetaObject::invokeMethod(
                _commsObj,
                "downloadAssetFromDDHL",
                Qt::BlockingQueuedConnection,
                Q_ARG(const CommsLinkDDHLDTO&, _dataModel.getCommsLinkDDHLDTO()),
                Q_ARG(const QString&, url));
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Download data from DDHL error - ") + tr(e.what()));
    }
}

} // namespace gcs
