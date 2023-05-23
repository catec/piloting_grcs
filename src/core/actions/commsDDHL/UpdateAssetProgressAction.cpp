

#include "UpdateAssetProgressAction.h"

#include <QsLog/QsLog.h>
#include <config.h>
#include <dataModel/DataModel.h>
#include <dataModel/dtos/AssetProgressDTO.h>

namespace gcs {

UpdateAssetProgressAction::UpdateAssetProgressAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel), _assetBusiness(dataModel.getInspectionPlanDTO().getAsset())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateAssetProgressAction::~UpdateAssetProgressAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateAssetProgressAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_ASSET_PROGRESS;
}

void UpdateAssetProgressAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<AssetProgressDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be AssetDTO");
        }

        ///\note By default total size is set by downloaded File size in JSon
        auto assetFileSize = _assetBusiness.getAssetFileSize();
        if (parsedDTO->getTotalBytes() != -1) {
            assetFileSize = parsedDTO->getTotalBytes();
        }
        parsedDTO->getTotalBytes()    = assetFileSize;
        parsedDTO->getAssetFileName() = _assetBusiness.getAssetFileName();

        Q_EMIT sendDTO(parsedDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update Asset Progress error - ") + tr(e.what()));
    }
}

} // namespace gcs
