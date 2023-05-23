#include "UpdateAssetPathAction.h"

#include <QsLog/QsLog.h>
#include <config.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/DataModel.h>

namespace gcs {

UpdateAssetPathAction::UpdateAssetPathAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _inspectionPlanBusiness(dataModel.getInspectionPlanDTO()),
        _assetBusiness(dataModel.getInspectionPlanDTO().getAsset())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateAssetPathAction::~UpdateAssetPathAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateAssetPathAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_ASSET_PATH;
}

void UpdateAssetPathAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        _dataModel.getInspectionPlanDTO().getAsset().getLoadedFile().getPath() = parsedDTO->getString();
        _assetBusiness.saveAssetInFolder();
        _inspectionPlanBusiness.saveInspectionPlan();

        auto assetDTO = createSharedDTO<AssetDTO>();
        *assetDTO     = _dataModel.getInspectionPlanDTO().getAsset();
        Q_EMIT sendDTO(assetDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update Asset Path error - ") + tr(e.what()));
    }
}

} // namespace gcs
