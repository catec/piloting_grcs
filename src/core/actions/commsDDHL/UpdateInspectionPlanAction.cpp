#include "UpdateInspectionPlanAction.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <config.h>
#include <dataModel/DataModel.h>

#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>

namespace gcs {

UpdateInspectionPlanAction::UpdateInspectionPlanAction(DDHLComms& comms, DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _inspectionPlanBusiness(dataModel.getInspectionPlanDTO()),
        _assetBusiness(dataModel.getInspectionPlanDTO().getAsset()),
        _comms(comms),
        _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateInspectionPlanAction::~UpdateInspectionPlanAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateInspectionPlanAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_INSPECTION_PLAN;
}

void UpdateInspectionPlanAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<InspectionPlanDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be InspectionPlanDTO");
        }

        _dataModel.getInspectionPlanDTO() = *parsedDTO;

        if (_assetBusiness.checkIfNeedDownloadAsset()) {
            _dataModel.getInspectionPlanDTO().getAsset().getPath() = "";

            auto assetDTO = _dataModel.getInspectionPlanDTO().getAsset();

            QLOG_DEBUG() << __PRETTY_FUNCTION__
                         << "Downloaded file list associated to Inspection Plan:" << assetDTO.getDownloadedFileList();

            if (assetDTO.getDownloadedFileList().getDownloadedFileList().isEmpty()) {
                throw std::runtime_error("File list downloaded from the DDHL is empty");
            }

            auto downloadedAssetFile = createSharedDTO<DownloadedFileListDTO>();
            *downloadedAssetFile     = _dataModel.getInspectionPlanDTO().getAsset().getDownloadedFileList();
            Q_EMIT sendDTO(downloadedAssetFile);
        } else {
            auto savedAsset                              = _assetBusiness.convertInformationJsonToAssetDTO();
            _dataModel.getInspectionPlanDTO().getAsset() = savedAsset;
            sendInfoMsg(QString("Not necessary to download asset file with name: \"%1\" and UUID: \"%2\" from DDHL, "
                                "loading it from local storage")
                                .arg(savedAsset.getName(), savedAsset.getUUID()));
            auto assetDTO = createSharedDTO<AssetDTO>();
            *assetDTO     = _dataModel.getInspectionPlanDTO().getAsset();
            Q_EMIT sendDTO(assetDTO);
        }

        _inspectionPlanBusiness.saveInspectionPlan();

    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Update Inspection Plan error - ") + tr(e.what()));
    }
}
} // namespace gcs
