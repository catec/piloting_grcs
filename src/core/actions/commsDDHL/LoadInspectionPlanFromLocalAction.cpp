#include "LoadInspectionPlanFromLocalAction.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <config.h>
#include <core/dtos/BasicStringDTO.h>
#include <dataModel/DataModel.h>

#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>

namespace gcs {

LoadInspectionPlanFromLocalAction::LoadInspectionPlanFromLocalAction(
        DDHLComms& comms, DataModel& dataModel, QObject* parent) :
        BasicAction(parent),
        _dataModel(dataModel),
        _inspectionPlanBusiness(dataModel.getInspectionPlanDTO()),
        _comms(comms),
        _commsObj(dynamic_cast<QObject*>(&_comms))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

LoadInspectionPlanFromLocalAction::~LoadInspectionPlanFromLocalAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType LoadInspectionPlanFromLocalAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::LOAD_INSPECTION_PLAN_FROM_LOCAL;
}

void LoadInspectionPlanFromLocalAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto parsedDTO = dto.dynamicCast<BasicStringDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be BasicStringDTO");
        }

        _inspectionPlanBusiness.loadInspectionPlanFromLocalPath(parsedDTO->getString());

        auto assetDTO = createSharedDTO<AssetDTO>();
        *assetDTO     = _dataModel.getInspectionPlanDTO().getAsset();
        Q_EMIT sendDTO(assetDTO);

        sendInfoMsg("Successfully loaded Inspection Plan with UUID: " + _dataModel.getInspectionPlanDTO().getUUID());
    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Load Inspection Plan from local file error - ") + tr(e.what()));
    }
}

} // namespace gcs
