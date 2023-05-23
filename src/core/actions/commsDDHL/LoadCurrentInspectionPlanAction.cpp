#include "LoadCurrentInspectionPlanAction.h"

#include <QsLog/QsLog.h>
#include <config.h>
#include <dataModel/DataModel.h>

namespace gcs {

LoadCurrentInspectionPlanAction::LoadCurrentInspectionPlanAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

LoadCurrentInspectionPlanAction::~LoadCurrentInspectionPlanAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType LoadCurrentInspectionPlanAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::LOAD_CURRENT_INSPECTION_PLAN;
}

void LoadCurrentInspectionPlanAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        auto inspPlanDTO = createSharedDTO<InspectionPlanDTO>();
        *inspPlanDTO     = _dataModel.getInspectionPlanDTO();
        Q_EMIT sendDTO(inspPlanDTO);

    } catch (std::runtime_error& e) {
        QLOG_WARN() << __PRETTY_FUNCTION__ << " - Error:" << e.what();

        sendWarningMsg(tr("Load Current Inspection Plan error - ") + tr(e.what()));
    }
}

} // namespace gcs
