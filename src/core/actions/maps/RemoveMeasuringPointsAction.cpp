#include "RemoveMeasuringPointsAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

RemoveMeasuringPointsAction::RemoveMeasuringPointsAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << "RemoveMeasuringPointsAction::RemoveMeasuringPointsAction()";
}

RemoveMeasuringPointsAction::~RemoveMeasuringPointsAction()
{
    QLOG_TRACE() << "RemoveMeasuringPointsAction::~RemoveMeasuringPointsAction()";
}

ActionType RemoveMeasuringPointsAction::getActionType()
{
    QLOG_TRACE() << "RemoveMeasuringPointsAction::getActionType()";

    return ActionType::REMOVE_MEASURING_POINTS;
}

void RemoveMeasuringPointsAction::execute(QSharedPointer<IDTO>)
{
    QLOG_TRACE() << "RemoveMeasuringPointsAction::execute()";

    _dataModel.getMeasureDTO().getPointList() = QList<Position3dDTO>();

    auto measureDTO = createSharedDTO<MeasureDTO>();
    *measureDTO     = _dataModel.getMeasureDTO();
    Q_EMIT sendDTO(measureDTO);
}

} // namespace gcs
