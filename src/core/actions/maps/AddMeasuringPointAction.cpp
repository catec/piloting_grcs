#include "AddMeasuringPointAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

AddMeasuringPointAction::AddMeasuringPointAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << "AddMeasuringPointAction::AddMeasuringPointAction()";
}

AddMeasuringPointAction::~AddMeasuringPointAction()
{
    QLOG_TRACE() << "AddMeasuringPointAction::~AddMeasuringPointAction()";
}

ActionType AddMeasuringPointAction::getActionType()
{
    QLOG_TRACE() << "AddMeasuringPointAction::getActionType()";

    return ActionType::ADD_MEASURING_POINT;
}

void AddMeasuringPointAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "AddMeasuringPointAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<Position3dDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be Position3dDTO");
        }

        auto& pointList = _dataModel.getMeasureDTO().getPointList();
        if (pointList.size() >= 2) {
            QLOG_DEBUG() << "AddMeasuringPointAction::execute() - "
                            "Clear measure point list";

            pointList.clear();
        }
        pointList.append(*parsedDTO);

        auto measureDTO = createSharedDTO<MeasureDTO>();
        *measureDTO     = _dataModel.getMeasureDTO();
        Q_EMIT sendDTO(measureDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "AddMeasuringPointAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Add measuring point error - ") + tr(e.what()));
    }
}

} // namespace gcs
