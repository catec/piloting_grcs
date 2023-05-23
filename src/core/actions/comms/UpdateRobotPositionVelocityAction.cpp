
#include "UpdateRobotPositionVelocityAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>
#include <dataModel/RobotPositionVelocityNedDTO.h>

namespace gcs {

UpdateRobotPositionVelocityAction::UpdateRobotPositionVelocityAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateRobotPositionVelocityAction::~UpdateRobotPositionVelocityAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateRobotPositionVelocityAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_POSITION_VELOCITY_NED;
}

void UpdateRobotPositionVelocityAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateRobotPositionVelocityAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<RobotPositionVelocityNedDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be RobotPositionVelocityNedDTO");
        }

        _dataModel.getRobotPositionVelocityDTO() = *parsedDTO;

        auto posVelDTO = createSharedDTO<RobotPositionVelocityNedDTO>();
        *posVelDTO     = _dataModel.getRobotPositionVelocityDTO();
        Q_EMIT sendDTO(posVelDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateRobotPositionVelocityAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update Robot position and velocity error - ") + tr(e.what()));
    }
}

} // namespace gcs
