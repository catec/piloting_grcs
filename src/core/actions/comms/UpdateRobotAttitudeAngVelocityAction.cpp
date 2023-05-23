
#include "UpdateRobotAttitudeAngVelocityAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>
#include <dataModel/RobotAttitudeAngVelocityDTO.h>

namespace gcs {

UpdateRobotAttitudeAngVelocityAction::UpdateRobotAttitudeAngVelocityAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

UpdateRobotAttitudeAngVelocityAction::~UpdateRobotAttitudeAngVelocityAction()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

ActionType UpdateRobotAttitudeAngVelocityAction::getActionType()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return ActionType::UPDATE_ATTITUDE_ANGVELOCITY;
}

void UpdateRobotAttitudeAngVelocityAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateRobotAttitudeAngVelocityAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<RobotAttitudeAngVelocityDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be RobotAttitudeAngVelocityDTO");
        }

        _dataModel.getRobotAttitudeAngVelocityDTO() = *parsedDTO;

        auto attitudeDTO = createSharedDTO<RobotAttitudeAngVelocityDTO>();
        *attitudeDTO     = _dataModel.getRobotAttitudeAngVelocityDTO();
        Q_EMIT sendDTO(attitudeDTO);
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateRobotAttitudeAngVelocityAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update Robot Attitude error - ") + tr(e.what()));
    }
}

} // namespace gcs
