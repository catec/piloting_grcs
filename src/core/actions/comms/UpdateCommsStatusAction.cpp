#include "UpdateCommsStatusAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/DataModel.h>

namespace gcs {

UpdateCommsStatusAction::UpdateCommsStatusAction(DataModel& dataModel, QObject* parent) :
        BasicAction(parent), _dataModel(dataModel)
{
    QLOG_TRACE() << "UpdateCommsStatusAction::UpdateCommsStatusAction()";
}

UpdateCommsStatusAction::~UpdateCommsStatusAction()
{
    QLOG_TRACE() << "UpdateCommsStatusAction::~UpdateCommsStatusAction()";
}

ActionType UpdateCommsStatusAction::getActionType()
{
    QLOG_TRACE() << "UpdateCommsStatusAction::getActionType()";

    return ActionType::UPDATE_COMMS_STATUS;
}

void UpdateCommsStatusAction::execute(QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "UpdateCommsStatusAction::execute()";

    try {
        auto parsedDTO = dto.dynamicCast<CommsStatusDTO>();
        if (!parsedDTO) {
            throw std::runtime_error("DTO type should be CommsStatusDTO");
        }

        _dataModel.getCommsStatusDTO() = *parsedDTO;

        auto commsStatusDTO = createSharedDTO<CommsStatusDTO>();
        *commsStatusDTO     = _dataModel.getCommsStatusDTO();
        Q_EMIT sendDTO(commsStatusDTO);

        if (!parsedDTO->getIsConnected()) {
            _dataModel.getRobotPositionVelocityDTO() = RobotPositionVelocityNedDTO();
            auto robotPositionPtr                    = createSharedDTO<RobotPositionVelocityNedDTO>();
            *robotPositionPtr                        = _dataModel.getRobotPositionVelocityDTO();
            Q_EMIT sendDTO(robotPositionPtr);

            _dataModel.getRobotAttitudeAngVelocityDTO() = RobotAttitudeAngVelocityDTO();
            auto robotAttitudePtr                       = createSharedDTO<RobotAttitudeAngVelocityDTO>();
            *robotAttitudePtr                           = _dataModel.getRobotAttitudeAngVelocityDTO();
            Q_EMIT sendDTO(robotAttitudePtr);
        }
    } catch (std::runtime_error& e) {
        QLOG_WARN() << "UpdateCommsStatusAction::execute() - "
                       "Error:"
                    << e.what();

        sendWarningMsg(tr("Update comms status error - ") + tr(e.what()));
    }
}

} // namespace gcs
