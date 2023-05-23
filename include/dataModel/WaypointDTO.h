#pragma once

#include <dataModel/ActionParametersDTO.h>
#include <dataModel/BasicWaypointDTO.h>
#include <dataModel/Position3dDTO.h>
#include <dataModel/QuaternionDTO.h>
namespace gcs {

class WaypointDTO : public BasicWaypointDTO
{
  public:
    virtual ~WaypointDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::WAYPOINT; }

    Position3dDTO& getPosition() Q_DECL_NOEXCEPT { return _position; }

    const Position3dDTO& getPosition() const Q_DECL_NOEXCEPT { return _position; }

    QuaternionDTO& getOrientation() Q_DECL_NOEXCEPT { return _quat; }

    const QuaternionDTO& getOrientation() const Q_DECL_NOEXCEPT { return _quat; }

    bool& getAutoContinue() Q_DECL_NOEXCEPT { return _autocontinue; }

    const bool& getAutoContinue() const Q_DECL_NOEXCEPT { return _autocontinue; }

    QString& getTaskUUID() Q_DECL_NOEXCEPT { return _task_uuid; }

    const QString& getTaskUUID() const Q_DECL_NOEXCEPT { return _task_uuid; }

    ActionParametersDTO& getActionParameters() Q_DECL_NOEXCEPT { return _action_parameters; }

    const ActionParametersDTO& getActionParameters() const Q_DECL_NOEXCEPT { return _action_parameters; }
    /// Operators
    WaypointDTO&               operator=(const WaypointDTO& o) Q_DECL_NOEXCEPT
    {
        BasicWaypointDTO::operator=(o);

        _position          = o._position;
        _quat              = o._quat;
        _autocontinue      = o._autocontinue;
        _task_uuid         = o._task_uuid;
        _action_parameters = o._action_parameters;

        return *this;
    }

    bool operator==(const WaypointDTO& o) const Q_DECL_NOEXCEPT
    {
        return BasicWaypointDTO::operator==(o) && _autocontinue == o._autocontinue && _position == o._position
            && _quat == o._quat && _task_uuid == o._task_uuid && _action_parameters == o._action_parameters;
    }

    bool operator!=(const WaypointDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    Position3dDTO       _position;
    QuaternionDTO       _quat;
    bool                _autocontinue{true};
    QString             _task_uuid;
    ActionParametersDTO _action_parameters;
};

} // namespace gcs
