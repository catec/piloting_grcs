
#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

#include <QtGlobal>

namespace gcs {

class RobotDataBaseDTO : public IDTO
{
  public:
    virtual ~RobotDataBaseDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ROBOT_DATA_BASE; }

    quint8& systemId() Q_DECL_NOEXCEPT { return _system_id; }

    const quint8& systemId() const Q_DECL_NOEXCEPT { return _system_id; }

    quint8& componentId() Q_DECL_NOEXCEPT { return _component_id; }

    const quint8& componentId() const Q_DECL_NOEXCEPT { return _component_id; }

    /// Operators
    RobotDataBaseDTO& operator=(const RobotDataBaseDTO& o) Q_DECL_NOEXCEPT
    {
        _system_id    = o._system_id;
        _component_id = o._component_id;

        return *this;
    }

    bool operator==(const RobotDataBaseDTO& o) const Q_DECL_NOEXCEPT
    {
        return _system_id == o._system_id && _component_id == o._component_id;
    }

    bool operator!=(const RobotDataBaseDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint8 _system_id{0};    // ID of message sender system/aircraft
    quint8 _component_id{0}; // ID of message sender component
};

} // namespace gcs
