#pragma once

#include <common/IDTO.h>

#include "RobotDataBaseDTO.h"

namespace gcs {

class RobotPositionVelocityNedDTO : public RobotDataBaseDTO
{
  public:
    virtual ~RobotPositionVelocityNedDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ROBOT_POSITION_VELOCITY_NED; }

    double& getPositionX() Q_DECL_NOEXCEPT { return _position_x; }

    const double& getPositionX() const Q_DECL_NOEXCEPT { return _position_x; }

    double& getPositionY() Q_DECL_NOEXCEPT { return _position_y; }

    const double& getPositionY() const Q_DECL_NOEXCEPT { return _position_y; }

    double& getPositionZ() Q_DECL_NOEXCEPT { return _position_z; }

    const double& getPositionZ() const Q_DECL_NOEXCEPT { return _position_z; }

    double& getNorthSpeed() Q_DECL_NOEXCEPT { return _north_m_s; }

    const double& getNorthSpeed() const Q_DECL_NOEXCEPT { return _north_m_s; }

    double& getEastSpeed() Q_DECL_NOEXCEPT { return _east_m_s; }

    const double& getEastSpeed() const Q_DECL_NOEXCEPT { return _east_m_s; }

    double& getDownSpeed() Q_DECL_NOEXCEPT { return _down_m_s; }

    const double& getDownSpeed() const Q_DECL_NOEXCEPT { return _down_m_s; }

    /// Operators
    RobotPositionVelocityNedDTO& operator=(const RobotPositionVelocityNedDTO& o) Q_DECL_NOEXCEPT
    {
        _position_x = o._position_x;
        _position_y = o._position_y;
        _position_z = o._position_z;
        _north_m_s  = o._north_m_s;
        _east_m_s   = o._east_m_s;
        _down_m_s   = o._down_m_s;

        return *this;
    }

    bool operator==(const RobotPositionVelocityNedDTO& o) const Q_DECL_NOEXCEPT
    {
        return _position_x == o._position_x && _position_y == o._position_y && _position_z == o._position_z
            && _north_m_s == o._north_m_s && _east_m_s == o._east_m_s && _down_m_s == o._down_m_s;
    }

    bool operator!=(const RobotPositionVelocityNedDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    double _position_x{};
    double _position_y{};
    double _position_z{};
    double _north_m_s{}; /**< Velocity along north direction in metres per second */
    double _east_m_s{};  /**< Velocity along east direction in metres per second */
    double _down_m_s{};  /**< Velocity along down direction in metres per second */
};

} // namespace gcs
