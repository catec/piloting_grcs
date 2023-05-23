#pragma once

#include <common/IDTO.h>

#include "RobotDataBaseDTO.h"

namespace gcs {

class RobotAttitudeAngVelocityDTO : public RobotDataBaseDTO
{
  public:
    virtual ~RobotAttitudeAngVelocityDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ROBOT_ATTITUDE_ANGVEL; }

    double& getQuaternionW() Q_DECL_NOEXCEPT { return _qw; }

    const double& getQuaternionW() const Q_DECL_NOEXCEPT { return _qw; }

    double& getQuaternionX() Q_DECL_NOEXCEPT { return _qx; }

    const double& getQuaternionX() const Q_DECL_NOEXCEPT { return _qx; }

    double& getQuaternionY() Q_DECL_NOEXCEPT { return _qy; }

    const double& getQuaternionY() const Q_DECL_NOEXCEPT { return _qy; }

    double& getQuaternionZ() Q_DECL_NOEXCEPT { return _qz; }

    const double& getQuaternionZ() const Q_DECL_NOEXCEPT { return _qz; }

    double& getRollAngularVelocity() Q_DECL_NOEXCEPT { return _roll_rad_s; }

    const double& getRollAngularVelocity() const Q_DECL_NOEXCEPT { return _roll_rad_s; }

    double& getPitchAngularVelocity() Q_DECL_NOEXCEPT { return _pitch_rad_s; }

    const double& getPitchAngularVelocity() const Q_DECL_NOEXCEPT { return _pitch_rad_s; }

    double& getYawAngularVelocity() Q_DECL_NOEXCEPT { return _yaw_rad_s; }

    const double& getYawAngularVelocity() const Q_DECL_NOEXCEPT { return _yaw_rad_s; }

    /// Operators
    RobotAttitudeAngVelocityDTO& operator=(const RobotAttitudeAngVelocityDTO& o) Q_DECL_NOEXCEPT
    {
        _roll_rad_s  = o._roll_rad_s;
        _pitch_rad_s = o._pitch_rad_s;
        _yaw_rad_s   = o._yaw_rad_s;
        _qw          = o._qw;
        _qx          = o._qx;
        _qy          = o._qy;
        _qz          = o._qz;

        return *this;
    }

    bool operator==(const RobotAttitudeAngVelocityDTO& o) const Q_DECL_NOEXCEPT
    {
        return _roll_rad_s == o._roll_rad_s && _pitch_rad_s == o._pitch_rad_s && _yaw_rad_s == o._yaw_rad_s
            && _qw == o._qw && _qx == o._qx && _qy == o._qy && _qz == o._qz;
    }

    bool operator!=(const RobotAttitudeAngVelocityDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    double _roll_rad_s{0};
    double _pitch_rad_s{0};
    double _yaw_rad_s{0};

    double _qw{0};
    double _qx{0};
    double _qy{0};
    double _qz{0};
};

} // namespace gcs
