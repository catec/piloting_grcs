#pragma once

#include <common/IDTO.h>

namespace gcs {

class PoseDTO : public IDTO
{
  public:
    virtual ~PoseDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::POSE; }

    Position3dDTO& getPosition() Q_DECL_NOEXCEPT { return _position; }

    const Position3dDTO& getPosition() const Q_DECL_NOEXCEPT { return _position; }

    QuaternionDTO& getOrientation() Q_DECL_NOEXCEPT { return _quaternion; }

    const QuaternionDTO& getOrientation() const Q_DECL_NOEXCEPT { return _quaternion; }

    /// Operators
    PoseDTO& operator=(const PoseDTO& o) Q_DECL_NOEXCEPT
    {
        _position   = o._position;
        _quaternion = o._quaternion;

        return *this;
    }

    bool operator==(const PoseDTO& o) const Q_DECL_NOEXCEPT
    {
        return _position == o._position && _quaternion == o._quaternion;
    }

    bool operator!=(const PoseDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    Position3dDTO _position;
    QuaternionDTO _quaternion;
};

} // namespace gcs
