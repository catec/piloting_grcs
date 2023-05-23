#pragma once

#include <common/IDTO.h>

namespace gcs {

class QuaternionDTO : public IDTO
{
  public:
    virtual ~QuaternionDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::QUATERNION; }

    float& getW() Q_DECL_NOEXCEPT { return _w; }

    const float& getW() const Q_DECL_NOEXCEPT { return _w; }

    float& getX() Q_DECL_NOEXCEPT { return _x; }

    const float& getX() const Q_DECL_NOEXCEPT { return _x; }

    float& getY() Q_DECL_NOEXCEPT { return _y; }

    const float& getY() const Q_DECL_NOEXCEPT { return _y; }

    float& getZ() Q_DECL_NOEXCEPT { return _z; }

    const float& getZ() const Q_DECL_NOEXCEPT { return _z; }

    /// Operators
    QuaternionDTO& operator=(const QuaternionDTO& o) Q_DECL_NOEXCEPT
    {
        _w = o._w;
        _x = o._x;
        _y = o._y;
        _z = o._z;

        return *this;
    }

    bool operator==(const QuaternionDTO& o) const Q_DECL_NOEXCEPT
    {
        return _w == o._w && _x == o._x && _y == o._y && _z == o._z;
    }

    bool operator!=(const QuaternionDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

    // friend QDebug& operator<<(QDebug& dbg, const QuaternionDTO& quat)
    // {
    //    dbg << "w:" << quat.w() << "x:" << quat.x()
    //        << "y:" << quat.y() << "z:" << quat.z();

    //    return dbg;
    // }

  private:
    float _w{1.0};
    float _x{0.0};
    float _y{0.0};
    float _z{0.0};
};

} // namespace gcs
