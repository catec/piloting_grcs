#pragma once

#include <common/IDTO.h>

#include <QtGlobal>

namespace gcs {

class BasicRouteDTO : public IDTO
{
  public:
    virtual ~BasicRouteDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::BASIC_ROUTE; }

    quint16& getId() Q_DECL_NOEXCEPT { return _id; }

    const quint16& getId() const Q_DECL_NOEXCEPT { return _id; }

    /// Operators
    BasicRouteDTO& operator=(const BasicRouteDTO& o) Q_DECL_NOEXCEPT
    {
        _id = o._id;

        return *this;
    }

    bool operator==(const BasicRouteDTO& o) const Q_DECL_NOEXCEPT { return _id == o._id; }

    bool operator!=(const BasicRouteDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16 _id{0};
};

} // namespace gcs
