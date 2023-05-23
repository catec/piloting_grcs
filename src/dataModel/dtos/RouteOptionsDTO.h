#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class RouteOptionsDTO : public IDTO
{
  public:
    virtual ~RouteOptionsDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ROUTE_OPTIONS; }

    RouteOptions& getFlag() Q_DECL_NOEXCEPT { return _flag; }

    const RouteOptions& getFlag() const Q_DECL_NOEXCEPT { return _flag; }

    /// Operators
    RouteOptionsDTO& operator=(const RouteOptionsDTO& o) Q_DECL_NOEXCEPT
    {
        _flag = o._flag;

        return *this;
    }

    bool operator==(const RouteOptionsDTO& o) const Q_DECL_NOEXCEPT { return _flag == o._flag; }

    bool operator!=(const RouteOptionsDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    RouteOptions _flag{NoShow};
};

} // namespace gcs
