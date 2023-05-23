#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class ApplicationStatusDTO : public IDTO
{
  public:
    virtual ~ApplicationStatusDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::APPLICATION_STATUS; }

    AppStatusType& getStatus() Q_DECL_NOEXCEPT { return _status; }

    const AppStatusType& getStatus() const Q_DECL_NOEXCEPT { return _status; }

    /// Operators
    ApplicationStatusDTO& operator=(const ApplicationStatusDTO& o) Q_DECL_NOEXCEPT
    {
        _status = o._status;

        return *this;
    }

    bool operator==(const ApplicationStatusDTO& o) const Q_DECL_NOEXCEPT { return _status == o._status; }

    bool operator!=(const ApplicationStatusDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    AppStatusType _status{AppStatusType::Panning};
};

} // namespace gcs
