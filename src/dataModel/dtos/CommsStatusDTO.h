#pragma once

#include <common/IDTO.h>

namespace gcs {

class CommsStatusDTO : public IDTO
{
  public:
    virtual ~CommsStatusDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::COMMS_STATUS; }

    bool& getIsConnected() Q_DECL_NOEXCEPT { return _isConnected; }

    const bool& getIsConnected() const Q_DECL_NOEXCEPT { return _isConnected; }

    /// Operators
    CommsStatusDTO& operator=(const CommsStatusDTO& o) Q_DECL_NOEXCEPT
    {
        _isConnected = o._isConnected;

        return *this;
    }

    bool operator==(const CommsStatusDTO& o) const Q_DECL_NOEXCEPT { return _isConnected == o._isConnected; }

    bool operator!=(const CommsStatusDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    bool _isConnected{false};
};

} // namespace gcs
