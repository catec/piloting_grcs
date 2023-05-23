#pragma once

#include <common/Types.h>

#include "CommsLinkDTO.h"

namespace gcs {
class CommsLinkDDHLDTO : public CommsLinkDTO
{
  public:
    virtual ~CommsLinkDDHLDTO() {}

    virtual DTOType getDTOType() override { return DTOType::DDHL_COMMS_LINK; }

    DDHLEndpointMode&       getEndpointMode() Q_DECL_NOEXCEPT { return _endpointMode; }
    const DDHLEndpointMode& getEndpointMode() const Q_DECL_NOEXCEPT { return _endpointMode; }

    QString&       getTargetUrl() Q_DECL_NOEXCEPT { return _targetUrl; }
    const QString& getTargetUrl() const Q_DECL_NOEXCEPT { return _targetUrl; }

    QString&       getTargetIP() Q_DECL_NOEXCEPT { return _targetIp; }
    const QString& getTargetIP() const Q_DECL_NOEXCEPT { return _targetIp; }

    quint16&       getTargetPort() Q_DECL_NOEXCEPT { return _targetPort; }
    const quint16& getTargetPort() const Q_DECL_NOEXCEPT { return _targetPort; }

    QString&       getUsername() Q_DECL_NOEXCEPT { return _username; }
    const QString& getUsername() const Q_DECL_NOEXCEPT { return _username; }

    QString&       getPassword() Q_DECL_NOEXCEPT { return _password; }
    const QString& getPassword() const Q_DECL_NOEXCEPT { return _password; }

    /// Operators
    CommsLinkDDHLDTO& operator=(const CommsLinkDDHLDTO& o) Q_DECL_NOEXCEPT
    {
        _endpointMode = o._endpointMode;
        _targetUrl    = o._targetUrl;
        _targetIp     = o._targetIp;
        _targetPort   = o._targetPort;
        _username     = o._username;
        _password     = o._password;

        return *this;
    }

    bool operator==(const CommsLinkDDHLDTO& o) const Q_DECL_NOEXCEPT
    {
        return _endpointMode == o._endpointMode && _targetUrl == o._targetUrl && _targetIp == o._targetIp
            && _targetPort == o._targetPort && _username == o._username && _password == o._password;
    }

    bool operator!=(const CommsLinkDDHLDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString          _targetUrl;
    QString          _targetIp;
    quint16          _targetPort;
    QString          _username{""};
    QString          _password{""};
    DDHLEndpointMode _endpointMode;
};

} // namespace gcs
