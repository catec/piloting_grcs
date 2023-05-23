#pragma once

#include <QTextStream>

#include "CommsLinkDTO.h"

namespace gcs {
class CommsLinkMavsdkDTO : public CommsLinkDTO
{
  public:
    virtual ~CommsLinkMavsdkDTO() {}

    virtual DTOType getDTOType() override { return DTOType::MAVSDK_COMMS_LINK; }

    quint16&       getHeartBeatInterval() Q_DECL_NOEXCEPT { return _heartbeatInterval; }
    const quint16& getHeartBeatInterval() const Q_DECL_NOEXCEPT { return _heartbeatInterval; }

    quint16&       getLocalSystemId() Q_DECL_NOEXCEPT { return _localSystemId; }
    const quint16& getLocalSystemId() const Q_DECL_NOEXCEPT { return _localSystemId; }

    quint16&       getLocalComponentId() Q_DECL_NOEXCEPT { return _localComponentId; }
    const quint16& getLocalComponentId() const Q_DECL_NOEXCEPT { return _localComponentId; }

    QString&       getLocalIP() Q_DECL_NOEXCEPT { return _localIp; }
    const QString& getLocalIP() const Q_DECL_NOEXCEPT { return _localIp; }

    quint16&       getLocalPort() Q_DECL_NOEXCEPT { return _localPort; }
    const quint16& getLocalPort() const Q_DECL_NOEXCEPT { return _localPort; }

    quint16&       getTargetSystemId() Q_DECL_NOEXCEPT { return _targetSystemId; }
    const quint16& getTargetSystemId() const Q_DECL_NOEXCEPT { return _targetSystemId; }

    QString&       getTargetIP() Q_DECL_NOEXCEPT { return _targetIp; }
    const QString& getTargetIP() const Q_DECL_NOEXCEPT { return _targetIp; }

    quint16&       getTargetPort() Q_DECL_NOEXCEPT { return _targetPort; }
    const quint16& getTargetPort() const Q_DECL_NOEXCEPT { return _targetPort; }

    /// Operators
    CommsLinkMavsdkDTO& operator=(const CommsLinkMavsdkDTO& o) Q_DECL_NOEXCEPT
    {
        _heartbeatInterval = o._heartbeatInterval;
        _localSystemId     = o._localSystemId;
        _localComponentId  = o._localComponentId;
        _localIp           = o._localIp;
        _localPort         = o._localPort;
        _targetIp          = o._targetIp;
        _targetPort        = o._targetPort;
        _targetSystemId    = o._targetSystemId;

        return *this;
    }

    bool operator==(const CommsLinkMavsdkDTO& o) const Q_DECL_NOEXCEPT
    {
        return _heartbeatInterval == o._heartbeatInterval && _localSystemId == o._localSystemId
            && _localComponentId == o._localComponentId && _localIp == o._localIp && _localPort == o._localPort
            && _targetIp == o._targetIp && _targetPort == o._targetPort && _targetSystemId == o._targetSystemId;
    }

    bool operator!=(const CommsLinkMavsdkDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    /// \note. Unused field. MAVSDK has fixed heartbeat (1 sec)
    quint16 _heartbeatInterval;

    quint16 _localSystemId;
    quint16 _localComponentId;

    QString _localIp;
    quint16 _localPort;
    QString _targetIp;
    quint16 _targetPort;

    quint16 _targetSystemId;
};

} // namespace gcs