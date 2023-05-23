#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

#include <QString>

namespace gcs {

class CommsProgressDTO : public IDTO
{
  public:
    virtual ~CommsProgressDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::COMMS_PROGRESS; }

    CommsProgressType& getCommsProgressType() Q_DECL_NOEXCEPT { return _commsProgressType; }

    const CommsProgressType& getCommsProgressType() const Q_DECL_NOEXCEPT { return _commsProgressType; }

    QString& getCommsProgressInfo() Q_DECL_NOEXCEPT { return _commsProgressInfo; }

    const QString& getCommsProgressInfo() const Q_DECL_NOEXCEPT { return _commsProgressInfo; }

    /// Operators
    CommsProgressDTO& operator=(const CommsProgressDTO& o) Q_DECL_NOEXCEPT
    {
        _commsProgressType = o._commsProgressType;
        _commsProgressInfo = o._commsProgressInfo;

        return *this;
    }

    bool operator==(const CommsProgressDTO& o) const Q_DECL_NOEXCEPT
    {
        return _commsProgressType == o._commsProgressType && _commsProgressInfo == o._commsProgressInfo;
    }

    bool operator!=(const CommsProgressDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    CommsProgressType _commsProgressType{CommsProgressType::START};
    QString           _commsProgressInfo;
};

} // namespace gcs
