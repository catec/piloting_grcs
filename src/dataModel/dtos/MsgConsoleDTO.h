#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

#include <QString>

namespace gcs {

class MsgConsoleDTO : public IDTO
{
  public:
    virtual ~MsgConsoleDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MSG_CONSOLE; }

    MsgType& getMsgType() Q_DECL_NOEXCEPT { return _msgType; }

    const MsgType& getMsgType() const Q_DECL_NOEXCEPT { return _msgType; }

    QString& getMessage() Q_DECL_NOEXCEPT { return _message; }

    const QString& getMessage() const Q_DECL_NOEXCEPT { return _message; }

    /// Operators
    MsgConsoleDTO& operator=(const MsgConsoleDTO& o) Q_DECL_NOEXCEPT
    {
        _msgType = o._msgType;
        _message = o._message;

        return *this;
    }

    bool operator==(const MsgConsoleDTO& o) const Q_DECL_NOEXCEPT
    {
        return _msgType == o._msgType && _message == o._message;
    }

    bool operator!=(const MsgConsoleDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    MsgType _msgType{MsgType::INFO};
    QString _message;
};

} // namespace gcs
