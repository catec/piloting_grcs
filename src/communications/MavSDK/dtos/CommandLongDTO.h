#pragma once

#include "CommandBasicDTO.h"
#include "ParamDTO.h"

namespace gcs {

class CommandLongDTO : public CommandBasicDTO
{
  public:
    virtual ~CommandLongDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::COMMAND_LONG; }

    QList<ParamDTO>& getParamList() Q_DECL_NOEXCEPT { return _paramList; }

    const QList<ParamDTO>& getParamList() const Q_DECL_NOEXCEPT { return _paramList; }

    /// Operators
    CommandLongDTO& operator=(const CommandLongDTO& o) Q_DECL_NOEXCEPT
    {
        CommandBasicDTO::operator=(o);
        _paramList = o._paramList;

        return *this;
    }

    bool operator==(const CommandLongDTO& o) const Q_DECL_NOEXCEPT
    {
        return CommandBasicDTO::operator==(o) && _paramList == o._paramList;
    }

    bool operator!=(const CommandLongDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<ParamDTO> _paramList;
};

} // namespace gcs
