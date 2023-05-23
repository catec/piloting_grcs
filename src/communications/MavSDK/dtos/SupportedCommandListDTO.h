
#pragma once

#include <common/IDTO.h>

#include "CommandLongDTO.h"

namespace gcs {

class SupportedCommandListDTO : public IDTO
{
  public:
    virtual ~SupportedCommandListDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::SUPPORTED_COMMAND_LIST; }

    QList<CommandLongDTO>& getSupportedCommandList() Q_DECL_NOEXCEPT { return _supportedCommandList; }

    const QList<CommandLongDTO>& getSupportedCommandList() const Q_DECL_NOEXCEPT { return _supportedCommandList; }

    /// Operators
    SupportedCommandListDTO& operator=(const SupportedCommandListDTO& o) Q_DECL_NOEXCEPT
    {
        _supportedCommandList = o._supportedCommandList;
        return *this;
    }

    bool operator==(const SupportedCommandListDTO& o) const Q_DECL_NOEXCEPT
    {
        return _supportedCommandList == o._supportedCommandList;
    }

    bool operator!=(const SupportedCommandListDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<CommandLongDTO> _supportedCommandList;
};

} // namespace gcs
