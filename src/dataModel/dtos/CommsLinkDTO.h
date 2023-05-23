#pragma once

#include <common/IDTO.h>

namespace gcs {

class CommsLinkDTO : public IDTO
{
  public:
    virtual ~CommsLinkDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::COMMS_LINK; }
};

} // namespace gcs
