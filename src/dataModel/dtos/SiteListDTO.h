#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

#include "SiteDTO.h"

namespace gcs {

class SiteListDTO : public IDTO
{
  public:
    virtual ~SiteListDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::SITE_LIST; }

    QList<SiteDTO>& getSiteList() Q_DECL_NOEXCEPT { return _siteList; }

    const QList<SiteDTO>& getSiteList() const Q_DECL_NOEXCEPT { return _siteList; }

    /// Operators
    SiteListDTO& operator=(const SiteListDTO& o) Q_DECL_NOEXCEPT
    {
        _siteList = o._siteList;

        return *this;
    }

    bool operator==(const SiteListDTO& o) const Q_DECL_NOEXCEPT { return _siteList == o._siteList; }

    bool operator!=(const SiteListDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<SiteDTO> _siteList;
};

} // namespace gcs
