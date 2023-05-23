#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class SiteTypeDTO : public IDTO
{
  public:
    virtual ~SiteTypeDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::SITE_TYPE; }

    SiteType& getSiteType() Q_DECL_NOEXCEPT { return _siteType; }

    const SiteType& getSiteType() const Q_DECL_NOEXCEPT { return _siteType; }

    /// Operators
    SiteTypeDTO& operator=(const SiteTypeDTO& o) Q_DECL_NOEXCEPT
    {
        _siteType = o._siteType;

        return *this;
    }

    bool operator==(const SiteTypeDTO& o) const Q_DECL_NOEXCEPT { return _siteType == o._siteType; }

    bool operator!=(const SiteTypeDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    SiteType _siteType{SiteType::UNKNOWN};
};

} // namespace gcs
