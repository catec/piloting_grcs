#pragma once

#include <common/IDTO.h>
#include <common/Types.h>

#include <QDebug>

#include "BasicInfoDTO.h"
#include "SiteTypeDTO.h"

namespace gcs {

class SiteDTO : public SiteTypeDTO
{
  public:
    virtual ~SiteDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::SITE; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    QString& getSubtype() Q_DECL_NOEXCEPT { return _subtype; }

    const QString& getSubtype() const Q_DECL_NOEXCEPT { return _subtype; }

    BasicInfoDTO& getBasicInspectionPlan() Q_DECL_NOEXCEPT { return _basicInspectionPlan; }

    const BasicInfoDTO& getBasicInspectionPlan() const Q_DECL_NOEXCEPT { return _basicInspectionPlan; }

    BasicInfoDTO& getBasicAsset() Q_DECL_NOEXCEPT { return _basicAsset; }

    const BasicInfoDTO& getBasicAsset() const Q_DECL_NOEXCEPT { return _basicAsset; }

    /// Operators
    SiteDTO& operator=(const SiteDTO& o) Q_DECL_NOEXCEPT
    {
        SiteTypeDTO::operator=(o);
        _name                = o._name;
        _subtype             = o._subtype;
        _basicInspectionPlan = o._basicInspectionPlan;
        _basicAsset          = o._basicAsset;

        return *this;
    }

    bool operator==(const SiteDTO& o) const Q_DECL_NOEXCEPT
    {
        return SiteTypeDTO::operator==(o) && _name == _name && _subtype == o._subtype
            && _basicInspectionPlan == o._basicInspectionPlan && _basicAsset == o._basicAsset;
    }

    bool operator!=(const SiteDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

    friend QDebug& operator<<(QDebug& dbg, const SiteDTO& asset)
    {
        dbg << "SiteDTO ->"
            << "\n"
            << "Name: " << asset.getName() << "\n"
            << "Subtype: " << asset.getSubtype() << "\n"
            << "BasicInspectionPlanDTO ->"
            << "\n"
            << "UUID: " << asset.getBasicInspectionPlan().getUUID() << "\n"
            << "Name: " << asset.getBasicInspectionPlan().getName() << "\n"
            << "Description: " << asset.getBasicInspectionPlan().getDescription() << "\n"
            << "CreationDateTime: " << asset.getBasicInspectionPlan().getCreationDateTime().toString(Qt::ISODateWithMs)
            << "\n"
            << "LastUpdateDateTime: "
            << asset.getBasicInspectionPlan().getLastUpdateDateTime().toString(Qt::ISODateWithMs) << "\n"
            << "BasicAssetDTO ->"
            << "\n"
            << "UUID: " << asset.getBasicAsset().getUUID() << "\n"
            << "Name: " << asset.getBasicAsset().getName() << "\n"
            << "Description: " << asset.getBasicAsset().getDescription() << "\n"
            << "CreationDateTime: " << asset.getBasicAsset().getCreationDateTime().toString(Qt::ISODateWithMs) << "\n"
            << "LastUpdateDateTime: " << asset.getBasicAsset().getLastUpdateDateTime().toString(Qt::ISODateWithMs)
            << "\n";
        return dbg;
    }

  private:
    QString      _name;
    QString      _subtype;
    BasicInfoDTO _basicInspectionPlan;
    BasicInfoDTO _basicAsset;
};

} // namespace gcs
