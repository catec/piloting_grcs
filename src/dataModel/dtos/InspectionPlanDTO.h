
#pragma once

#include <common/IDTO.h>

#include <QDateTime>
#include <QString>

#include "AssetDTO.h"
#include "BasicInfoDTO.h"
#include "InspectionTaskDTO.h"

namespace gcs {

class InspectionPlanDTO : public BasicInfoDTO
{
  public:
    virtual ~InspectionPlanDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::INSPECTION_PLAN; }

    AssetDTO& getAsset() Q_DECL_NOEXCEPT { return _asset; }

    const AssetDTO& getAsset() const Q_DECL_NOEXCEPT { return _asset; }

    QList<InspectionTaskDTO>& getInspectionTaskList() Q_DECL_NOEXCEPT { return _inspectionTaskList; }

    const QList<InspectionTaskDTO>& getInspectionTaskList() const Q_DECL_NOEXCEPT { return _inspectionTaskList; }

    /// Operators
    InspectionPlanDTO& operator=(const InspectionPlanDTO& o) Q_DECL_NOEXCEPT
    {
        BasicInfoDTO::operator=(o);
        _asset              = o._asset;
        _inspectionTaskList = o._inspectionTaskList;

        return *this;
    }

    bool operator==(const InspectionPlanDTO& o) const Q_DECL_NOEXCEPT
    {
        return BasicInfoDTO::operator==(o) && _asset == o._asset;
        _inspectionTaskList == o._inspectionTaskList;
    }

    bool operator!=(const InspectionPlanDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    AssetDTO                 _asset;
    QList<InspectionTaskDTO> _inspectionTaskList;
};

} // namespace gcs
