#pragma once

#include <common/Types.h>

#include "JsonCheckKeys.h"
#include "gcs_dataModel_export.h"

namespace gcs {

class InspectionPlanDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT InspectionPlanBusiness
{
  public:
    explicit InspectionPlanBusiness(InspectionPlanDTO& inspectionPlanDTO);
    virtual ~InspectionPlanBusiness();

    void saveInspectionPlan();
    void loadInspectionPlanFromLocalPath(const QString& filePath);

  private:
    void                       saveInspectionPlanAsJson(const QString& inspectionPlanPath);
    InspectionTaskLocationType toInspectionTaskLocationType(const QString& key);

  private:
    InspectionPlanDTO& _inspectionPlanDTO;
    JsonCheckKeys      _jsonCheckKeys;
};

} // namespace gcs
