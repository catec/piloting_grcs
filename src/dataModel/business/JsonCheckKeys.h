#pragma once

#include "gcs_dataModel_export.h"

class QString;
class QJsonObject;

namespace gcs {

class PILOTING_GRCS_DATAMODEL_EXPORT JsonCheckKeys
{
  public:
    explicit JsonCheckKeys();
    virtual ~JsonCheckKeys();

    void checkExistWaypointListKeys(const QJsonObject&);
    void checkExistWaypointKeys(const QJsonObject&);
    void checkExistPositionKeys(const QJsonObject&);
    void checkExistOrientationKeys(const QJsonObject&);
    void checkExistActionParametersKeys(const QJsonObject&);
    void checkExistInspPlanKeys(const QJsonObject&);
    void checkExistInspPlanBasicInfoKeys(const QJsonObject&);
    void checkExistInspTaskKeys(const QJsonObject&);
    void checkExistAssetKeys(const QJsonObject&);
    void checkExistMissionTaskKeys(const QJsonObject&);

  private:
    void checkKeyExist(const QJsonObject&, const QString&);
};

} // namespace gcs
