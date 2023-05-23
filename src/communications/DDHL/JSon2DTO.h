
#pragma once

#include <dataModel/dtos/AssetDTO.h>
#include <dataModel/dtos/InspectionPlanDTO.h>
#include <dataModel/dtos/SiteListDTO.h>

#include <QObject>

#include "JSonKeys.h"
namespace gcs {

class JSon2DTO : public QObject
{
    Q_OBJECT

  public:
    explicit JSon2DTO(QObject* parent = nullptr);
    ~JSon2DTO();

    InspectionPlanDTO        convertInspectionPlanJsonToDTO(const QJsonObject&);
    QList<InspectionTaskDTO> convertInspectionTaskJsonArrayToList(const QJsonArray&);
    DownloadedFileListDTO    convertAssetFileJsonArrayToList(const QJsonArray&);
    AssetDTO                 convertAssetJsonToDTO(const QJsonObject&);
    SiteListDTO              convertSiteListJsonToDTO(const QJsonObject&);
    SiteDTO                  convertSiteJsonToDTO(const QJsonObject&);

  private:
    JSonKeys _jsonKeys;
};

} // namespace gcs
