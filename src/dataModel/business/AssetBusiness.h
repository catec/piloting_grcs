#pragma once

#include <common/Types.h>

#include <QString>

#include "gcs_dataModel_export.h"

namespace gcs {

class AssetDTO;

class PILOTING_GRCS_DATAMODEL_EXPORT AssetBusiness
{
  public:
    explicit AssetBusiness(AssetDTO& assetDTO);
    virtual ~AssetBusiness();

    bool     checkIfNeedDownloadAsset();
    AssetDTO convertInformationJsonToAssetDTO();
    void     saveAssetInFolder();

    QString getAssetFileName() const;
    quint64 getAssetFileSize() const;

  private:
    void      convertAssetInfoToJson(const QString& assetInfoPath);
    QDateTime obtainLastUpdateTimeFromFile(const QString& assetInfoPath);
    QString   obtainAssetsFolderPath();

    AssetDTO& _assetDTO;
};

} // namespace gcs
