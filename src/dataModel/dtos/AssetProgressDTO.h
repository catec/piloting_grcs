
#pragma once

#include <common/IDTO.h>

namespace gcs {
class AssetProgressDTO : public IDTO
{
  public:
    virtual ~AssetProgressDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ASSET_PROGRESS; }

    qint64&       getBytesRead() Q_DECL_NOEXCEPT { return _bytesRead; }
    const qint64& getBytesRead() const Q_DECL_NOEXCEPT { return _bytesRead; }

    qint64&       getTotalBytes() Q_DECL_NOEXCEPT { return _totalBytes; }
    const qint64& getTotalBytes() const Q_DECL_NOEXCEPT { return _totalBytes; }

    QString&       getAssetFileName() Q_DECL_NOEXCEPT { return _assetFileName; }
    const QString& getAssetFileName() const Q_DECL_NOEXCEPT { return _assetFileName; }

    /// Operators
    AssetProgressDTO& operator=(const AssetProgressDTO& o) Q_DECL_NOEXCEPT
    {
        _bytesRead     = o._bytesRead;
        _totalBytes    = o._totalBytes;
        _assetFileName = o._assetFileName;

        return *this;
    }

    bool operator==(const AssetProgressDTO& o) const Q_DECL_NOEXCEPT
    {
        return _bytesRead == o._bytesRead && _totalBytes == o._totalBytes && _assetFileName == o._assetFileName;
    }

    bool operator!=(const AssetProgressDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    qint64  _bytesRead{0};
    qint64  _totalBytes{-1};
    QString _assetFileName{""};
};
} // namespace gcs