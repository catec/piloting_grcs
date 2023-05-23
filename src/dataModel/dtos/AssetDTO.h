#pragma once

#include <QDateTime>
#include <QDebug>

#include "BasicInfoDTO.h"
#include "DownloadedFileListDTO.h"
#include "common/IDTO.h"

namespace gcs {

class AssetDTO : public BasicInfoDTO
{
  public:
    virtual ~AssetDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ASSET; }

    QString& getPath() Q_DECL_NOEXCEPT { return _path; }

    const QString& getPath() const Q_DECL_NOEXCEPT { return _path; }

    DownloadedFileListDTO&       getDownloadedFileList() Q_DECL_NOEXCEPT { return _downloadedFileList; }
    const DownloadedFileListDTO& getDownloadedFileList() const Q_DECL_NOEXCEPT { return _downloadedFileList; }

    FileDTO&       getLoadedFile() Q_DECL_NOEXCEPT { return _loadedFile; }
    const FileDTO& getLoadedFile() const Q_DECL_NOEXCEPT { return _loadedFile; }

    /// Operators
    AssetDTO& operator=(const AssetDTO& o) Q_DECL_NOEXCEPT
    {
        BasicInfoDTO::operator=(o);
        _path               = o._path;
        _downloadedFileList = o._downloadedFileList;
        _loadedFile         = o._loadedFile;
        return *this;
    }

    bool operator==(const AssetDTO& o) const Q_DECL_NOEXCEPT
    {
        return BasicInfoDTO::operator==(o) && _path == o._path && _downloadedFileList == o._downloadedFileList
            && _loadedFile == o._loadedFile;
    }

    bool operator!=(const AssetDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

    friend QDebug& operator<<(QDebug& dbg, const AssetDTO& asset)
    {
        dbg << "AssetDTO ->"
            << "\n"
            << "UUID: " << asset.getUUID() << "\n"
            << "Name: " << asset.getName() << "\n"
            << "Description: " << asset.getDescription() << "\n"
            << "Path: " << asset.getPath() << "\n"
            << "CreationDateTime: " << asset.getCreationDateTime().toString(Qt::ISODateWithMs) << "\n"
            << "LastUpdateDateTime: " << asset.getLastUpdateDateTime().toString(Qt::ISODateWithMs) << "\n"
            << "DownloadedFileList: " << asset.getDownloadedFileList() << "\n"
            << "LoadedFile: " << asset.getLoadedFile();
        return dbg;
    }

  private:
    QString               _path;
    DownloadedFileListDTO _downloadedFileList;
    FileDTO               _loadedFile;
};
} // namespace gcs
