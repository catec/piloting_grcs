
#pragma once

#include <common/IDTO.h>

#include "FileDTO.h"

namespace gcs {

class DownloadedFileListDTO : public IDTO
{
  public:
    virtual ~DownloadedFileListDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::DOWNLOADED_FILE_LIST; }

    QList<FileDTO>& getDownloadedFileList() Q_DECL_NOEXCEPT { return _downloadedFileList; }

    const QList<FileDTO>& getDownloadedFileList() const Q_DECL_NOEXCEPT { return _downloadedFileList; }

    /// Operators
    DownloadedFileListDTO& operator=(const DownloadedFileListDTO& o) Q_DECL_NOEXCEPT
    {
        _downloadedFileList = o._downloadedFileList;

        return *this;
    }

    bool operator==(const DownloadedFileListDTO& o) const Q_DECL_NOEXCEPT
    {
        return _downloadedFileList == o._downloadedFileList;
    }

    bool operator!=(const DownloadedFileListDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

    friend QDebug& operator<<(QDebug& dbg, const DownloadedFileListDTO& downloadedFileList)
    {
        dbg << "DownloadedFileList ->"
            << "\n"
            << downloadedFileList.getDownloadedFileList();
        return dbg;
    }

  private:
    QList<FileDTO> _downloadedFileList;
};

} // namespace gcs
