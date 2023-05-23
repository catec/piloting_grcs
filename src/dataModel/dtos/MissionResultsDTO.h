#pragma once

#include <QList>

#include "MissionResultDTO.h"

namespace gcs {

class MissionResultsDTO : public IDTO
{
  public:
    virtual ~MissionResultsDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MISSION_RESULTS; }

    QString& getFolderPath() Q_DECL_NOEXCEPT { return _folderPath; }

    const QString& getFolderPath() const Q_DECL_NOEXCEPT { return _folderPath; }

    QList<MissionResultDTO>& getList() Q_DECL_NOEXCEPT { return _list; }

    const QList<MissionResultDTO>& getList() const Q_DECL_NOEXCEPT { return _list; }

    /// Operators
    MissionResultsDTO& operator=(const MissionResultsDTO& o) Q_DECL_NOEXCEPT
    {
        _folderPath = o._folderPath;
        _list       = o._list;

        return *this;
    }

    bool operator==(const MissionResultsDTO& o) const Q_DECL_NOEXCEPT
    {
        return _folderPath == o._folderPath && _list == o._list;
    }

    bool operator!=(const MissionResultsDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString                 _folderPath;
    QList<MissionResultDTO> _list;
};

} // namespace gcs
