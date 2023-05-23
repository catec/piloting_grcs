
#pragma once

#include <QDateTime>
#include <QDebug>

#include "common/IDTO.h"

namespace gcs {

class FileDTO : public IDTO
{
  public:
    virtual ~FileDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::FILE; }

    QString& getUUID() Q_DECL_NOEXCEPT { return _uuid; }

    const QString& getUUID() const Q_DECL_NOEXCEPT { return _uuid; }

    QString& getUrl() Q_DECL_NOEXCEPT { return _url; }

    const QString& getUrl() const Q_DECL_NOEXCEPT { return _url; }

    QString& getOriginalName() Q_DECL_NOEXCEPT { return _originalName; }

    const QString& getOriginalName() const Q_DECL_NOEXCEPT { return _originalName; }

    qint64& getSize() Q_DECL_NOEXCEPT { return _size; }

    const qint64& getSize() const Q_DECL_NOEXCEPT { return _size; }

    QDateTime&       getCreationDateTime() Q_DECL_NOEXCEPT { return _creationDateTime; }
    const QDateTime& getCreationDateTime() const Q_DECL_NOEXCEPT { return _creationDateTime; }

    QDateTime& getLastUpdateDateTime() Q_DECL_NOEXCEPT { return _lastUpdateDateTime; }

    const QDateTime& getLastUpdateDateTime() const Q_DECL_NOEXCEPT { return _lastUpdateDateTime; }

    QString& getPath() Q_DECL_NOEXCEPT { return _path; }

    const QString& getPath() const Q_DECL_NOEXCEPT { return _path; }
    /// Operators
    FileDTO&       operator=(const FileDTO& o) Q_DECL_NOEXCEPT
    {
        _uuid               = o._uuid;
        _url                = o._url;
        _originalName       = o._originalName;
        _size               = o._size;
        _creationDateTime   = o._creationDateTime;
        _lastUpdateDateTime = o._lastUpdateDateTime;
        _path               = o._path;
        return *this;
    }

    bool operator==(const FileDTO& o) const Q_DECL_NOEXCEPT
    {
        return _uuid == o._uuid && _url == o._url && _originalName == o._originalName && _size == o._size
            && _creationDateTime == o._creationDateTime && _lastUpdateDateTime == o._lastUpdateDateTime
            && _path == o._path;
    }

    bool operator!=(const FileDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

    friend QDebug& operator<<(QDebug& dbg, const FileDTO& file)
    {
        dbg << "FileDTO ->"
            << "\n"
            << "UUID: " << file.getUUID() << "\n"
            << "OriginalName: " << file.getOriginalName() << "\n"
            << "URL: " << file.getUrl() << "\n"
            << "Size: " << QString::number(file.getSize()) << "\n"
            << "CreationDateTime: " << file.getCreationDateTime().toString(Qt::ISODateWithMs) << "\n"
            << "LastUpdateDateTime: " << file.getLastUpdateDateTime().toString(Qt::ISODateWithMs) << "\n";
        return dbg;
    }

  private:
    QString   _uuid{""};
    QString   _originalName{""};
    qint64    _size{0};
    QDateTime _creationDateTime;
    QDateTime _lastUpdateDateTime;
    QString   _url{""};
    QString   _path{""};
};
} // namespace gcs
