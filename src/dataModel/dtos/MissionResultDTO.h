#pragma once

#include "MissionTaskDTO.h"

namespace gcs {

class MissionResultDTO : public IDTO
{
  public:
    virtual ~MissionResultDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::MISSION_RESULT; }

    QString& getName() Q_DECL_NOEXCEPT { return _name; }

    const QString& getName() const Q_DECL_NOEXCEPT { return _name; }

    qint32& getSyncId() Q_DECL_NOEXCEPT { return _sync_id; }

    const qint32& getSyncId() const Q_DECL_NOEXCEPT { return _sync_id; }

    QString& getInspectionPlanUUID() Q_DECL_NOEXCEPT { return _inspection_plan_uuid; }

    const QString& getInspectionPlanUUID() const Q_DECL_NOEXCEPT { return _inspection_plan_uuid; }

    QString& getDescription() Q_DECL_NOEXCEPT { return _description; }

    const QString& getDescription() const Q_DECL_NOEXCEPT { return _description; }

    QDateTime& getCreationTimeStamp() Q_DECL_NOEXCEPT { return _creationTimeStamp; }

    const QDateTime& getCreationTimeStamp() const Q_DECL_NOEXCEPT { return _creationTimeStamp; }

    QDateTime& getLastUpdateTimeStamp() Q_DECL_NOEXCEPT { return _lastUpdateTimeStamp; }

    const QDateTime& getLastUpdateTimeStamp() const Q_DECL_NOEXCEPT { return _lastUpdateTimeStamp; }

    QList<MissionTaskDTO>& getMissionTaskList() Q_DECL_NOEXCEPT { return _missionTaskList; }

    const QList<MissionTaskDTO>& getMissionTaskList() const Q_DECL_NOEXCEPT { return _missionTaskList; }

    /// Operators
    MissionResultDTO& operator=(const MissionResultDTO& o) Q_DECL_NOEXCEPT
    {
        _sync_id              = o._sync_id;
        _name                 = o._name;
        _inspection_plan_uuid = o._inspection_plan_uuid;
        _description          = o._description;
        _missionTaskList      = o._missionTaskList;
        _creationTimeStamp    = o._creationTimeStamp;
        _lastUpdateTimeStamp  = o._lastUpdateTimeStamp;

        return *this;
    }

    bool operator==(const MissionResultDTO& o) const Q_DECL_NOEXCEPT
    {
        return _sync_id == o._sync_id && _name == o._name && _description == o._description
            && _inspection_plan_uuid == o._inspection_plan_uuid && _missionTaskList == o._missionTaskList
            && _creationTimeStamp == o._creationTimeStamp && _lastUpdateTimeStamp == o._lastUpdateTimeStamp;
    }

    bool operator!=(const MissionResultDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QString               _name;
    qint32                _sync_id{-1};
    QString               _inspection_plan_uuid;
    QString               _description;
    QList<MissionTaskDTO> _missionTaskList;
    QDateTime             _creationTimeStamp;
    QDateTime             _lastUpdateTimeStamp;
};

} // namespace gcs
