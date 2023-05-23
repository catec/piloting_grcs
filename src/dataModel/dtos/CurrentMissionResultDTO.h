#pragma once

#include <communications/MavSDK/dtos/CheckListDTO.h>

#include <QTextStream>

#include "MissionResultDTO.h"

namespace gcs {

class CurrentMissionResultDTO : public MissionResultDTO
{
  public:
    virtual ~CurrentMissionResultDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::CURRENT_MISSION_RESULT; }

    CheckListDTO& getCheckListDTO() Q_DECL_NOEXCEPT { return _checkListDTO; }

    const CheckListDTO& getCheckListDTO() const Q_DECL_NOEXCEPT { return _checkListDTO; }

    QString& getConsoleLogFilePath() Q_DECL_NOEXCEPT { return _consoleLogFilePath; }

    const QString& getConsoleLogFilePath() const Q_DECL_NOEXCEPT { return _consoleLogFilePath; }

    QString& getTelemetryLogFilePath() Q_DECL_NOEXCEPT { return _telemetryLogFilePath; }

    const QString& getTelemetryLogFilePath() const Q_DECL_NOEXCEPT { return _telemetryLogFilePath; }

    QString& getCheckListLogFilePath() Q_DECL_NOEXCEPT { return _checkListLogFilePath; }

    const QString& getCheckListLogFilePath() const Q_DECL_NOEXCEPT { return _checkListLogFilePath; }

    QString& getLoadedTrajectoryLogFilePath() Q_DECL_NOEXCEPT { return _loadedTrajectoryLogFilePath; }

    const QString& getLoadedTrajectoryLogFilePath() const Q_DECL_NOEXCEPT { return _loadedTrajectoryLogFilePath; }

    QString& getMissionResultLogFilePath() Q_DECL_NOEXCEPT { return _missionResultLogFilePath; }

    const QString& getMissionResultLogFilePath() const Q_DECL_NOEXCEPT { return _missionResultLogFilePath; }

    /// Operators
    CurrentMissionResultDTO& operator=(const CurrentMissionResultDTO& o) Q_DECL_NOEXCEPT
    {
        MissionResultDTO::operator=(o);
        _checkListDTO                = o._checkListDTO;
        _consoleLogFilePath          = o._consoleLogFilePath;
        _telemetryLogFilePath        = o._telemetryLogFilePath;
        _checkListLogFilePath        = o._checkListLogFilePath;
        _loadedTrajectoryLogFilePath = o._loadedTrajectoryLogFilePath;
        _missionResultLogFilePath    = o._missionResultLogFilePath;

        return *this;
    }

    bool operator==(const CurrentMissionResultDTO& o) const Q_DECL_NOEXCEPT
    {
        return MissionResultDTO::operator==(o) && _checkListDTO == o._checkListDTO
            && _consoleLogFilePath == o._consoleLogFilePath && _telemetryLogFilePath == o._telemetryLogFilePath
            && _checkListLogFilePath == o._checkListLogFilePath
            && _loadedTrajectoryLogFilePath == o._loadedTrajectoryLogFilePath
            && _missionResultLogFilePath == o._missionResultLogFilePath;
    }

    bool operator!=(const CurrentMissionResultDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    CheckListDTO _checkListDTO;
    QString      _consoleLogFilePath;
    QString      _telemetryLogFilePath;
    QString      _checkListLogFilePath;
    QString      _loadedTrajectoryLogFilePath;
    QString      _missionResultLogFilePath;
};

} // namespace gcs
