#pragma once

#include <QsLog/QsLog.h>
#include <communications/MavSDK/dtos/CurrentInspItemDTO.h>
#include <communications/MavSDK/dtos/ReachedInspItemDTO.h>

#include <QFile>
#include <QTimer>
#include <QTreeWidget>
#include <memory>

#include "common/LoggingManager.h"

namespace Ui {
class TelemetryTreeWidget;
}

namespace gcs {

class GuiDTO;
class RobotDataBaseDTO;
class RobotPositionVelocityNedDTO;
class RobotAttitudeAngVelocityDTO;

class TelemetryTreeWidget : public QTreeWidget
{
    Q_OBJECT

  public:
    explicit TelemetryTreeWidget(GuiDTO& guiDTO, QWidget* parent = nullptr);
    virtual ~TelemetryTreeWidget();

    void setUpdateInterval(const int);

    void updateCommsStatus(const bool);

    void update(QSharedPointer<RobotDataBaseDTO>);
    void update(QSharedPointer<CurrentInspItemDTO>);
    void update(QSharedPointer<ReachedInspItemDTO>);

    void updateLogging(const QString&);

  private Q_SLOTS:
    void updateWidget();

  private:
    void resetValues();

    void insertItems();
    void insertRobotPositionVelocityItems();
    void insertRobotAttitudeItems();
    void insertWaypointsIndexItems();

    void update(const RobotPositionVelocityNedDTO&);
    void update(const RobotAttitudeAngVelocityDTO&);

    void insertItem(const quint8, QTreeWidgetItem*, const QString&, const QString& value = "0");
    void updateItem(const quint8, const QVariant&);

    void addTelemetryMsg(const QString&, const RobotPositionVelocityNedDTO&, const RobotAttitudeAngVelocityDTO&);

  private:
    std::unique_ptr<Ui::TelemetryTreeWidget> _ui;

    GuiDTO& _guiDTO;

    std::unique_ptr<LoggingManager> _loggingManager;

    QMap<int, QTreeWidgetItem*> _items;

    QTimer _updateTimer;

    QSharedPointer<RobotPositionVelocityNedDTO> _lastRobotPositionVelocityDTO{nullptr};
    QSharedPointer<RobotAttitudeAngVelocityDTO> _lastRobotAttitudeAngVelocityDTO{nullptr};
    QSharedPointer<CurrentInspItemDTO>          _lastCurrentInspItemDTO{nullptr};
    QSharedPointer<ReachedInspItemDTO>          _lastReachedInspItemDTO{nullptr};
};

} // namespace gcs
