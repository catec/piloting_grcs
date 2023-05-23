
#pragma once

#include <common/Types.h>

#include <QMap>
#include <QTimer>
#include <QTreeWidgetItem>
#include <QWidget>
#include <memory>
#include <vector>

namespace Ui {
class RobotAlarmsWidget;
}

class QLabel;

namespace gcs {
class AlarmListDTO;
class AlarmStatusDTO;

class RobotAlarmsWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit RobotAlarmsWidget(QWidget* parent = nullptr);
    virtual ~RobotAlarmsWidget();

    void updateCommsStatus(const bool);

    void setAlarmList(const AlarmListDTO&);
    void resetAlarmList();

    void update(const AlarmStatusDTO&);

  private Q_SLOTS:
    void updateLabelTimedOut();

  private:
    void updateAlarm(QLabel*, const AlarmStatusType&);

  private:
    std::unique_ptr<Ui::RobotAlarmsWidget> _ui;

    QMap<int, QTreeWidgetItem*> _items;
    QMap<int, QTimer*>          _itemsTimers;
};

} // namespace gcs
