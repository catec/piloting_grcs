
#pragma once

#include <QTimer>
#include <QWidget>
#include <memory>

namespace Ui {
class RobotControlWidget;
}

namespace gcs {
class RouteDTO;
class RobotControlWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit RobotControlWidget(QWidget* parent = nullptr);
    virtual ~RobotControlWidget();

    void updateCommsStatus(const bool);
    void updateLoadedRoute(const RouteDTO&);

  private Q_SLOTS:
    void actionButtonClicked();

  Q_SIGNALS:
    void uploadWaypointList();
    void downloadWaypointList();
    void downloadCheckList();
    void downloadAlarmsList();
    void downloadHighLevelActions();
    void setCurrentWaypointItem(quint16);
    void sendHome();

  private:
    std::unique_ptr<Ui::RobotControlWidget> _ui;
};

} // namespace gcs
