#pragma once

#include <dataModel/dtos/MissionTaskDTO.h>

#include <QList>
#include <QPushButton>
#include <QWidget>
#include <functional>
#include <memory>

namespace Ui {
class MissionTasksWidget;
}

namespace gcs {

class GuiDTO;
class CurrentMissionResultDTO;

class MissionTasksWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit MissionTasksWidget(GuiDTO& guiDTO, QWidget* parent = nullptr);
    virtual ~MissionTasksWidget();

    void updateCommsStatus(const bool);

    void setMissionTasks(const QList<MissionTaskDTO>&);

  private Q_SLOTS:
    void missionInfoButtonClicked();

  private:
    QPushButton* createCircularButton(const int, std::function<void()>);

    std::unique_ptr<Ui::MissionTasksWidget> _ui;

    GuiDTO& _guiDTO;

  Q_SIGNALS:
    void updateMissionTask(QSharedPointer<MissionTaskDTO>);
    void updateMissionResult(QSharedPointer<CurrentMissionResultDTO>);
};

} // namespace gcs
