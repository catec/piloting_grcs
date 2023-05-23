#pragma once

#include <dataModel/dtos/MissionTaskDTO.h>

#include <QDialog>
#include <functional>
#include <memory>

namespace Ui {
class MissionTaskDialog;
}

namespace gcs {

class MissionTaskDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit MissionTaskDialog(const bool isEditable = true, QWidget* parent = nullptr);
    virtual ~MissionTaskDialog();

    void           setMissionTask(const MissionTaskDTO&);
    MissionTaskDTO getMissionTask();

  private:
    void accept();

    bool checkCorrectMissionName();

    QPushButton* createCircularButton(const int, std::function<void()>);

    std::unique_ptr<Ui::MissionTaskDialog> _ui;
    MissionTaskDTO                         _missionTaskDTO;
};

} // namespace gcs
