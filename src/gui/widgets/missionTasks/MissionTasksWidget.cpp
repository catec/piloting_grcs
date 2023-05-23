#include "MissionTasksWidget.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/CurrentMissionResultDTO.h>

#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>
#include <QTreeWidget>
#include <iostream>

#include "../../dialogs/missionInfo/MissionInfoDialog.h"
#include "../../dialogs/missionTask/MissionTaskDialog.h"
#include "../../dtos/GuiDTO.h"
#include "ui_MissionTasksWidget.h"

#define EDIT            0
#define ID              1
#define NAME            2
#define INSPECTION_TASK 3

namespace gcs {

MissionTasksWidget::MissionTasksWidget(GuiDTO& guiDTO, QWidget* parent) :
        QWidget(parent), _ui(std::make_unique<Ui::MissionTasksWidget>()), _guiDTO(guiDTO)
{
    _ui->setupUi(this);

    // clang-format off
    connect(_ui->editMissionB, &QPushButton::clicked, this, &MissionTasksWidget::missionInfoButtonClicked);
    // clang-format on
}

MissionTasksWidget::~MissionTasksWidget()
{
    _ui->treeWidget->clear();
}

void MissionTasksWidget::updateCommsStatus(const bool isConnected)
{
    if (!isConnected) {
        _ui->treeWidget->clear();
    }
}

void MissionTasksWidget::setMissionTasks(const QList<MissionTaskDTO>& mission_tasks)
{
    blockSignals(true);

    _ui->treeWidget->clear();
    for (const auto& mission_task : mission_tasks) {
        auto taskItem = new QTreeWidgetItem;
        if (taskItem) {
            taskItem->setFlags(static_cast<Qt::ItemFlags>(Qt::ItemIsSelectable | Qt::ItemIsEnabled));

            taskItem->setData(ID, Qt::DisplayRole, mission_task.getId());
            taskItem->setData(NAME, Qt::DisplayRole, mission_task.getName());
            taskItem->setData(INSPECTION_TASK, Qt::DisplayRole, mission_task.getInspectionTaskAssociated().getName());

            _ui->treeWidget->insertTopLevelItem(0, taskItem);

            const int btn_radius = 19;
            auto      btnEdit    = createCircularButton(btn_radius, [=]() {
                MissionTaskDialog dialog(true, this);
                dialog.setMissionTask(mission_task);
                if (dialog.exec() != QDialog::Accepted) {
                    return;
                }

                auto dto = createSharedDTO<MissionTaskDTO>();
                *dto     = dialog.getMissionTask();
                Q_EMIT updateMissionTask(dto);
            });

            /// \note. These two additional pixels are to avoid overlapping between icons
            taskItem->setSizeHint(EDIT, QSize(btn_radius + 2, btn_radius + 2));

            _ui->treeWidget->setItemWidget(taskItem, EDIT, btnEdit);
        }
    }

    _ui->treeWidget->sortItems(ID, Qt::AscendingOrder);
    _ui->treeWidget->hideColumn(ID);
    _ui->treeWidget->resizeColumnToContents(EDIT);

    // for(int idx = 0; idx < _ui->treeWidget->columnCount(); ++idx)
    //    _ui->treeWidget->resizeColumnToContents(idx);

    blockSignals(false);
}

void MissionTasksWidget::missionInfoButtonClicked()
{
    if (_guiDTO.getMissionStatus() == MissionStatusType::NoMission) {
        auto msgBox = new QMessageBox(this);
        msgBox->setModal(true);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setWindowTitle(tr("Mission Information"));
        msgBox->setText(tr("The inspection plan must be downloaded from the DDHL to have a mission"));
        msgBox->setIcon(QMessageBox::Information);
        msgBox->show();

        return;
    }

    MissionInfoDialog dialog(this);
    dialog.setMissionName(_guiDTO.getMissionName());
    dialog.setMissionCreationDate(_guiDTO.getMissionCreationDate().toString(Qt::ISODateWithMs));
    dialog.setMissionLastUpdateDate(_guiDTO.getMissionLastUpdateDate().toString(Qt::ISODateWithMs));
    dialog.setMissionDescription(_guiDTO.getMissionDescription());
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    auto dto              = createSharedDTO<CurrentMissionResultDTO>();
    dto->getName()        = dialog.getMissionName();
    dto->getDescription() = dialog.getMissionDescription();

    Q_EMIT updateMissionResult(dto);
}

QPushButton* MissionTasksWidget::createCircularButton(const int radius, std::function<void()> btn_callback)
{
    auto btn = new QPushButton(this);
    btn->setIcon(QIcon(":/images/note-icon"));
    btn->setStyleSheet(QString("border-width: 0px"));

    QRect btn_rect(0, 0, radius, radius);
    btn->setIconSize(btn_rect.size());
    btn->setFixedSize(btn_rect.size());
    btn->setMask(QRegion(btn_rect, QRegion::Ellipse));

    connect(btn, &QPushButton::clicked, btn_callback);

    return btn;
}

} // namespace gcs
