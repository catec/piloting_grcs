#include "UploadDDHLManagerDialog.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/InspectionTaskDTO.h>
#include <dataModel/dtos/MissionResultDTO.h>

#include <QMessageBox>

#include "../inspectionTaskInfo/InspectionTaskInfoDialog.h"
#include "../missionTask/MissionTaskDialog.h"
#include "ui_UploadDDHLManagerDialog.h"

#define CHECK 0
#define NAME  1
#define VALUE 2

namespace gcs {

UploadDDHLManagerDialog::UploadDDHLManagerDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::UploadDDHLManagerDialog>())
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _ui->setupUi(this);
}

UploadDDHLManagerDialog::~UploadDDHLManagerDialog()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void UploadDDHLManagerDialog::setMissionResultList(const QList<MissionResultDTO>& list)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _missionMapToUpload.clear();

    for (const auto& missionResultDTO : list) {
        insertMissionRoot(missionResultDTO);
    }

    _ui->treeWidget->resizeColumnToContents(CHECK);
    _ui->treeWidget->resizeColumnToContents(NAME);

    // _ui->treeWidget->setSelectionMode(QAbstractItemView::NoSelection);
}

QStringList UploadDDHLManagerDialog::getMissionsToUpload()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    return _missionsToUpload;
}

void UploadDDHLManagerDialog::insertMissionRoot(const MissionResultDTO& missionResult)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto treeItem = new QTreeWidgetItem(_ui->treeWidget);
    // treeItem->setFlags(Qt::NoItemFlags);
    treeItem->setText(NAME, missionResult.getName());
    treeItem->setText(VALUE, "");

    insertMissionItem(treeItem, "SyncId", missionResult.getSyncId());
    insertMissionItem(
            treeItem, "CreationTimestamp", missionResult.getCreationTimeStamp().toString("yyyy-MM-dd hh:mm:ss"));
    insertMissionItem(treeItem, "Name", missionResult.getName());
    insertMissionItem(treeItem, "Description", missionResult.getDescription());
    insertMissionItem(treeItem, "Mission Tasks", QVariant::fromValue(missionResult.getMissionTaskList()));

    auto checkBox = new QCheckBox(this);
    _ui->treeWidget->setItemWidget(treeItem, CHECK, checkBox);
    _missionMapToUpload.insert(checkBox, missionResult);
}

void UploadDDHLManagerDialog::insertMissionItem(
        QTreeWidgetItem* parent, const QString& name, const QVariant& receivedValue)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto treeItem = new QTreeWidgetItem();
    if (receivedValue.canConvert<QString>()) {
        treeItem->setText(NAME, name);
        treeItem->setData(VALUE, Qt::DisplayRole, receivedValue);

        parent->addChild(treeItem);
    } else if (receivedValue.canConvert<QList<gcs::MissionTaskDTO>>()) {
        const auto missionTaskList = receivedValue.value<QList<gcs::MissionTaskDTO>>();

        treeItem->setText(NAME, name);
        treeItem->setData(VALUE, Qt::DisplayRole, missionTaskList.size());

        parent->addChild(treeItem);

        for (const auto& mission_task : missionTaskList) {
            auto task_child = new QTreeWidgetItem();
            treeItem->addChild(task_child);

            task_child->setText(NAME, mission_task.getName());
            const int btn_radius = 14;
            auto      btn        = createCircularButton(btn_radius, [=]() {
                MissionTaskDialog dialog(false, this);
                dialog.setMissionTask(mission_task);
                if (dialog.exec() != QDialog::Accepted) {
                    return;
                }
            });
            /// \note. These two additional pixels are to avoid overlapping between icons
            task_child->setSizeHint(VALUE, QSize(btn_radius + 2, btn_radius + 2));

            _ui->treeWidget->setItemWidget(task_child, VALUE, btn);
        }
        treeItem->sortChildren(NAME, Qt::AscendingOrder);
    } else {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "Invalid input type";
    }
}

void UploadDDHLManagerDialog::accept()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _missionsToUpload.clear();
    for (const auto& checkbox : _missionMapToUpload.keys()) {
        if (checkbox->isChecked()) {
            _missionsToUpload.push_back(_missionMapToUpload.value(checkbox).getName());
        }
    }

    const auto response = QMessageBox::question(
            this,
            tr("Upload data to DDHL"),
            QString("Are you sure that you want to upload %1 missions data to DDHL?\n").arg(_missionsToUpload.size()));

    if (response == QMessageBox::No) {
        return;
    }

    QDialog::accept();
}

QPushButton* UploadDDHLManagerDialog::createCircularButton(const int radius, std::function<void()> btn_callback)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    auto btn = new QPushButton(this);
    btn->setIcon(QIcon(":/images/info"));
    btn->setStyleSheet(QString("border-width: 0px"));

    QRect btn_rect(0, 0, radius, radius);
    btn->setIconSize(btn_rect.size());
    btn->setFixedSize(btn_rect.size());
    btn->setMask(QRegion(btn_rect, QRegion::Ellipse));

    connect(btn, &QPushButton::clicked, btn_callback);

    return btn;
}

} // namespace gcs
