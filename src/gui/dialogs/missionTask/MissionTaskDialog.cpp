#include "MissionTaskDialog.h"

#include <QDateTime>
#include <QMessageBox>
#include <QPushButton>

#include "../inspectionTaskInfo/InspectionTaskInfoDialog.h"
#include "ui_MissionTaskDialog.h"

namespace gcs {

MissionTaskDialog::MissionTaskDialog(const bool isEditable, QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::MissionTaskDialog>())
{
    _ui->setupUi(this);

    _ui->nameLE->setReadOnly(!isEditable);
    _ui->descriptionTE->setReadOnly(!isEditable);
    if (isEditable) {
        _ui->buttonLayout->addWidget(createCircularButton(19, [this]() {
            InspectionTaskInfoDialog dialog(this);
            dialog.setInspectionTaskInfo(_missionTaskDTO.getInspectionTaskAssociated());
            if (dialog.exec() != QDialog::Accepted) {
                return;
            }
        }));
    }
}

MissionTaskDialog::~MissionTaskDialog() {}

void MissionTaskDialog::setMissionTask(const MissionTaskDTO& dto)
{
    _missionTaskDTO = dto;

    _ui->nameLE->setText(_missionTaskDTO.getName());
    _ui->descriptionTE->setPlainText(_missionTaskDTO.getDescription());
    _ui->dateL->setText(_missionTaskDTO.getCreationDateTime().toString(Qt::ISODateWithMs));
    _ui->lastUpdateDateL->setText(_missionTaskDTO.getLastUpdateDateTime().toString(Qt::ISODateWithMs));
    _ui->inspectionTaskL->setText(_missionTaskDTO.getInspectionTaskAssociated().getUUID());
}

MissionTaskDTO MissionTaskDialog::getMissionTask()
{
    if (_missionTaskDTO.getName() != _ui->nameLE->text()
        || _missionTaskDTO.getDescription() != _ui->descriptionTE->toPlainText()) {
        _missionTaskDTO.getName()               = _ui->nameLE->text();
        _missionTaskDTO.getDescription()        = _ui->descriptionTE->toPlainText();
        _missionTaskDTO.getLastUpdateDateTime() = QDateTime::currentDateTime();
    }

    return _missionTaskDTO;
}

void MissionTaskDialog::accept()
{
    if (!checkCorrectMissionName()) {
        return;
    }

    QDialog::accept();
}

bool MissionTaskDialog::checkCorrectMissionName()
{
    try {
        if (_ui->nameLE->text().isEmpty()) {
            throw std::runtime_error("Mission task name cannot be empty. Please, introduce it again.");
        }

        if (_ui->nameLE->text().contains(" ")) {
            throw std::runtime_error("Mission name cannot contain blank characters.");
        }

        return true;
    } catch (std::runtime_error& e) {
        auto msgBox = new QMessageBox(this);
        msgBox->setModal(true);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setWindowTitle("Mission Task Name");
        msgBox->setText(e.what());
        msgBox->setIcon(QMessageBox::Warning);
        msgBox->show();

        return false;
    }
}

QPushButton* MissionTaskDialog::createCircularButton(const int radius, std::function<void()> btn_callback)
{
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
