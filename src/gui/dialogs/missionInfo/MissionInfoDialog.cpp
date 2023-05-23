
#include "MissionInfoDialog.h"

#include "ui_MissionInfoDialog.h"

namespace gcs {

MissionInfoDialog::MissionInfoDialog(QWidget* parent) : QDialog(parent), _ui(std::make_unique<Ui::MissionInfoDialog>())
{
    _ui->setupUi(this);
}

MissionInfoDialog::~MissionInfoDialog() {}

void MissionInfoDialog::setMissionName(const QString& name) const
{
    _ui->missionNameLE->setText(name);
}

void MissionInfoDialog::setMissionDescription(const QString& description) const
{
    _ui->notesTE->setPlainText(description);
}

void MissionInfoDialog::setMissionCreationDate(const QString& date) const
{
    _ui->creationDateL->setText(date);
}

void MissionInfoDialog::setMissionLastUpdateDate(const QString& date) const
{
    _ui->lastUpdateDateL->setText(date);
}

const QString MissionInfoDialog::getMissionName()
{
    return _ui->missionNameLE->text();
}

const QString MissionInfoDialog::getMissionDescription()
{
    return _ui->notesTE->toPlainText();
}

} // namespace gcs
