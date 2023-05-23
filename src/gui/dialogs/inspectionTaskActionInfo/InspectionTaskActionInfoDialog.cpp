
#include "InspectionTaskActionInfoDialog.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/InspectionTaskDTO.h>

#include <QLineEdit>

#include "ui_InspectionTaskActionInfoDialog.h"
namespace gcs {

InspectionTaskActionInfoDialog::InspectionTaskActionInfoDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::InspectionTaskActionInfoDialog>())
{
    _ui->setupUi(this);
}

InspectionTaskActionInfoDialog::~InspectionTaskActionInfoDialog() {}

void InspectionTaskActionInfoDialog::setInspTaskActionName(const QString& name)
{
    _ui->taskActionNameL->setText(name);
}

void InspectionTaskActionInfoDialog::setInspTaskActionUUID(const QString& uuid)
{
    _ui->taskActionUUIDL->setText(uuid);
}

void InspectionTaskActionInfoDialog::setInspTaskActionNotes(const QString& notes)
{
    _ui->taskActionNotesL->setText(notes);
}

void InspectionTaskActionInfoDialog::setInspTaskActionProperties(const QString& prop)
{
    _ui->taskActionPropL->setText(prop);
}

} // namespace gcs
