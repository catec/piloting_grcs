
#include "InspectionTypeInfoDialog.h"

#include "ui_InspectionTypeInfoDialog.h"

namespace gcs {

InspectionTypeInfoDialog::InspectionTypeInfoDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::InspectionTypeInfoDialog>())
{
    _ui->setupUi(this);
}

InspectionTypeInfoDialog::~InspectionTypeInfoDialog() {}

void InspectionTypeInfoDialog::setInspectionTypeInfo(const InspectionTypeDTO& inspType)
{
    _ui->nameL->setText(inspType.getName());
    _ui->uuidL->setText(inspType.getUUID());
    _ui->descriptionL->setText(inspType.getDescription());
}

} // namespace gcs
