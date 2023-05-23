
#include "InspectionPlanInfoDialog.h"

#include "ui_InspectionPlanInfoDialog.h"

namespace gcs {

InspectionPlanInfoDialog::InspectionPlanInfoDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::InspectionPlanInfoDialog>())
{
    _ui->setupUi(this);
}

InspectionPlanInfoDialog::~InspectionPlanInfoDialog() {}

void InspectionPlanInfoDialog::setInspectionPlanInfo(const InspectionPlanDTO& inspectionPlanDTO)
{
    _ui->nameL->setText(inspectionPlanDTO.getName());
    _ui->uuidL->setText(inspectionPlanDTO.getUUID());
    _ui->creationDateL->setText(inspectionPlanDTO.getCreationDateTime().toString(Qt::ISODateWithMs));
    _ui->lastUpdateDateL->setText(inspectionPlanDTO.getLastUpdateDateTime().toString(Qt::ISODateWithMs));
    _ui->descriptionL->setText(inspectionPlanDTO.getDescription());
}

} // namespace gcs
