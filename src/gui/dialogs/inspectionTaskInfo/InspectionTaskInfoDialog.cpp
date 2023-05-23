
#include "InspectionTaskInfoDialog.h"

#include <dialogs/inspectionTypeInfo/InspectionTypeInfoDialog.h>

#include "ui_InspectionTaskInfoDialog.h"
namespace gcs {

InspectionTaskInfoDialog::InspectionTaskInfoDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::InspectionTaskInfoDialog>())
{
    _ui->setupUi(this);
}

InspectionTaskInfoDialog::~InspectionTaskInfoDialog() {}

void InspectionTaskInfoDialog::setInspectionTaskInfo(const InspectionTaskDTO& inspTaskDTO)
{
    _ui->nameL->setText(inspTaskDTO.getName());
    _ui->uuidL->setText(inspTaskDTO.getUUID());
    _ui->creationDateL->setText(inspTaskDTO.getCreationDateTime().toString(Qt::ISODateWithMs));
    _ui->lastUpdateDateL->setText(inspTaskDTO.getLastUpdateDateTime().toString(Qt::ISODateWithMs));
    _ui->locationType->setText(ToString(inspTaskDTO.getLocationType()));
    _ui->descriptionL->setText(inspTaskDTO.getDescription());
    _ui->typeL->setText(inspTaskDTO.getType().getUUID());

    _ui->buttonLayout->addWidget(createCircularButton(19, [&inspTaskDTO, this]() {
        InspectionTypeInfoDialog dialog(this);
        dialog.setInspectionTypeInfo(inspTaskDTO.getType());
        if (dialog.exec() != QDialog::Accepted) {
            return;
        }
    }));
}

QPushButton* InspectionTaskInfoDialog::createCircularButton(const int radius, std::function<void()> btn_callback)
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
