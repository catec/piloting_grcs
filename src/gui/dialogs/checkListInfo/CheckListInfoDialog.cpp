
#include "CheckListInfoDialog.h"

#include <common/Types.h>

#include <QLabel>
#include <QVBoxLayout>

#include "communications/MavSDK/dtos/CheckListDTO.h"
#include "ui_CheckListInfoDialog.h"

#define CHECK_STATUS 0
#define ID           1
#define NAME         2
#define DESCRIPTION  3

namespace gcs {
CheckListInfoDialog::CheckListInfoDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::CheckListInfoDialog>())
{
    _ui->setupUi(this);
}

CheckListInfoDialog::~CheckListInfoDialog() {}

void CheckListInfoDialog::setCheckList(const CheckListDTO& checklistDTO)
{
    for (const auto& checkItem : checklistDTO.getCheckItemsList()) {
        insertLabel(CHECK_STATUS, ToString(checkItem.getCheckStatus()));
        insertLabel(ID, QString::number(checkItem.getIndex()));
        insertLabel(NAME, checkItem.getName());
        insertLabel(DESCRIPTION, checkItem.getDescription());
    }

    _ui->gridLayout->setSizeConstraint(QLayout::SetFixedSize);
}

void CheckListInfoDialog::insertLabel(quint16 item, QString msg)
{
    QLabel*      label = new QLabel(this);
    QVBoxLayout* layout;

    switch (item) {
        case CHECK_STATUS:
            layout = _ui->checkStatusLayout;
            break;
        case ID:
            layout = _ui->idLayout;
            break;
        case NAME:
            layout = _ui->nameLayout;
            break;
        case DESCRIPTION:
            layout = _ui->descriptionLayout;
            break;
        default:
            return;
            break;
    }

    if (!layout) {
        return;
    }

    label->setText(msg);
    layout->addWidget(label);
}

} // namespace gcs
