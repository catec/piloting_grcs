
#include "ManageAutoHeadingDialog.h"

#include <QsLog/QsLog.h>

#include <QLineEdit>

#include "ui_ManageAutoHeadingDialog.h"

#define ENABLED  0
#define DISABLED 1

namespace gcs {

ManageAutoHeadingDialog::ManageAutoHeadingDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::ManageAutoHeadingDialog>())
{
    _ui->setupUi(this);

    _ui->stateCB->lineEdit()->setAlignment(Qt::AlignCenter);
    _ui->stateCB->lineEdit()->setReadOnly(true);
    _ui->stateCB->addItem("Enabled");
    _ui->stateCB->addItem("Disabled");
    _ui->stateCB->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);
    _ui->stateCB->setItemData(1, Qt::AlignCenter, Qt::TextAlignmentRole);
}

ManageAutoHeadingDialog::~ManageAutoHeadingDialog() {}

void ManageAutoHeadingDialog::setCurrentAutoHeadingState(const bool& state)
{
    _ui->stateCB->setCurrentIndex(state ? ENABLED : DISABLED);
}

bool ManageAutoHeadingDialog::getAutoHeadingState() const
{
    const auto currState = _ui->stateCB->currentIndex() == ENABLED ? true : false;
    return currState;
}

} // namespace gcs
