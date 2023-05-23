
#include "ConnectionConfirmationDialog.h"

#include "dataModel/dtos/CommsLinkMavsdkDTO.h"
#include "ui_ConnectionConfirmationDialog.h"

namespace gcs {

ConnectionConfirmationDialog::ConnectionConfirmationDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::ConnectionConfirmationDialog>())
{
    _ui->setupUi(this);
}

ConnectionConfirmationDialog::~ConnectionConfirmationDialog() {}

void ConnectionConfirmationDialog::setCommsOptions(const CommsLinkMavsdkDTO& dto)
{
    _ui->heartBeatL->setText(QString::number(dto.getHeartBeatInterval()));
    _ui->localSystemIdL->setText(QString::number(dto.getLocalSystemId()));
    _ui->localCompIdL->setText(QString::number(dto.getLocalComponentId()));
    _ui->localIPL->setText(dto.getLocalIP());
    _ui->localPortL->setText(QString::number(dto.getLocalPort()));
    _ui->tgtSystemIdL->setText(QString::number(dto.getTargetSystemId()));
    _ui->tgtIPL->setText(dto.getTargetIP());
    _ui->tgtPortL->setText(QString::number(dto.getTargetPort()));
}

} // namespace gcs
