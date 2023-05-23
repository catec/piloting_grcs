#pragma once

#include <QDialog>
#include <memory>

namespace Ui {
class ConnectionConfirmationDialog;
}

namespace gcs {
class CommsLinkMavsdkDTO;

class ConnectionConfirmationDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit ConnectionConfirmationDialog(QWidget* parent = nullptr);
    virtual ~ConnectionConfirmationDialog();

    void setCommsOptions(const CommsLinkMavsdkDTO&);

  private:
    std::unique_ptr<Ui::ConnectionConfirmationDialog> _ui;
};

} // namespace gcs
