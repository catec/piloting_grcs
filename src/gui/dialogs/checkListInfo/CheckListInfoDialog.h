#pragma once

#include <QDialog>
#include <memory>

namespace Ui {
class CheckListInfoDialog;
}

namespace gcs {
class CheckListDTO;

class CheckListInfoDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit CheckListInfoDialog(QWidget* parent = nullptr);
    virtual ~CheckListInfoDialog();

    void setCheckList(const CheckListDTO&);

  private:
    void insertLabel(quint16, QString);

    std::unique_ptr<Ui::CheckListInfoDialog> _ui;
};
} // namespace gcs
