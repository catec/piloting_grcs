#pragma once

#include <QDialog>
#include <memory>

namespace Ui {
class CheckListDialog;
}

class QTreeWidgetItem;

namespace gcs {

class CheckListDTO;

class CheckListDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit CheckListDialog(QWidget* parent = nullptr);
    virtual ~CheckListDialog();

    void         setCheckList(const CheckListDTO&);
    CheckListDTO getCheckList() const;

  private:
    std::unique_ptr<Ui::CheckListDialog> _ui;

    QList<QTreeWidgetItem*> _items;
};

} // namespace gcs
