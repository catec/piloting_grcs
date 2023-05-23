#pragma once

#include <QDialog>
#include <memory>

#include "gcs_gui_export.h"

namespace Ui {
class ManageAutoHeadingDialog;
}

namespace gcs {
class PILOTING_GRCS_GUI_EXPORT ManageAutoHeadingDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit ManageAutoHeadingDialog(QWidget* parent = nullptr);
    virtual ~ManageAutoHeadingDialog();

    void setCurrentAutoHeadingState(const bool&);
    bool getAutoHeadingState() const;

  private:
    std::unique_ptr<Ui::ManageAutoHeadingDialog> _ui;
};

} // namespace gcs
