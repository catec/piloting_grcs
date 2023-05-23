#pragma once

#include <QDialog>
#include <QTimer>
#include <memory>

#include "gcs_gui_export.h"

namespace Ui {
class CommsProgressDialog;
}

class PILOTING_GRCS_GUI_EXPORT CommsProgressDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit CommsProgressDialog(QWidget* parent = nullptr);
    virtual ~CommsProgressDialog();

    void setInfo(const QString&);

  public Q_SLOTS:
    void show();
    bool close();

  private Q_SLOTS:
    void updateProgress();

  private:
    std::unique_ptr<Ui::CommsProgressDialog> _ui;

    QTimer      _progressTimer;
    QStringList _progress;
    int         _count{0};
};
