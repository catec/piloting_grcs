#include "CommsProgressDialog.h"

#include <QsLog/QsLog.h>

#include <QDesktopWidget>

#include "ui_CommsProgressDialog.h"

CommsProgressDialog::CommsProgressDialog(QWidget* parent) :
        QDialog(parent), _ui(std::make_unique<Ui::CommsProgressDialog>()), _progressTimer(this)
{
    _ui->setupUi(this);

    overrideWindowFlags(Qt::Window | Qt::Dialog | Qt::FramelessWindowHint);

    _progress.append("--");
    _progress.append("\\");
    _progress.append("|");
    _progress.append("/");

    _progressTimer.setInterval(100);
    // clang-format off
    connect(&_progressTimer,   &QTimer::timeout,
         this,              &CommsProgressDialog::updateProgress);
    // clang-format on

    const QRect scr = QApplication::desktop()->screenGeometry();
    move(scr.center() - rect().center());
}

CommsProgressDialog::~CommsProgressDialog() {}

void CommsProgressDialog::setInfo(const QString& info)
{
    _ui->infoL->setText(info);
}

void CommsProgressDialog::show()
{
    _progressTimer.start();

    return QDialog::show();
}

bool CommsProgressDialog::close()
{
    _progressTimer.stop();

    return QDialog::close();
}

void CommsProgressDialog::updateProgress()
{
    QApplication::processEvents();

    _ui->progressL->setText(_progress.at(_count++));
    _count %= _progress.size();
}
