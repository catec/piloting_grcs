#pragma once

#include <common/Types.h>

#include <QDialog>
#include <QRadioButton>
#include <QToolButton>
#include <memory>

namespace Ui {
class CommsOptionsDDHLDialog;
}

namespace gcs {
class CommsOptionsDDHLDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit CommsOptionsDDHLDialog(QWidget* parent = nullptr);
    virtual ~CommsOptionsDDHLDialog();

    void setEndpointMode(const DDHLEndpointMode&);
    void setTargetUrl(const QString&);
    void setTargetIP(const QString&);
    void setTargetPort(const quint16&);
    void setUsername(const QString&);
    void setPassword(const QString&);

    DDHLEndpointMode getEndpointMode() const;

    QString getTargetUrl() const;
    QString getTargetIP() const;
    quint16 getTargetPort() const;
    QString getUsername() const;
    QString getPassword() const;

    void accept();

  private Q_SLOTS:
    void endpointTypeChanged(bool ipChecked);

  private:
    std::unique_ptr<Ui::CommsOptionsDDHLDialog> _ui;
    QToolButton*                                _passwordB;
};

} // namespace gcs
