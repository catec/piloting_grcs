#pragma once

#include <QDialog>
#include <memory>

namespace Ui {
class CommsOptionsMavsdkDialog;
}

namespace gcs {

class CommsOptionsMavsdkDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit CommsOptionsMavsdkDialog(QWidget* parent = nullptr);
    virtual ~CommsOptionsMavsdkDialog();

    void setHeartbeatInterval(const quint16);
    void setLocalSystemId(const quint16);
    void setLocalComponentId(const quint16);
    void setLocalIP(const QString);
    void setLocalPort(const quint16);
    void setTargetIP(const QString);
    void setTargetPort(const quint16);
    void setTargetSystemId(const quint16);

    quint16 getHeartbeatInterval() const;
    quint16 getLocalSystemId() const;
    quint16 getLocalComponentId() const;
    QString getLocalIP() const;
    quint16 getLocalPort() const;
    QString getTargetIP() const;
    quint16 getTargetPort() const;
    quint16 getTargetSystemId() const;

    void accept();

  private:
    std::unique_ptr<Ui::CommsOptionsMavsdkDialog> _ui;
};

} // namespace gcs
