#ifndef SERVERCONFIGDIALOG_H
#define SERVERCONFIGDIALOG_H

#include <QDialog>

namespace Ui {
class ServerConfigDialog;
}

class ServerConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ServerConfigDialog(QWidget *parent = 0);
    ~ServerConfigDialog();

    int getServerPort();

    qreal getDroneSpeedEstimate();

private:
    Ui::ServerConfigDialog *ui;
};

#endif // SERVERCONFIGDIALOG_H
