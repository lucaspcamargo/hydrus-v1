#include "serverconfigdialog.h"
#include "ui_serverconfigdialog.h"

ServerConfigDialog::ServerConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ServerConfigDialog)
{
    ui->setupUi(this);
}

ServerConfigDialog::~ServerConfigDialog()
{
    delete ui;
}

int ServerConfigDialog::getServerPort()
{
    return ui->portSpinBox->value();
}

qreal ServerConfigDialog::getDroneSpeedEstimate()
{
    return ui->doubleSpinBox->value();
}
