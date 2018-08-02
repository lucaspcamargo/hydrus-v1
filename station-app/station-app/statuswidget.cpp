#include "statuswidget.h"
#include "ui_statuswidget.h"

#include <QLabel>

StatusWidget::StatusWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StatusWidget),
    m_previousBBValid(false),
    m_didInitial(false)
{
    ui->setupUi(this);

    QLabel * l = new QLabel(ui->displayWidget);
    l->setPixmap(QPixmap(":/res/glass-refl.png"));
    l->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    l->setMinimumSize(500, 500);
    l->setAlignment(Qt::AlignLeft | Qt::AlignTop);
}

StatusWidget::~StatusWidget()
{
    delete ui;
}

const char * StatusWidget__sysStateStr(SystemState st)
{
  const char * const tbl[] =
     {"INITIALIZING",
      "READY",
      "NAVIGATING",
      "BUSY",
      "HALTED",
      "FAULT"};

  return tbl[(int) (st%6)];
}

const char * StatusWidget__sysStateColor(SystemState st)
{
  const char * const tbl[] =
     {"#936",
      "#690",
      "#fc0",
      "#f60",
      "#ff5050",
      "#c00"};

  return tbl[(int) st%6];
}


const char * StatusWidget__navStateStr(NavigationState st)
{
  const char * const tbl[] =
     {"NOT NAVIGATING",
      "ALIGN",
      "ALIGN_WAIT",
      "TRAVERSE",
      "ARRIVAL_WAIT",
      "ACQUIRE",
      "HOMING EMERGENCY"};

  return tbl[(int) st];
}

#define UPDATE_BLOCK(x) if((!m_didInitial) || (m_previousBB.x != bb.x) )
void StatusWidget::blackboardUpdate(Blackboard *bb_in)
{
    const Blackboard &bb = *bb_in; // turn pointer into reference

    if(!m_previousBBValid)
        m_previousBB = bb;

    UPDATE_BLOCK(sys.state)
    {
        ui->dSysState->setText(StatusWidget__sysStateStr(bb.sys.state));
        ui->dSysState->setStyleSheet(QStringLiteral("color: ") + StatusWidget__sysStateColor(bb.sys.state) + ";");
    }

    UPDATE_BLOCK(sys.battVoltage)
    {
        ui->dSysBattVoltage->setText(QString("%1 V").arg(bb.sys.battVoltage, 0, 'd', 2));
    }

    UPDATE_BLOCK(sys.battLevel)
    {
        ui->dSysBattLevel->setValue((int)(bb.sys.battLevel*100));
        ui->dSysBattLevel->setStyleSheet(bb.sys.battLevel > 0.85f? "color: #7ec242" : (bb.sys.battLevel < 0.1f? "color: red" : (bb.sys.battLevel < 0.25f? "color: #f67400" : (bb.sys.battLevel < 0.5f? "color: orange" : ""))));
    }

    UPDATE_BLOCK(sys.coreTemp)
    {
        ui->dSysCoreTempLevel->setText(QString("%1°C").arg(bb.sys.coreTemp));
    }

    UPDATE_BLOCK(sys.cpuLoad)
    {
        ui->dSysCpuLoadLevel->setValue(bb.sys.cpuLoad);
    }

    UPDATE_BLOCK(nav.state)
    {
        ui->dNavState->setText(StatusWidget__navStateStr(bb.nav.state));
    }

    UPDATE_BLOCK(nav.gpsHasFix)
    {
        ui->dNavGpsFix->setText(bb.nav.gpsHasFix? "Yes" : "No");
    }

    UPDATE_BLOCK(nav.gpsLon)
    {
        ui->dNavGpsLon->setText(bb.nav.gpsHasFix? QString("%1").arg(bb.nav.gpsLon) : "-");
    }
    UPDATE_BLOCK(nav.gpsLat)
    {
        ui->dNavGpsLat->setText(bb.nav.gpsHasFix? QString("%1").arg(bb.nav.gpsLat) : "-");
    }
    UPDATE_BLOCK(nav.geoHeading)
    {
        ui->dNavGpsHeading->setText(bb.nav.gpsHasFix? QString("%1").arg(bb.nav.geoHeading) : "-");
    }
    UPDATE_BLOCK(nav.distanceFromNextWaypoint)
    {
        ui->dNavGPSDistanceNext->setText(bb.nav.gpsHasFix? QString("%1").arg(bb.nav.distanceFromNextWaypoint) : "-");
    }

    UPDATE_BLOCK(sensors.waterTemp)
    {
        ui->dSensTemp->setText(QString("%1°C").arg(bb.sensors.waterTemp, 0, 'f', 2));
    }

    UPDATE_BLOCK(sensors.waterTurb)
    {
        ui->dSensTurbidity->setText(QString("%1 NTU").arg((5 - bb.sensors.waterTurb)/1081.08, 0, 'f', 2));
    }

    UPDATE_BLOCK(sensors.waterPH)
    {
        ui->dSensPH->setText(QString("%1").arg(bb.sensors.waterPH*14/5, 0, 'f', 2));
    }

    m_previousBBValid = true;
    m_didInitial = true;
}

void StatusWidget::addressChanged(QString addr)
{
    ui->dIP->setText(addr);
}

void StatusWidget::connectionUpdate(bool connected)
{
    if(!connected)
    {
        ui->dSysState->setText("DISCONNECTED");
        ui->dSysState->setStyleSheet("color: rgb(85, 100, 85);");

        const char *emd = "─";

        ui->dSysBattVoltage->setText(emd);
        ui->dSysBattLevel->setValue(0);
        ui->dSysCoreTempLevel->setText(emd);
        ui->dSysCpuLoadLevel->setValue(0);

        ui->dNavGpsFix->setText(emd);
        ui->dNavGpsLat->setText(emd);
        ui->dNavGpsLon->setText(emd);
        ui->dNavGpsHeading->setText(emd);
        ui->dNavGPSDistanceNext->setText(emd);

        ui->dSensTemp->setText(emd);
        ui->dSensTurbidity->setText(emd);
        ui->dSensPH->setText(emd);

        m_didInitial = false;
    }

    ui->rebootBtn->setEnabled(connected);
    ui->haltBtn->setEnabled(connected);
    ui->abortNavBtn->setEnabled(connected);

    m_previousBBValid = false;
}

void StatusWidget::on_rebootBtn_clicked()
{
    emit rebootClicked();
}

void StatusWidget::on_haltBtn_clicked()
{
    emit haltClicked();
}

void StatusWidget::on_abortNavBtn_clicked()
{
    emit abortNavClicked();
}
