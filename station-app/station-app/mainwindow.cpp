#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QScrollBar>
#include "droneconnection.h"
#include "serverconfigdialog.h"

#include "dronemap.h"
#include "droneroute.h"
#include "dronerouteset.h"
#include "droneblackboard.h"
#include "dronesimulator.h"
#include "dronelog.h"
#include "logtimelinewidget.h"

#include <marble/GeoDataCoordinates.h>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_logTimeline(0)
{
    ui->setupUi(this);
    ui->rcDock->setVisible(false);

    m_config = new ServerConfigDialog(this);

    m_mapWidget = new DroneMap(ui->mapContainer);
    m_currRoute = 0;

    m_routes = new DroneRouteSet(this);
    m_routes->restore();
    ui->routesCombo->setModel(m_routes);

    if(!m_routes->rowCount(QModelIndex()))
    {
        DroneRoute *r = m_routes->addRoute("(DEFAULT ROUTE)", -48.7658, -26.2251);
        r->setReturnHome(true);
        setCurrentRoute(r);
    }
    else
    {
        setCurrentRoute(m_routes->firstRoute());
    }

    // end bullshit test code

    QHBoxLayout *l = new QHBoxLayout;
    l->addWidget(m_mapWidget);
    l->setMargin(1);
    ui->mapContainer->setLayout(l);

    // restore settings
    restore();

    m_conn = new DroneConnection();
    connect(m_conn, &DroneConnection::connectedChanged, this , &MainWindow::handleConnectedChanged);
    connect(m_conn, &DroneConnection::logReceived, this , &MainWindow::appendLogMessage);
    connect(m_conn, &DroneConnection::blackboardChanged, this , &MainWindow::blackboardUpdate);
    connect(ui->statusDockContents, &StatusWidget::rebootClicked, m_conn, &DroneConnection::rebootDrone);
    connect(ui->statusDockContents, &StatusWidget::haltClicked, m_conn, &DroneConnection::haltDrone);
    connect(ui->statusDockContents, &StatusWidget::abortNavClicked, m_conn, &DroneConnection::abortDroneNavigation);
    handleConnectedChanged(false);

    connect(m_conn, &DroneConnection::connectedChanged, ui->statusDockContents , &StatusWidget::connectionUpdate);
    connect(m_conn, &DroneConnection::blackboardChanged, ui->statusDockContents , &StatusWidget::blackboardUpdate);


    connect(ui->actionRemote_Control, &QAction::toggled, ui->rcDockContents, &RCWidget::toggleRCMode);
    ui->rcDockContents->setDroneConnection(m_conn);

    m_sim = new DroneSimulator(this, m_conn);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::handleConnectedChanged(bool conn)
{
    if(conn)
    {
        appendLogMessage("Connected to drone");
    }
    else
    {
        appendLogMessage("Disconnected from drone");
    }

    ui->statusDockContents->addressChanged(m_conn->host());

    ui->actionDisconnect->setEnabled(conn);
    ui->actionServer_Setup->setEnabled(!conn);

    ui->actionRemote_Control->setEnabled(conn);
    ui->actionRemote_Control->setChecked(false);
    ui->actionNavigate->setEnabled(conn);

    m_mapWidget->setDroneHasFix(false);
    m_mapWidget->setDroneLongitudeDeg(-48.7661);
    m_mapWidget->setDroneLatitudeDeg(-26.2248);

    ui->actionSimulation_Mode->setEnabled(conn);
}

void MainWindow::zoomCh(int)
{
}

void MainWindow::blackboardUpdate(Blackboard *bb)
{
    m_mapWidget->setDroneHasFix(bb->nav.gpsHasFix);
    m_mapWidget->setDroneLongitudeDeg(bb->nav.gpsLon);
    m_mapWidget->setDroneLatitudeDeg(bb->nav.gpsLat);
    m_mapWidget->setDroneHeadingDeg(bb->sys.state == SS_NAVIGATING? bb->nav.geoHeading : bb->sensors.imuHeading);
    m_mapWidget->update();
}

void MainWindow::appendLogMessage(const QString& text)
{
    bool scroll = ui->textEdit->verticalScrollBar()->value() == ui->textEdit->verticalScrollBar()->maximum();

    ui->textEdit->append(QDateTime::currentDateTime().toString() + ": " + text); // Adds the message to the widget

    if(scroll)
        ui->textEdit->verticalScrollBar()->setValue(ui->textEdit->verticalScrollBar()->maximum()); // Scrolls to the bottom
}

void MainWindow::on_actionDisconnect_triggered()
{
     m_conn->disconnect();
}

void MainWindow::setCurrentRoute(DroneRoute *r)
{
    m_currRoute = r;

    m_mapWidget->setRoute(m_currRoute);
    ui->routeWaypointsView->setModel(m_currRoute);

    ui->routeBaseLocation->setText(
                Marble::GeoDataCoordinates(m_currRoute->homeLongDeg(), m_currRoute->homeLatDeg(), 0.0, Marble::GeoDataCoordinates::Degree).toString());
    ui->routeReturnToBase->setChecked(m_currRoute->returnHome());

    updateRouteInfo();
}

#include <QTime>

void MainWindow::updateRouteInfo()
{
    if(m_currRoute)
    {
        qreal dist = m_currRoute->calculateTotalDistance();
        qreal estSecs = (dist/m_config->getDroneSpeedEstimate());
        int mins = (int)estSecs/60;

        ui->routeInfoLen->setText(QString("%1 m").arg(dist));
        if(mins < 60)
            ui->routeInfoEstTime->setText(QString("%1 min").arg(mins));
        else
            ui->routeInfoEstTime->setText(QString("%1 h %2 min").arg(mins/60).arg(mins - (mins/60)*60));
    }
    else
    {

    }
    m_mapWidget->update();
}

#include <QSettings>

void MainWindow::save()
{
    QSettings s;
    s.beginGroup("Main");

    s.setValue("centerLon", m_mapWidget->centerLongitude());
    s.setValue("centerLat", m_mapWidget->centerLatitude());
    s.setValue("zoom", m_mapWidget->zoom());
    s.setValue("currRouteIndex", ui->routesCombo->currentIndex());

    s.endGroup();
}

void MainWindow::restore()
{
    QSettings s;
    s.beginGroup("Main");

    m_mapWidget->setCenterLongitude(s.value("centerLon").toReal() ,Marble::Jump);
    m_mapWidget->setCenterLatitude(s.value("centerLat").toReal() ,Marble::Jump);
    m_mapWidget->setZoom(s.value("zoom").toInt() ,Marble::Jump);
    ui->routesCombo->setCurrentIndex(s.value("currRouteIndex").toInt());

    s.endGroup();
}


void MainWindow::on_logClearBtn_clicked()
{
    ui->textEdit->clear();
}

#include <QFileDialog>
#include <QFile>
void MainWindow::on_logSaveBtn_clicked()
{
    QString name = QFileDialog::getSaveFileName(this, "Save Log");
    if(name.isEmpty())
        return;

    QString log = ui->textEdit->toPlainText().append('\n');

    QFile file(name);
    file.open(QFile::WriteOnly);
    file.write(log.toLocal8Bit());
    file.close();
}

void MainWindow::on_actionServer_Setup_triggered()
{
    m_config->exec();
}

#include "aboutdialog.h"
void MainWindow::on_actionAbout_triggered()
{
    AboutDialog d(this);
    d.exec();
}

void MainWindow::on_actionCenter_Map_triggered()
{
    if(m_conn->connected())
    {
        if(m_conn->blackboard()->nav.gpsHasFix)
        {
            m_mapWidget->setCenterLatitude(m_conn->blackboard()->nav.gpsLat);
            m_mapWidget->setCenterLongitude(m_conn->blackboard()->nav.gpsLon);
        }
        else appendLogMessage("GPS has no fix. Can't center.");
    }
    else appendLogMessage("Not connected. Can't center.");
}

void MainWindow::on_routeReturnToBase_clicked()
{
    if(m_currRoute)
    {
        m_currRoute->setReturnHome(ui->routeReturnToBase->checkState() == Qt::Checked);
        updateRouteInfo();
    }
}

void MainWindow::on_routeSetStartCursor_clicked()
{
    if(m_currRoute)
    {
        m_currRoute->setHomeLatDeg(m_mapWidget->centerLatitude());
        m_currRoute->setHomeLongDeg(m_mapWidget->centerLongitude());
        updateRouteInfo();
    }
}

void MainWindow::on_routeSetStartGPS_clicked()
{
    if(m_conn->connected())
    {
        if(m_conn->blackboard()->nav.gpsHasFix)
        {
            m_mapWidget->setCenterLatitude(m_conn->blackboard()->nav.gpsLat);
            m_mapWidget->setCenterLongitude(m_conn->blackboard()->nav.gpsLon);
            m_currRoute->setHomeLatDeg(m_mapWidget->centerLatitude());
            m_currRoute->setHomeLongDeg(m_mapWidget->centerLongitude());
        }
        else appendLogMessage("GPS has no fix. Can't set home position.");
    }
    else appendLogMessage("Not connected. Can't set home position.");
}

#include <QInputDialog>

void MainWindow::on_actionAddRoute_triggered()
{
    QString name = QInputDialog::getText(this, "Route name", "Insert the name of the new route", QLineEdit::Normal);
    name = name.trimmed();
    if(!name.isEmpty())
    {
        if(name.contains(';'))
        {
            appendLogMessage("Route name cannot contain a semicolon \";\". Doing nothing.");
            return;
        }

        DroneRoute *r = m_routes->addRoute(name, m_mapWidget->centerLongitude(), m_mapWidget->centerLatitude());
        if(r)
            ui->routesCombo->setCurrentIndex(m_routes->routeIndex(name));
        else
            appendLogMessage(QString("Could not create route \"%1\".").arg(name));
    }
    else appendLogMessage("Route name not given. Doing nothing.");
}

void MainWindow::on_routesCombo_currentIndexChanged(const QString &arg1)
{
    if(arg1=="")
        ui->routesCombo->setCurrentIndex(0);
    else
        setCurrentRoute(m_routes->route(arg1));
}

void MainWindow::on_wayAdd_clicked()
{
    if(m_currRoute)
    {
        m_currRoute->addPoint(DroneRoute::Waypoint(m_mapWidget->centerLongitude(), m_mapWidget->centerLatitude(), false));
        updateRouteInfo();
    }
}

void MainWindow::on_wayDel_clicked()
{
    if(m_currRoute)
    {
        if(ui->routeWaypointsView->currentIndex().isValid())
        {
            m_currRoute->deletePoint(ui->routeWaypointsView->currentIndex().row());
            updateRouteInfo();
        }
    }
}

void MainWindow::on_waySet_clicked()
{
    if(m_currRoute)
    {
        if(ui->routeWaypointsView->currentIndex().isValid())
        {
            int idx = ui->routeWaypointsView->currentIndex().row();
            m_currRoute->updatePoint(idx, DroneRoute::Waypoint(m_mapWidget->centerLongitude(), m_mapWidget->centerLatitude(), m_currRoute->point(idx).acquire));
            updateRouteInfo();
        }
    }
}

void MainWindow::on_wayUp_clicked()
{
    if(m_currRoute)
    {
        if(ui->routeWaypointsView->currentIndex().isValid())
        {
            int idx = ui->routeWaypointsView->currentIndex().row();
            m_currRoute->movePointUp(idx);
            updateRouteInfo();
        }
    }
}

void MainWindow::on_wayDown_clicked()
{
    if(m_currRoute)
    {
        if(ui->routeWaypointsView->currentIndex().isValid())
        {
            int idx = ui->routeWaypointsView->currentIndex().row();
            m_currRoute->movePointDown(idx);
            updateRouteInfo();
        }
    }
}

void MainWindow::on_wayClear_clicked()
{
    if(m_currRoute)
    {
        m_currRoute->clear();
        updateRouteInfo();
        appendLogMessage("Route cleared");
    }
}

#include <QMessageBox>

void MainWindow::on_actionSave_Routes_triggered()
{
    int rep = QMessageBox::question(this, "Save", "Save current set of routes?", QMessageBox::Yes|QMessageBox::Default, QMessageBox::No|QMessageBox::Escape);
    if(rep == QMessageBox::Yes)
        m_routes->save();
}

void MainWindow::on_actionSimulation_Mode_changed()
{
    bool on = ui->actionSimulation_Mode->isChecked();

    if(on)
        m_sim->setDronePosition(m_mapWidget->centerLongitude(), m_mapWidget->centerLatitude());

    if(!m_sim->setActive(on))
        ui->actionSimulation_Mode->setChecked(false);
}

void MainWindow::on_actionNavigate_triggered()
{
    if(m_conn->connected())
    {
        if(m_currRoute)
        {
            //upload route to drone
            m_conn->sendCommand(QString("$NAVROUTE,%1").arg(m_currRoute->toCommand()));

            //send begin navigation
            m_conn->sendCommand("$NAVBEGIN");
        }

    }
    else appendLogMessage("Not connected. Doing nothing.");
}

void MainWindow::on_actionSend_Command_triggered()
{
    if(m_conn->connected())
    {
        QString cmd = QInputDialog::getText(this, "Command", "Insert manual command", QLineEdit::Normal, "$GPS,$PSRF104,-26.289961,-48.837110,0,96000,418383,1920,12,1*19");
        m_conn->sendCommand(cmd);

    }
}

void MainWindow::on_actionWarmstart_GPS_triggered()
{
    if(m_conn->connected())
    {
        double lat = m_mapWidget->centerLatitude();
        double lon = m_mapWidget->centerLongitude();
        double alt = 0;

        const int secondsInWeek = 604800;
        const QDateTime gpsTimeBeginning(QDate(1980,01,06), QTime(0,0), Qt::UTC);

        long deltaSecs = (QDateTime::currentDateTimeUtc().toMSecsSinceEpoch() - gpsTimeBeginning.toMSecsSinceEpoch()) / 1000;

        deltaSecs += 17; // 17 leap seconds since

        int timeOfWeek = deltaSecs % secondsInWeek;
        int gpsWeek = deltaSecs / secondsInWeek;

        QString message("PSRF104,%1,%2,%3,0,%4,%5,12,8");

        message = message.arg(lat).arg(lon).arg(alt).arg(timeOfWeek).arg(gpsWeek);

        unsigned char checksum;
        for(int i = 0; i < message.size(); i++)
        {
            checksum ^= (unsigned char) message[i].toLatin1();
        }

        QString cmd = QString("$GPS,$%1*%2").arg(message).arg((uint)checksum,2,16);

        m_conn->sendCommand(cmd);
    }

}

void MainWindow::closeEvent(QCloseEvent * evt)
{
    save();
    m_routes->save();
}

void MainWindow::on_actionDelete_Route_triggered()
{
    if(!m_currRoute)
        return;

    if(m_routes->count() == 1)
        return;

    m_routes->removeRoute(m_currRoute->objectName());
    setCurrentRoute(m_routes->firstRoute());
}


void MainWindow::on_actionOpen_Log_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, "Select a log file", "/opt/hydrus");

    if(filename.isEmpty())
        return;

    DroneLog * log = new DroneLog(filename);
    if(log->valid())
    {

        if(!m_logTimeline)
        {
            //create
            m_logTimeline = new LogTimelineWidget(ui->logDock);
            connect(m_logTimeline, &LogTimelineWidget::trailNeedsUpdate, this, &MainWindow::updateTrail);
        }

        m_logTimeline->setLog(log);

        // change to log view mode
        ui->logDock->setWidget(m_logTimeline);

        m_mapWidget->setAndUpdateTrail(m_logTimeline->trail());
    }
    else
    {
        delete log;
    }
}

void MainWindow::updateTrail()
{
    qDebug("update");
    m_mapWidget->update();
}
