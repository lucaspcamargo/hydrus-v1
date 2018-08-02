#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


class DroneConnection;
class DroneMap;
class DroneRoute;
class DroneRouteSet;
class ServerConfigDialog;
class LogTimelineWidget;

class Blackboard;
class DroneSimulator;
class DroneLog;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void handleConnectedChanged(bool conn);
    void appendLogMessage(const QString& text);
    void zoomCh(int);
    void blackboardUpdate (Blackboard * bb );

private slots:
    void on_actionDisconnect_triggered();

    void on_logClearBtn_clicked();

    void on_logSaveBtn_clicked();

    void on_actionServer_Setup_triggered();

    void on_actionAbout_triggered();

    void on_actionCenter_Map_triggered();

    void on_routeReturnToBase_clicked();

    void on_routeSetStartCursor_clicked();

    void on_routeSetStartGPS_clicked();

    void on_actionAddRoute_triggered();

    void on_routesCombo_currentIndexChanged(const QString &arg1);

    void on_wayAdd_clicked();

    void on_wayDel_clicked();

    void on_waySet_clicked();

    void on_wayUp_clicked();

    void on_wayDown_clicked();

    void on_wayClear_clicked();

    void on_actionSave_Routes_triggered();

    void on_actionSimulation_Mode_changed();

    void on_actionNavigate_triggered();

    void on_actionSend_Command_triggered();

    void on_actionWarmstart_GPS_triggered();

    void on_actionDelete_Route_triggered();

    void on_actionOpen_Log_triggered();

    void updateTrail();

protected:
    void closeEvent(QCloseEvent *evt);

private:
    void setCurrentRoute(DroneRoute * r);
    void updateRouteInfo();

    void save();
    void restore();

    Ui::MainWindow *ui;
    DroneMap * m_mapWidget;
    DroneConnection * m_conn;
    ServerConfigDialog * m_config;
    DroneRoute * m_currRoute;
    DroneRouteSet *m_routes;

    DroneSimulator * m_sim;
    DroneLog * m_log;
    LogTimelineWidget * m_logTimeline;
};

#endif // MAINWINDOW_H
