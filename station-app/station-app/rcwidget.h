#ifndef RCWIDGET_H
#define RCWIDGET_H

#include <QWidget>

class DroneConnection;
//class dwControllerHub;
class cJoystick;

namespace Ui {
class RCWidget;
}

class RCWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RCWidget(QWidget *parent = 0);
    ~RCWidget();

    void setDroneConnection(DroneConnection * d) {m_drone = d;}
    void toggleRCMode();

private slots:
    void sendCommand(QString cmd);

    void on_upBtn_pressed();
    void on_upBtn_released();
    void on_downBtn_pressed();
    void on_downBtn_released();
    void on_leftBtn_pressed();
    void on_leftBtn_released();
    void on_rightBtn_pressed();
    void on_rightBtn_released();


private:
    void sendJoystickUpdate(qreal x, qreal y);

    Ui::RCWidget *ui;
    DroneConnection * m_drone;
   //dwControllerHub * m_controllerHub;
    cJoystick * m_joy;
    int m_joyTimer;

    // QObject interface
protected:
    void timerEvent(QTimerEvent *);
};

#endif // RCWIDGET_H
