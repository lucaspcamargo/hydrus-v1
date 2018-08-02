#include "rcwidget.h"
#include "ui_rcwidget.h"

#include "droneconnection.h"
//#include "dwcontrollerhub.h"
#include "thirdparty/joystick.h"

RCWidget::RCWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RCWidget),
    m_joyTimer(-1)
{
    ui->setupUi(this);

    //m_controllerHub = new dwControllerHub(this);

    m_joy = new cJoystick();

}

RCWidget::~RCWidget()
{
    delete ui;
}


void RCWidget::sendCommand(QString cmd)
{
    if(!(m_drone && m_drone->connected())) return;
    m_drone->sendCommand(cmd);
}

#define ENSURE_CONNECTED

void RCWidget::toggleRCMode()
{
    bool on = isVisible();
    sendCommand(on? "$RCON" : "$RCOFF");
    if(on)
    {
        m_joyTimer = startTimer(33);
    }
    else
    {
        killTimer(m_joyTimer);
        m_joyTimer = -1;
    }
}



void RCWidget::on_upBtn_pressed()
{
    sendJoystickUpdate(0,1);
}

void RCWidget::on_upBtn_released()
{
    sendJoystickUpdate(0,0);
}

void RCWidget::on_downBtn_pressed()
{
    sendJoystickUpdate(0,-1);
}

void RCWidget::on_downBtn_released()
{
    sendJoystickUpdate(0,0);
}

void RCWidget::on_leftBtn_pressed()
{
    sendJoystickUpdate(-1,0);
}

void RCWidget::on_leftBtn_released()
{
    sendJoystickUpdate(0,0);
}
void RCWidget::on_rightBtn_pressed()
{
    sendJoystickUpdate(1,0);
}

void RCWidget::on_rightBtn_released()
{
    sendJoystickUpdate(0,0);
}


void RCWidget::sendJoystickUpdate(qreal x, qreal y)
{
    sendCommand(QString("$RCUP,%1,%2").arg(x).arg(y));
}

void RCWidget::timerEvent(QTimerEvent *)
{
    static qreal prevX = 0, prevY = 0;

    if(!m_joy->isActive()) return;
    joystick_position p = m_joy->joystickPosition(0);

    p.x = qAbs(p.x) > 0.1? (p.x + (p.x>0?-0.1f:0.1f)) * 1.111111111f : 0; // deadband
    p.y = qAbs(p.y) > 0.1? (p.y + (p.y>0?-0.1f:0.1f)) * 1.111111111f : 0; // deadband

    if(prevX != p.x || prevY != p.y)
        sendJoystickUpdate(p.x, p.y);

    prevX = p.x; prevY = p.y;
}
