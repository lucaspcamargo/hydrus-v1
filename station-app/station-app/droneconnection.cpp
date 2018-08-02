#include "droneconnection.h"

#include <QTcpSocket>
#define BB_NO_TO_STRING
#include "droneblackboard.h"

DroneConnection::DroneConnection(QObject *parent) :
    QTcpServer(parent),
    m_drone(0),
    m_connected(false),
    m_st(WAIT_DEF)
{
    m_bb = new Blackboard;
    memset(m_bb, 0, sizeof(Blackboard));

    listen(QHostAddress::Any, 6666);

    connect(this, &DroneConnection::newConnection, this, &DroneConnection::handleConnection);
}

DroneConnection::~DroneConnection()
{
    delete m_bb;
}

QString DroneConnection::host() const
{
    if(m_drone)
        return QString("%1:%2").arg(m_drone->peerAddress().toString()).arg(m_drone->peerPort());

    return QString("-");
}

void DroneConnection::handleConnection()
{
    if(m_drone)
    {
        disconnect();
    }

    m_drone = nextPendingConnection();

    connect(m_drone, &QTcpSocket::disconnected, this, &DroneConnection::handleSocketDisconnect);
    connect(m_drone, &QTcpSocket::readyRead, this, &DroneConnection::handleSocketReadyRead);

    if(m_drone->bytesAvailable())
        handleSocketReadyRead();
    setConnected(true);
}

void DroneConnection::handleSocketDisconnect()
{
    m_drone->deleteLater();
    m_drone = 0;

    setConnected(false);
}

void DroneConnection::handleSocketReadyRead()
{
    QByteArray in = m_drone->readAll();

    if(!in.size())
        return;

    m_buf += in;

    while(true)
    {
        switch(m_st)
        {
        case WAIT_DEF:
            {
                if(!m_buf.size())
                    return;
                char inCh;
                inCh = m_buf.at(0);
                m_buf.remove(0,1);
                if(inCh == '$')
                {
                    m_st = READ_LOG;
                    continue;
                }
                else if(inCh == '@')
                {
                    m_st = READ_BB;
                    continue;
                }
            }
            break;
        case READ_BB:
            if(m_buf.size() >= sizeof(Blackboard))
            {
                (*m_bb) = *((Blackboard*)m_buf.data());
                m_buf.remove(0, sizeof(Blackboard));
                emit blackboardChanged(m_bb);
                m_st = WAIT_DEF;
            }
            else return;
            break;
        case READ_LOG:
            if(m_buf.indexOf('\n') != -1)
            {
                in = m_buf.left(m_buf.indexOf('\n'));
                m_buf.remove(0, m_buf.indexOf('\n')+1);
                emit logReceived(QString(in));
                m_st = WAIT_DEF;
            }
            else return;
            break;
        }
    }
}

void DroneConnection::disconnect()
{
    if(!m_connected) return;
    if(m_drone == 0) return;

    m_drone->close();

}

void DroneConnection::sendCommand(QString command)
{
    if(m_connected && m_drone)
    {
        m_drone->write(command.toLocal8Bit());
        m_drone->putChar('\n');
        m_drone->flush();
    }
}

void DroneConnection::rebootDrone()
{
    sendCommand("$REBOOT");
}

void DroneConnection::haltDrone()
{
    sendCommand("$HALT");

}

void DroneConnection::abortDroneNavigation()
{
    sendCommand("$NAVABORT");

}
