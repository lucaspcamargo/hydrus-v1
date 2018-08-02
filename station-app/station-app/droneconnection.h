#ifndef DRONECONNECTION_H
#define DRONECONNECTION_H

#include <QObject>
#include <QTcpServer>
#include <QByteArray>

class Blackboard;

class DroneConnection : public QTcpServer
{
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)

    enum MachineState
    {
        WAIT_DEF,
        READ_BB,
        READ_LOG
    };

public:
    explicit DroneConnection(QObject *parent = 0);
    virtual ~DroneConnection();

    bool connected() const
    {
        return m_connected;
    }

    Blackboard * blackboard() const {return m_bb;}
    QString host() const;

signals:

    void connectedChanged(bool connected);
    void blackboardChanged(Blackboard * bb);
    void logReceived(QString log);

public slots:
    void handleConnection();
    void handleSocketDisconnect();
    void handleSocketReadyRead();

    void disconnect();

    void sendCommand(QString command);

    void rebootDrone();
    void haltDrone();
    void abortDroneNavigation();

private:
    void setConnected(bool connected)
    {
        if (m_connected == connected)
            return;

        m_connected = connected;
        emit connectedChanged(connected);
    }

    QTcpSocket * m_drone;
    QByteArray m_buf;
    bool m_connected;
    Blackboard * m_bb;

    MachineState m_st;
};

#endif // DRONECONNECTION_H
