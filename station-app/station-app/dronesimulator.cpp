#include "dronesimulator.h"
#include "droneconnection.h"

DroneSimulator::DroneSimulator(QObject *parent, DroneConnection *conn) : QObject(parent)
{
    m_conn = conn;
    m_sim_timer = -1;
    memset(&m_d, 0, sizeof(m_simdata_st)); // clear sim data struct

}

bool DroneSimulator::setActive(bool active)
{
    if(active)
    {
        if(!m_conn->connected())
            return false;
        // let's turn on
        m_conn->sendCommand("$SIMON");

        //turn on timer
        m_sim_timer = startTimer(1000 / SIM_FREQ_HZ, Qt::PreciseTimer);

        // TEST setup vars
        m_d.vel_lon = 0.001;
        m_d.vel_lat = 0.0005;

    }
    else
    {
        if(m_sim_timer == -1)
            return true; // already stopped

        //kill our esteemed timer
        killTimer(m_sim_timer);

        //let's turn off
        if(!m_conn->connected())
            return true;
        m_conn->sendCommand("$SIMOFF");
    }

    return true;
}

void DroneSimulator::timerEvent(QTimerEvent *)
{
    // simulation tick
    qreal dt = 1.0 / SIM_FREQ_HZ;


    // 1. update velocities
    // drag effect
    m_d.vel_lat -= m_d.vel_lat * dt;
    m_d.vel_lon -= m_d.vel_lon * dt;

    // 2. update positions
    m_d.pos_lon += m_d.vel_lon * dt;
    m_d.pos_lat += m_d.vel_lat * dt;

    // 3. send simulation data packet
    int precision = std::numeric_limits<qreal>::max_digits10 + 2; // docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.htm
    QString cmd = QStringLiteral("$SIMDAT,%1,%2").arg(m_d.pos_lon,0,'g',precision).arg(m_d.pos_lat,0,'g',precision);
    m_conn->sendCommand(cmd);
}
