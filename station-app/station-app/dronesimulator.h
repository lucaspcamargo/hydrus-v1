#ifndef DRONESIMULATOR_H
#define DRONESIMULATOR_H

#include <QObject>

class DroneConnection;

class DroneSimulator : public QObject
{
    Q_OBJECT
public:

    const static int SIM_FREQ_HZ = 25;

    explicit DroneSimulator(QObject *parent, DroneConnection * conn);

    /**
     * @brief setActive
     * @param active
     * @return True on success.
     */
    bool setActive(bool active);

    void setDronePosition(qreal lon, qreal lat)
    {
        m_d.pos_lon = lon;
        m_d.pos_lat = lat;
    }

signals:

public slots:

private:
    DroneConnection *m_conn;

    struct m_simdata_st {
        qreal pos_lon;
        qreal pos_lat;
        qreal pos_orient;

        qreal vel_lon;
        qreal vel_lat;
    } m_d;

    int m_sim_timer;

    // QObject interface
protected:
    void timerEvent(QTimerEvent *);
};

#endif // DRONESIMULATOR_H
