#ifndef DRONEMAP_H
#define DRONEMAP_H

#include <marble/MarbleWidget.h>
#include <QVector>

class DroneRoute;

class DroneMap : public Marble::MarbleWidget
{

public:

    // types for plotting trails in the map
    struct GeoDatapoint
    {
        qreal lon;
        qreal lat;
        qreal value;
    };

    typedef QVector<GeoDatapoint> GeoDataTrail;

private:
    Q_OBJECT
    Q_PROPERTY(DroneRoute* route READ route WRITE setRoute NOTIFY routeChanged)
    Q_PROPERTY(bool droneHasFix READ droneHasFix WRITE setDroneHasFix NOTIFY droneHasFixChanged)
    Q_PROPERTY(qreal droneLatitudeDeg READ droneLatitudeDeg WRITE setDroneLatitudeDeg NOTIFY droneLatitudeDegChanged)
    Q_PROPERTY(qreal droneLongitudeDeg READ droneLongitudeDeg WRITE setDroneLongitudeDeg NOTIFY droneLongitudeDegChanged)
    Q_PROPERTY(qreal droneHeadingDeg READ droneHeadingDeg WRITE setDroneHeadingDeg NOTIFY droneHeadingDegChanged)

    DroneRoute* m_route;

    bool m_droneHasFix;

qreal m_droneLatitudeDeg;

qreal m_droneLongitudeDeg;

qreal m_droneHeadingDeg;

GeoDataTrail *m_trail;

public:


    explicit DroneMap(QWidget *parent = 0);


    virtual void customPaint(Marble::GeoPainter* painter);

DroneRoute* route() const
{
    return m_route;
}

bool droneHasFix() const
{
    return m_droneHasFix;
}

qreal droneLatitudeDeg() const
{
    return m_droneLatitudeDeg;
}

qreal droneLongitudeDeg() const
{
    return m_droneLongitudeDeg;
}

qreal droneHeadingDeg() const
{
    return m_droneHeadingDeg;
}

signals:

void routeChanged(DroneRoute* route);

void droneHasFixChanged(bool droneHasFix);

void droneLatitudeDegChanged(qreal droneLatitudeDeg);

void droneLongitudeDegChanged(qreal droneLongitudeDeg);

void droneHeadingDegChanged(qreal droneHeadingDeg);

public slots:
void setRoute(DroneRoute* route)
{
    if (m_route == route)
        return;

    m_route = route;
    emit routeChanged(route);

    update();
}
void setDroneHasFix(bool droneHasFix)
{
    if (m_droneHasFix == droneHasFix)
        return;

    m_droneHasFix = droneHasFix;
    emit droneHasFixChanged(droneHasFix);
    update();
}
void setDroneLatitudeDeg(qreal droneLatitudeDeg)
{
    if (m_droneLatitudeDeg == droneLatitudeDeg)
        return;

    m_droneLatitudeDeg = droneLatitudeDeg;
    emit droneLatitudeDegChanged(droneLatitudeDeg);
    update();
}
void setDroneLongitudeDeg(qreal droneLongitudeDeg)
{
    if (m_droneLongitudeDeg == droneLongitudeDeg)
        return;

    m_droneLongitudeDeg = droneLongitudeDeg;
    emit droneLongitudeDegChanged(droneLongitudeDeg);
    update();
}
void setDroneHeadingDeg(qreal droneHeadingDeg)
{
    if (m_droneHeadingDeg == droneHeadingDeg)
        return;

    m_droneHeadingDeg = droneHeadingDeg;
    emit droneHeadingDegChanged(droneHeadingDeg);
}

void setAndUpdateTrail(GeoDataTrail *trail)
{
    m_trail = trail;
    update();
}
};

#endif // DRONEMAP_H
