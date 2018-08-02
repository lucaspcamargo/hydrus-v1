#include "droneroute.h"
#include <marble/GeoDataCoordinates.h>
#include <marble/GeoDataLineString.h>

#include <QDebug>

DroneRoute::DroneRoute(QString name, qreal homeLon, qreal homeLat, QObject *parent) :
    QAbstractListModel(parent),
    m_returnHome(true),
    m_homeLatDeg(homeLat),
    m_homeLongDeg(homeLon)
{
    setObjectName(name);
}
/*
#include <cmath>
typedef float nav_f_t;
#define deg2rad(x) ((x)*M_PI/180.0)


nav_f_t calcBearing_rad(nav_f_t lonA, nav_f_t latA, nav_f_t lonB, nav_f_t latB)
{
    nav_f_t lon1 = deg2rad(lonA);
    nav_f_t lat1 = deg2rad(latA);
    nav_f_t lon2 = deg2rad(lonB);
    nav_f_t lat2 = deg2rad(latB);

    nav_f_t y = sin(lon2-lon1) * cos(lat2);
    nav_f_t x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
    nav_f_t ret = (-atan2(y, x)) + (M_PI*0.5);

    if(ret < 0)
        ret += (2*M_PI);
    return ret;
}


nav_f_t fancymod(nav_f_t a, nav_f_t n)
{
    return fmod( (fmod(a, n) + n), n );
}

nav_f_t calcAngleDifference_rad(nav_f_t sourceA, nav_f_t targetA)
{
    nav_f_t a = targetA - sourceA;
    return fancymod((a + M_PI), 2*M_PI) - M_PI;
}
*/

bool DroneRoute::addPoint(const DroneRoute::Waypoint &point)
{
    beginInsertRows(QModelIndex(), m_waylist.size(), m_waylist.size());
    m_waylist.push_back(point);
    endInsertRows();

    /*float bearing = calcBearing_rad(m_homeLongDeg, m_homeLatDeg, point.lon, point.lat);
    qDebug() << bearing;
    qDebug() << calcAngleDifference_rad(M_PI, bearing);*/
}

void DroneRoute::updatePoint(int index, const DroneRoute::Waypoint &point)
{
    { m_waylist[index] = point; }
    dataChanged(createIndex(index, 0, this), createIndex(index, 0, this));
}

void DroneRoute::deletePoint(int index)
{
    if(index < 0 || index >= m_waylist.size()) return;

    beginRemoveRows(QModelIndex(), index, index);
    m_waylist.removeAt(index);
    endRemoveRows();
}

void DroneRoute::movePointUp(int index)
{
    if(index > 0 && index < m_waylist.length())
    {
        beginMoveRows(QModelIndex(), index, index, QModelIndex(), index-1);
        m_waylist.swap(index, index-1);
        endMoveRows();
    }
}

void DroneRoute::movePointDown(int index)
{
    if(index >=0 && index < (m_waylist.length()-1) )
    {
        beginMoveRows(QModelIndex(), index, index, QModelIndex(), index+2);
        m_waylist.swap(index+1, index);
        endMoveRows();
    }
}

void DroneRoute::clear()
{
    beginRemoveRows(QModelIndex(), 0, m_waylist.size()-1);
    m_waylist.clear();
    endRemoveRows();
}

qreal DroneRoute::calculateTotalDistance()
{
    qreal ret = 0;

    Marble::GeoDataCoordinates home(homeLongDeg(), homeLatDeg(),0, Marble::GeoDataCoordinates::Degree);
    Marble::GeoDataLineString path(Marble::Tessellate);

    path.append(home);

    for(auto it = m_waylist.constBegin(); it!=m_waylist.constEnd(); it++)
    {
        const Waypoint & w = *it;
        Marble::GeoDataCoordinates next(w.lon, w.lat,0, Marble::GeoDataCoordinates::Degree);

        path.append(next);
    }

    if(returnHome())
        path.append(home);

    return path.length(6371000.0); // uses earth radius
    // TODO support other planets :p

}

QVariant DroneRoute::toVariant() const
{
    QVariantMap map;
    QList<QVariant> points;

    for(Waylist::const_iterator it = m_waylist.constBegin(); it!=m_waylist.constEnd(); it++)
    {
        const Waypoint &w = *it;

        QVariantMap pointMap;
        pointMap["lon"] = w.lon;
        pointMap["lat"] = w.lat;
        pointMap["acquire"] = w.acquire;

        points.push_back(pointMap);
    }

    map["name"] = objectName();
    map["returnHome"] = m_returnHome;
    map["homeLongDeg"] = m_homeLongDeg;
    map["homeLatDeg"] = m_homeLatDeg;
    map["points"] = points;

    return QVariant(map);
}

DroneRoute *DroneRoute::fromVariantMap(QVariantMap map)
{
    DroneRoute *newRoute = new DroneRoute(map["name"].toString(), map["homeLongDeg"].toReal(), map["homeLatDeg"].toReal(), 0);
    newRoute->setReturnHome(map["returnHome"].toBool());

    QList<QVariant> points = map["points"].toList();

    foreach(QVariant point, points)
    {
        QVariantMap pointMap = point.toMap();

        qreal lon = pointMap["lon"].toReal();
        qreal lat = pointMap["lat"].toReal();
        bool acquire = pointMap["acquire"].toBool();
        Waypoint w(lon, lat, acquire);

        newRoute->addPoint(w);
    }

    return newRoute;
}

#include <limits>


/**
 * Gives the route in a string representation suitable for consumption by the drone as a command string
 */
QString DroneRoute::toCommand()
{
    int precision = std::numeric_limits<qreal>::max_digits10 + 2; // docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.htm

    QString ret = QString("%1,%2,%3,,").arg(objectName()).arg(homeLongDeg(),0,'g',precision).arg(homeLatDeg(),0,'g',precision);

    for(auto it = m_waylist.begin(); it!=m_waylist.end(); it++)
    {
        ret.append(QString("%1,%2,%3,").arg((*it).lon,0,'g',precision).arg((*it).lat,0,'g',precision).arg((*it).acquire?"A":""));
    }

    if(returnHome())
    {
        ret.append(QString("%1,%2,,").arg(homeLongDeg(),0,'g',precision).arg(homeLatDeg(),0,'g',precision));
    }

    return ret;
}

int DroneRoute::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    if(parent.isValid())
        return 0;
    return m_waylist.size();
}

#include <QColor>

QVariant DroneRoute::data(const QModelIndex &index, int role) const
{
    if(!index.isValid())
        return QVariant();

    Waypoint w = m_waylist[index.row()];
    switch (role) {
    case Qt::DisplayRole:
        return QString("%1%2").arg(Marble::GeoDataCoordinates(w.lon, w.lat, 0.0, Marble::GeoDataCoordinates::Degree).toString()).arg(w.acquire? " (ACQ)" : "");
    case Qt::BackgroundRole:
        return w.acquire? QColor("#60ff8000") : QColor("#60ffc000");
    default:
        return QVariant();
    }
}

QVariant DroneRoute::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(orientation == Qt::Horizontal)
    {
    }
    return QVariant();
}
