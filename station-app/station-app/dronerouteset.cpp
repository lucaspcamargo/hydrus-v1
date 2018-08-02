#include "dronerouteset.h"
#include "droneroute.h"

DroneRouteSet::DroneRouteSet(QObject *parent)
    :QAbstractListModel(parent)
{

}

DroneRoute *DroneRouteSet::addRoute(QString name, qreal homeLon, qreal homeLat)
{
    auto keys = m_routes.keys();
    if(keys.contains(name))
        return 0;

    // we need to find out where the item will end up
    keys.insert(0, name);
    qSort(keys);

    int newIndex = keys.indexOf(name);

    // ok, now tell the model about the position
    beginInsertRows(QModelIndex(), newIndex, newIndex);

    DroneRoute *r = new DroneRoute(name, homeLon, homeLat, this);
    m_routes.insert(name, r);

    endInsertRows();

    return r;
}

bool DroneRouteSet::removeRoute(QString name)
{
    if(!m_routes.contains(name))
        return false;

    beginResetModel();

    m_routes.remove(name);

    endResetModel();
    return true;
}

int DroneRouteSet::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    if(parent.isValid())
        return 0;
    return m_routes.size();
}

QVariant DroneRouteSet::data(const QModelIndex &index, int role) const
{
    if(!index.isValid())
        return QVariant();

    DroneRoute *r = m_routes[m_routes.keys()[index.row()]];
    switch (role) {
    case Qt::DisplayRole:
        return r->objectName();
    default:
        return QVariant();
    }

}

#include <QVariantMap>
#include <QSettings>

void DroneRouteSet::save()
{
    QSettings settings;
    QVariantMap routes;

    foreach(const DroneRoute *r, m_routes)
    {
        routes[r->objectName()] = r->toVariant();
    }

    settings.beginGroup("Routes");
    settings.setValue("map", routes);
    settings.endGroup();

}


void DroneRouteSet::restore()
{
    beginResetModel();

    QSettings settings;
    QVariantMap routes;
    settings.beginGroup("Routes");
    routes = settings.value("map").toMap();
    settings.endGroup();

    foreach(QString key, routes.keys())
    {
        DroneRoute *route = DroneRoute::fromVariantMap(routes[key].toMap());
        m_routes.insert(route->objectName(), route);
    }

    endResetModel();

}
