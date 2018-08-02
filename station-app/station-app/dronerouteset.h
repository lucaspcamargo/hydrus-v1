#ifndef DRONEROUTESET_H
#define DRONEROUTESET_H

#include <QObject>
#include <QAbstractListModel>
#include <QMap>

class DroneRoute;

class DroneRouteSet : public QAbstractListModel
{
    Q_OBJECT
public:
    DroneRouteSet(QObject * parent);

    DroneRoute * addRoute(QString name, qreal homeLong, qreal homeLat);
    bool removeRoute(QString name);

    DroneRoute * firstRoute() { return m_routes[m_routes.keys()[0]]; }
    DroneRoute * route(QString name) { return m_routes[name]; }
    int routeIndex(QString name) { return m_routes.keys().indexOf(name); }
    int routeIndex(DroneRoute * route) { return m_routes.values().indexOf(route); }

    int count() {return m_routes.count();}

    int rowCount(const QModelIndex &parent) const;
    QVariant data(const QModelIndex &index, int role) const;

    void save();
    void restore();

private:
    typedef QMap<QString, DroneRoute*> Routes;
    Routes m_routes;
};

#endif // DRONEROUTESET_H
