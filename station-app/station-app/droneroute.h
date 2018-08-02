#ifndef DRONEROUTE_H
#define DRONEROUTE_H

#include <QObject>
#include <QAbstractListModel>
#include <QList>
#include <QVariant>

class DroneRoute : public QAbstractListModel
{
    Q_OBJECT
    Q_PROPERTY(bool returnHome READ returnHome WRITE setReturnHome NOTIFY returnHomeChanged)
    Q_PROPERTY(qreal homeLatDeg READ homeLatDeg WRITE setHomeLatDeg NOTIFY homeLatDegChanged)
    Q_PROPERTY(qreal homeLongDeg READ homeLongDeg WRITE setHomeLongDeg NOTIFY homeLongDegChanged)

    bool m_returnHome;
    qreal m_homeLatDeg;
    qreal m_homeLongDeg;

public:

    struct Waypoint
    {
        Waypoint(qreal lg, qreal lt, bool acq):
            lon(lg), lat(lt), acquire(acq)
        {
        }

        qreal lon;
        qreal lat;
        bool acquire;
    };

    typedef QList<Waypoint> Waylist;

    explicit DroneRoute(QString name, qreal homeLon, qreal homeLat, QObject *parent = 0);

    bool addPoint(const Waypoint &point);
    Waypoint point(int index) { return m_waylist[index]; }
    void updatePoint(int index, const Waypoint &point);
    void deletePoint(int index);
    void movePointUp(int index);
    void movePointDown(int index);

    void clear();

    qreal calculateTotalDistance();

    QVariant toVariant() const;

    static DroneRoute* fromVariantMap(QVariantMap);

signals:

void returnHomeChanged(bool returnHome);

void homeLatDegChanged(qreal homeLatDeg);

void homeLongDegChanged(qreal homeLongDeg);

public slots:
void setReturnHome(bool returnHome)
{
    if (m_returnHome == returnHome)
        return;

    m_returnHome = returnHome;
    emit returnHomeChanged(returnHome);
}

// QAbstractItemModel interface
void setHomeLatDeg(qreal homeLatDeg)
{
    if (m_homeLatDeg == homeLatDeg)
    return;

m_homeLatDeg = homeLatDeg;
emit homeLatDegChanged(homeLatDeg);
}

void setHomeLongDeg(qreal homeLongDeg)
{
    if (m_homeLongDeg == homeLongDeg)
    return;

m_homeLongDeg = homeLongDeg;
emit homeLongDegChanged(homeLongDeg);
}

public:

QString toCommand();

int rowCount(const QModelIndex &parent = QModelIndex()) const;
QVariant data(const QModelIndex &index, int role) const;
QVariant headerData(int section, Qt::Orientation orientation, int role) const;
qreal homeLatDeg() const
{
    return m_homeLatDeg;
}
qreal homeLongDeg() const
{
    return m_homeLongDeg;
}
bool returnHome() const
{
    return m_returnHome;
}
private:
    Waylist m_waylist;
};

#endif // DRONEROUTE_H
