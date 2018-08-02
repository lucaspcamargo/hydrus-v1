#ifndef DRONELOG_H
#define DRONELOG_H

#include <QObject>
#include <QMap>

class Blackboard;

class DroneLog : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool valid READ valid CONSTANT)
    Q_PROPERTY(bool complete READ complete CONSTANT)
    Q_PROPERTY(quint64 timestampMin READ timestampMin CONSTANT)
    Q_PROPERTY(quint64 timestampMax READ timestampMax CONSTANT)
public:
    typedef quint64 timestamp_t;

    explicit DroneLog(QString filename, QObject *parent = 0);


    bool valid() const
    {
        return m_valid;
    }

    bool complete() const
    {
        return m_complete;
    }

    quint64 timestampMin() const
    {
        return m_timestampMin;
    }

    quint64 timestampMax() const
    {
        return m_timestampMax;
    }

    const QMap<timestamp_t, Blackboard*>& bbTicks()
    {
        return m_blackboardTicks;
    }

    const QMap<timestamp_t, QString>& logMsgs()
    {
        return m_logMessages;
    }

signals:

public slots:

private:

    QMap<timestamp_t, Blackboard*> m_blackboardTicks;
    QMap<timestamp_t, QString> m_logMessages;

    bool m_valid;
    bool m_complete;
    quint64 m_timestampMin;
    quint64 m_timestampMax;
};

#endif // DRONELOG_H
