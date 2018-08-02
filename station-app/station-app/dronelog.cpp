#include "dronelog.h"
#include "droneblackboard.h"

#include <QIODevice>
#include <QFile>
#include <QDebug>

DroneLog::DroneLog(QString filename, QObject *parent) : QObject(parent), m_valid(false), m_complete(false)
{
    QFile file(filename);

    file.open(QFile::ReadOnly);

    if(!file.isOpen())
        return; // sad


    bool cont = true;

    while(!file.atEnd() && cont)
    {
        char c;
        file.getChar(&c);

        switch (c) {
        case '@':
        {
            //blackboard
            quint64 timestamp;
            quint64 read = file.read((char*)&timestamp, 8);
            if(read == 8)
            {
                // could read timestamp
                Blackboard bb;
                read = file.read((char*)&bb, sizeof(Blackboard));
                if(read == sizeof(Blackboard))
                {
                    // could read blackboard
                    Blackboard * newBB = new Blackboard();
                    memcpy(newBB, &bb, sizeof(Blackboard));

                    m_blackboardTicks[timestamp] = newBB;

                } else cont = false;
            }
            else
            {
                cont = false; // incomplete
            }
            break;
        }
        case '$':
        {
            //log message
            quint64 timestamp;
            quint64 read = file.read((char*)&timestamp, 8);
            if(read == 8)
            {
                // could read timestamp
                quint32 msgSize;
                read = file.read((char*)&msgSize, sizeof(quint32));
                if(read == sizeof(quint32))
                {
                    // could read size
                    char buf[msgSize + 1];
                    buf[msgSize] = '\0';
                    read = file.read(buf, msgSize);

                    if(read == msgSize)
                    {
                        // could read message
                        QString msg(buf);
                        m_logMessages[timestamp] = msg;
                    }


                } else cont = false;
            }
            else
            {
                cont = false; // incomplete
            }
            break;
        }
            break;
        case 'E':
            //end of log
            m_complete = true;
            cont = false;
            break;
        default:
            cont = false; // corrupted garbage ahead
            break;
        }
    }

    m_valid = true;

    m_timestampMin = qMin(m_blackboardTicks.keys()[0], m_logMessages.keys()[0]);
    m_timestampMax = qMax(m_blackboardTicks.keys()[m_blackboardTicks.size()-1], m_logMessages.keys()[m_logMessages.size()-1]);

    qDebug() << "Loaded log file";
    qDebug() << "valid: " << m_valid;
    qDebug() << "complete: " << m_complete;
    qDebug() << "num BB ticks: " << m_blackboardTicks.size();
    qDebug() << "num log msgs: " << m_logMessages.size();
    qDebug() << "timestampMin: " << m_timestampMin;
    qDebug() << "timestampMax: " << m_timestampMax;

}

