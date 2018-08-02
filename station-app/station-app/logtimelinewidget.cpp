#include "logtimelinewidget.h"
#include "ui_logtimelinewidget.h"

#include "dronelog.h"
#define BB_NO_TO_STRING
#include "droneblackboard.h"

LogTimelineWidget::LogTimelineWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LogTimelineWidget),
    m_log(0)
{
    ui->setupUi(this);
}

LogTimelineWidget::~LogTimelineWidget()
{
    delete ui;
}

void LogTimelineWidget::setLog(DroneLog *log)
{
    m_log = log;

    if(log)
    {
        ui->minTime->setText(timestampToString(log->timestampMin()));
        ui->maxTime->setText(timestampToString(log->timestampMax()));

        //force update
        on_timeSlider_valueChanged(ui->timeSlider->value());
    }
    else
    {

    }
}

QString LogTimelineWidget::timestampToString(DroneLog::timestamp_t t)
{
    quint64 secs = t / 1000000;
    quint64 micros = t % 1000000;

    // TODO make it readable
    return QString("%1").arg(t);
}

#include <QDebug>

void LogTimelineWidget::on_timeSlider_valueChanged(int value)
{
    if(!m_log) return;

    double alpha = ((double)value)/ui->timeSlider->maximum();
    qDebug() << alpha;
    m_curr = m_log->timestampMin() + (m_log->timestampMax() - m_log->timestampMin()) * alpha;

    ui->currTime->setText(timestampToString(m_curr));

    rebuildTrail();


}

typedef QMap<DroneLog::timestamp_t, Blackboard*> bbmap_t;

void LogTimelineWidget::rebuildTrail()
{
    m_trail.clear();

    const bbmap_t& ticks = m_log->bbTicks();

    QList<DroneLog::timestamp_t> ts = ticks.keys();

    double lastLon = 999999;
    double lastLat = 999999;

    foreach(DroneLog::timestamp_t t, ts)
    {
        if(t > m_curr) break;

        const Blackboard * bb = ticks[t];

        float lon = bb->nav.gpsLon;
        float lat = bb->nav.gpsLat;

        if(((lon == 0)&&(lat == 0)) || (lon == lastLon && lat == lastLat))
            continue; // ignore repeated points

        lastLon = lon;
        lastLat = lat;


        float value;


        switch (ui->datapointCombo->currentIndex()) {
        case 0:
            value = bb->sensors.waterTemp;
            break;
        case 1:
            value = bb->sensors.waterPH;
            break;
        case 2:
            value = bb->sensors.waterTurb;
            break;
        case 3:
            value = bb->nav.geoHeading;
            break;
        default:
            break;
        }

        DroneMap::GeoDatapoint datapoint;

        datapoint.lon = lon;
        datapoint.lat = lat;
        datapoint.value = value;

        m_trail.append(datapoint);
    }

    emit trailNeedsUpdate();
}




void LogTimelineWidget::on_datapointCombo_currentIndexChanged(int index)
{
    rebuildTrail();
}
