#ifndef LOGTIMELINEWIDGET_H
#define LOGTIMELINEWIDGET_H

#include <QWidget>

#include "dronelog.h"
#include "dronemap.h"

namespace Ui {
class LogTimelineWidget;
}

class LogTimelineWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LogTimelineWidget(QWidget *parent = 0);
    ~LogTimelineWidget();

    void setLog(DroneLog * log);

    static QString timestampToString(DroneLog::timestamp_t t);

    DroneMap::GeoDataTrail * trail() { return &m_trail; }

signals:
    void trailNeedsUpdate();

private slots:
    void on_timeSlider_valueChanged(int value);

    void on_datapointCombo_currentIndexChanged(int index);

private:
    void rebuildTrail();

    Ui::LogTimelineWidget *ui;
    DroneLog *m_log;
    DroneLog::timestamp_t m_curr;
    DroneMap::GeoDataTrail m_trail;
};

#endif // LOGTIMELINEWIDGET_H
