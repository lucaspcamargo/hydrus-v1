#include "dronemap.h"
#include <marble/ViewportParams.h>
#include "droneroute.h"

#include <QtMath>

using namespace Marble;


DroneMap::DroneMap(QWidget *parent) :
    Marble::MarbleWidget(parent),
    m_route(0),
    m_droneHasFix(false),
    m_droneLatitudeDeg(0),
    m_droneLongitudeDeg(0),
    m_droneHeadingDeg(0),
    m_trail()
{
    // Setup
    setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    setProjection(Mercator);
    setMapQualityForViewContext(HighQuality, Still);
    setMapQualityForViewContext(LowQuality, Animation);
    setShowBackground(false);
    setShowCompass(true);
    setShowGrid(false);
    setShowOverviewMap(true);
    setShowCrosshairs(false);
}

#include <marble/GeoPainter.h>
#include <marble/GeoDataLineString.h>
#include <QDebug>
#define POSITION_COLOR 10, 50, 255
#define HOME_COLOR 10, 200, 50
#define WAYPOINT_COLOR 255, 150, 10
#define WAYPOINT_ACQUIRE_COLOR 255, 50, 10
#define ROUTE_LINE_COLOR 255, 102, 10


void DroneMap::customPaint(GeoPainter *painter)
{
    GeoDataCoordinates viewCenter(centerLongitude(), centerLatitude(), 0.0, GeoDataCoordinates::Degree);

    QFont font;
    font.setPointSize(8);
    painter->setFont(font);

    painter->setPen(QColor(0,0,0));
    painter->drawRect(viewCenter, 3, 3);
    painter->drawText(viewCenter, viewCenter.toString(), 10, 15);

    if(m_route)
    {

        GeoDataCoordinates start(m_route->homeLongDeg(), m_route->homeLatDeg(), 0.0, GeoDataCoordinates::Degree);
        GeoDataCoordinates last = start;

        GeoDataLineString path(Tessellate);
        path << start;

        for(int i = 0; i < m_route->rowCount(); i++)
        {
            auto w = m_route->point(i);
            GeoDataCoordinates p(w.lon, w.lat, 0.0, GeoDataCoordinates::Degree);
            path << p;
            last = p;
        }

        painter->setPen(QColor(ROUTE_LINE_COLOR));
        painter->drawPolyline(path);

        painter->setPen(QColor(HOME_COLOR).darker());
        painter->setBrush(QColor(HOME_COLOR));
        painter->drawRect(start, 10, 10);
        painter->drawText(start, start.toString(), 10, 15);


        for(int i = 0; i < m_route->rowCount(); i++)
        {
            auto w = m_route->point(i);
            GeoDataCoordinates p(w.lon, w.lat, 0.0, GeoDataCoordinates::Degree);

            QColor c;
            c = (w.acquire? QColor(WAYPOINT_ACQUIRE_COLOR) : QColor(WAYPOINT_COLOR));
            int wh = w.acquire? 9 : 6;

            painter->setPen(c.darker());
            painter->setBrush(c);
            painter->drawRect(p, wh, wh);
            if(w.acquire)
                painter->drawText(p, "A", 10, 15);
        }

        if(m_route->returnHome())
        {
            GeoDataLineString backJourney(Tessellate);
            backJourney << last << start;
            painter->setPen(QPen(QColor(ROUTE_LINE_COLOR), 1, Qt::DashLine));
            painter->setOpacity(0.75);
            painter->drawPolyline(backJourney);
            painter->setOpacity(1.0);
        }

    }

    if(m_trail)
    {

        // lets draw some lines on the map

        qreal min = 9999999999;
        qreal max = -9999999999;

        foreach(const GeoDatapoint &p, (*m_trail))
        {
            if(p.value > max) max = p.value;
            if(p.value < min) min = p.value;
        }

        qDebug() << min << " " << max;

        qreal delta = max - min;

        for(int i = 0; i < m_trail->size() - 1; i++)
        {

            const GeoDatapoint &aP = (*m_trail)[i];
            const GeoDatapoint &bP = (*m_trail)[i+1];

            // calculate color
            qreal alpha = (aP.value - min)/delta;
            QColor color = QColor::fromHsvF(alpha*0.8, 1, 1);

            // create the line
            GeoDataLineString line(Tessellate);
            GeoDataCoordinates a(aP.lon, aP.lat, 0.0, GeoDataCoordinates::Degree);
            GeoDataCoordinates b(bP.lon, bP.lat, 0.0, GeoDataCoordinates::Degree);

            // test moving
            //GeoDataCoordinates a((aP.lon - m_route->homeLongDeg())*2 + centerLongitude(), (aP.lat - m_route->homeLatDeg())*2 + centerLatitude(), 0.0, GeoDataCoordinates::Degree);
            //GeoDataCoordinates b((bP.lon - m_route->homeLongDeg())*2 + centerLongitude(), (bP.lat - m_route->homeLatDeg())*2 + centerLatitude(), 0.0, GeoDataCoordinates::Degree);

            line << a << b;

            //paint line
            painter->setPen(color);
            painter->drawPolyline(line);
        }

    }

    if(m_droneHasFix)
    {
        GeoDataCoordinates dronePos(m_droneLongitudeDeg, m_droneLatitudeDeg, 0.0, GeoDataCoordinates::Degree);

        painter->setPen(QColor(POSITION_COLOR).darker());
        painter->setBrush(QColor(POSITION_COLOR));
        painter->drawRoundRect(dronePos, 8, 8, 4, 4);
        painter->drawText(dronePos, dronePos.toString(), 10, 15);

        const float distance = 11.0f;

        float deltaX = distance * qCos(m_droneHeadingDeg  * M_PI / 180);
        float deltaY = - distance * qSin(m_droneHeadingDeg  * M_PI / 180);

        painter->translate(deltaX, deltaY);
        painter->drawRect(dronePos, 2, 2);
    }

}
