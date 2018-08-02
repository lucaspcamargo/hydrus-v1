#pragma once

#include "geomag.h"
#include "util.h"

#include "nav-waypoints.h"

#define deg2rad(x) ((x)*M_PI/180.0)

typedef float nav_f_t;

class NavControllerCfg 
{
public :
    static const nav_f_t earthRadius_m = 6371000;
    
    static const nav_f_t rangeToleranceRadius_m = 7;
    static const nav_f_t rangeToleranceExitRadius_m = 12;
    static const nav_f_t headingTolerance_rad = deg2rad(10); // 10 degrees in radians
    
#define SPEED_FACTOR 0.3
    
    static const nav_f_t speedFull = (0.5 * SPEED_FACTOR);
    static const nav_f_t speedMedium = (0.25 * SPEED_FACTOR);
    static const nav_f_t speedLow = (0.1 * SPEED_FACTOR);
    
    static const nav_f_t speedMediumDistance = 20; //meters
    static const nav_f_t speedLowDistance = 5; //meters
    
    
    static const nav_f_t timeForAlign = 3; // 3 seconds
    static const nav_f_t timeForArrival = 5; // 5 seconds
};

class NavController
{
public:

    typedef NavControllerCfg cfg;
    
    NavController()
#if DRONE_HAS_GEOMAG
    :m_geomag(0)
#endif
    {
#ifdef DRONE_HAS_GEOMAG
        Util::log("NavControl", "Initializing geomagnetic model");
       // m_geomag = new GeoMag();
#endif
    }
    
    void setup(Waypoints &wps)
    {
        m_wps = &wps;
        m_wp = 1;
        
        BB.nav.state = NS_ALIGN;
        BB.sys.state = SS_NAVIGATING;
    }
    
    void clearActuation()
    {
        BB.nav.motor.spdL = 0;
        BB.nav.motor.spdR = 0;
        BB.nav.motor.enabled = false;
    }
    
    bool withinRange(nav_f_t lon, nav_f_t lat, bool exit = false)
    {
        return calcDistance_m(lon, lat, BB.nav.gpsLon, BB.nav.gpsLat) < (exit? cfg::rangeToleranceRadius_m : cfg::rangeToleranceExitRadius_m);
    }
    
    /**
     * Haversine and Forward Azimuth
     * from http://www.movable-type.co.uk/scripts/latlong.html 
     * 
     * These arguments are taken in degrees
     */
    nav_f_t calcDistance_m(nav_f_t lonA, nav_f_t latA, nav_f_t lonB, nav_f_t latB) 
    {
        const nav_f_t R = cfg::earthRadius_m; // metres
        nav_f_t lat1 = deg2rad(latA);
        nav_f_t lat2 = deg2rad(latB);
        nav_f_t delta_lat = deg2rad(latB-latA);
        nav_f_t delta_lon = deg2rad(lonB-lonA);

        nav_f_t a = sin(delta_lat/2) * sin(delta_lat/2) +
                cos(lat1) * cos(lat2) *
                sin(delta_lon/2) * sin(delta_lon/2);
        nav_f_t c = 2 * atan2(sqrt(a), sqrt(1-a));

        return R * c;
    }
    /*
     * These arguments are taken in degrees
     */
    nav_f_t calcDistanceDumb_m(nav_f_t lonA, nav_f_t latA, nav_f_t lonB, nav_f_t latB)
    {
        
        std::stringstream ss;
        ss << lonA << " " << latA << " ";
        ss << lonB << " " << latB << " ";
        Util::log("NAVCONTROL", ss.str().c_str());
        
        nav_f_t delta_lat = deg2rad(latB-latA);
        nav_f_t delta_lon = deg2rad(lonB-lonA);
        
        return cfg::earthRadius_m * (sqrt(delta_lat*delta_lat + delta_lon*delta_lon));
    }
    
    /*
     * These arguments are taken in radians
     */
    nav_f_t calcBearing_rad(nav_f_t lon1, nav_f_t lat1, nav_f_t lon2, nav_f_t lat2)
    {

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
    
    /**
     * Step navigation controller
     * @returns true if still navigating, false if not
     */
    bool step(nav_f_t time_delta)
    {
        // state timekeeping
        
        static NavigationState prev_state = (NavigationState) -1;
        static nav_f_t state_time  = 0;
        
        if(BB.nav.state == prev_state)
        {
            state_time += time_delta;
        }
        else
        {
            state_time = 0;
            prev_state = BB.nav.state;
        }
        
        // get gps state and position
        bool gpsFix = BB.nav.gpsHasFix;
        nav_f_t gpsLon = BB.nav.gpsLon, 
                gpsLat = BB.nav.gpsLat;
        
        // get destination angle positions
        nav_f_t destLon = ((*m_wps)[m_wp].longitude);
        nav_f_t destLat = ((*m_wps)[m_wp].latitude);
                
        // calculate distance
        nav_f_t distance = calcDistance_m(gpsLon, gpsLat, destLon, destLat);
        
        // get current heading
        nav_f_t magneticHeading = BB.sensors.imuHeading;
        
#if DRONE_COMPASS_FIXED_DECLINATION
        nav_f_t correctedHeading = magneticHeading + (DRONE_COMPASS_FIXED_DECLINATION_VALUE);
#else
        // TODO correct using WMM
        nav_f_t correctedHeading = magneticHeading;        
#endif
        
        nav_f_t heading = deg2rad(correctedHeading);
        nav_f_t angleToDestination = calcBearing_rad(deg2rad(gpsLon), deg2rad(gpsLat), deg2rad(destLon), deg2rad(destLat));
        
        // calculate angle difference
        nav_f_t theta = calcAngleDifference_rad(heading, angleToDestination);
        
        
        /*
         *  Now save some navigation data to the blackboard
         */
        
        
        BB.nav.geoHeading = correctedHeading;
        BB.nav.distanceFromNextWaypoint = distance;
        
        if(!gpsFix)
        {
            Util::log("NavControl", "Lost GPS fix. Skipping control, disabling motors.", Util::LS_CRITICAL);
            BB.nav.motor.enabled = false;
            return true;
        }
        
        switch(BB.nav.state)
        {
            case NS_NOT_NAVIGATING:
                return false; // well that was easy
            
            
            case NS_ALIGN:           // point vessel towards waypoint
            {
                // compare delta
                if(abs(theta) < cfg::headingTolerance_rad)
                {
                    BB.nav.state = NS_ALIGN_WAIT;
                }
                else
                {
                    // angle controller
                    bool goLeft = theta > 0;
                    nav_f_t pulseTime = fmod( state_time, 2);
                    
                    if(false)//pulseTime > 1) 
                    {
                        // jolt pause time
                        BB.nav.motor.spdL = 0;
                        BB.nav.motor.spdR = 0;
                    }
                    else // give it a jolt
                    {
                        if(goLeft)
                        {
                            BB.nav.motor.spdR = cfg::speedMedium;
                            BB.nav.motor.spdL = 0;
                        }
                        else
                        {
                            BB.nav.motor.spdL = cfg::speedMedium;
                            BB.nav.motor.spdR = 0;
                        }
                    }
#if !DRONE_NAV_SUPRESS_MOTORS 
                    BB.nav.motor.enabled = true;
#endif
                }
                
                /*
      std::stringstream ss;
      ss << "ALIGN " << heading << " ";
      ss << angleToDestination << " ";
      ss << theta << " ";
      // TODO write location name and distance
      Util::log("NavControl", ss.str().c_str());*/
                
                return true;
            }
            
            case NS_ALIGN_WAIT:      // vessel is pointing towards waypoint
            {                
                BB.nav.motor.enabled = false;
                    
                if(state_time > cfg::timeForAlign)
                {
                    BB.nav.state = NS_TRAVERSE;
                    clearActuation();
                }
                
                if(abs(theta) > cfg::headingTolerance_rad)
                {
                    BB.nav.state = NS_ALIGN;                    
                }
                
                return true;
            }
            
                
            case NS_TRAVERSE:        // vessel moves towards waypoint
            {
                nav_f_t speed = distance < cfg::speedMediumDistance? (distance < cfg::speedLowDistance? cfg::speedLow : cfg::speedMedium) : cfg::speedFull;
                
                // update actuation
                if(2*fabs(theta) > M_PI)
                {
                    // behind me
                    BB.nav.motor.spdL = cfg::speedFull;
                    BB.nav.motor.spdR = 0;
                }
                else
                {
                    // sane thing
                    BB.nav.motor.spdL = 0.0 * speed;
                    BB.nav.motor.spdR = 0.0 * speed;
                    
                    if(theta > 0) // a bit more to the left
                            BB.nav.motor.spdR += (0.5 + 0.5*abs(sin(theta))) * speed;
                    else          // a bit more to the right
                            BB.nav.motor.spdL += (0.5 + 0.5*abs(sin(theta))) * speed;
                }
                
#if !DRONE_NAV_SUPRESS_MOTORS 
                BB.nav.motor.enabled = true;
#endif
                
                // if within radius
                if(distance < cfg::rangeToleranceRadius_m)
                {
                    BB.nav.state = NS_ARRIVAL_WAIT;
                    clearActuation();
                }
                
                
                return true;
            }
                
            case NS_ARRIVAL_WAIT:    // vessel in in radius of waypoint
                
                // if drifted to exit radius, realign, must be close
                if(distance > cfg::rangeToleranceExitRadius_m)
                {
                    BB.nav.state = NS_ALIGN;
                    return true;
                }
                
                // if waited enough time, we are in the spot
                if(state_time > cfg::timeForArrival)
                {
                    BB.nav.state = NS_ACQUIRE;
                }
                
                
            case NS_ACQUIRE:         // vessel is acquiring data
                
                if(state_time > 1) // wait a sec
                {
                    // is there another waypoint?
                    if(m_wp == (m_wps->size() - 1))
                    {
                        // end of the line
                        BB.nav.state = NS_NOT_NAVIGATING;
                        return false;
                    }
                    else
                    {
                        // advance to next waypoint
                        Util::logf("NavControl", "Reached point %d", Util::LS_INFO, m_wp+1);
                        m_wp ++;
                        Util::logf("NavControl", "Going for point %d of ", Util::LS_INFO, m_wp+1, (*m_wps).size());
                        
                        // realign
                        BB.nav.state = NS_ALIGN;
                    }
                }
                
                return true;
                
            case NS_HOMING_EMERGENCY: // vessel is returning home because something is not right   
                
                // TODO
                // set home as next point, override waypoint list
                // set mode to align
                
                return true;
            
        }
        
        // unhandled state, we halt navigation then
        return false;
    }
    
private:
#if DRONE_HAS_GEOMAG
    GeoMag *m_geomag;
#endif
    Waypoints *m_wps;
    int m_wp;

};