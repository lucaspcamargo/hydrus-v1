#pragma once

#include "geomag/wmm-wrapper.h"

/**
 *  @brief Wrapper for NOAA's World Magnetic Model 2015
 *  @note WMM support code is released under public domain
 */

class GeoMag
{
public:
    GeoMag()
    {
        wmm_init(&wmm);
        
    }
    
    float declination(float lon_deg, float lat_deg, float alt_km = 0.0f)
    {
        return wmm_declination_deg(wmm, lon_deg, lat_deg, alt_km);
    }
    
    ~GeoMag()
    {
        wmm_destroy(&wmm);
    }
    
private:
    wmm_t wmm;
};