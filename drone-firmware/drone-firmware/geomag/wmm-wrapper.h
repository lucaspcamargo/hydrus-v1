/**
 * @brief World Magnetic Model wrapper functions
 * This wrapper is necessary not much for clarity, but because some function signatures of 
 * WMM are forbiden in C++ (pointers to unbounded arrays), so we can't use the headers directly
 * in C++ code.
 * 
 * @by Lucas Camargo
 */

#pragma once

#ifdef __cplusplus
#define GEOMAG_H_CPP
extern "C" {
#endif
    
#define MODEL_DATA_FILE "/opt/geomag/WMM.COF"
    
typedef void* wmm_t;
    
void wmm_init(wmm_t *);
void wmm_destroy(wmm_t *); 
    
float wmm_declination_deg(wmm_t, float lon_deg, float lat_deg, float alt_km);
    
#ifdef GEOMAG_H_CPP
}
#endif