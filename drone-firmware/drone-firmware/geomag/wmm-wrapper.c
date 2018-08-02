#include "./wmm-wrapper.h"

#include "./wmm.h"
//#include "EGM9615.h"

/* BUG time.h is busted in galileo
#include <time.h>
*/

typedef struct wmm_s_t {
    MAGtype_MagneticModel *mmodel[1], *tmodel;
    MAGtype_Geoid geoid;
    MAGtype_Ellipsoid ellip;
    int ready;
} wmm_s_t;

void wmm_init(wmm_t *data)
{
    *data = (wmm_s_t*) malloc(sizeof(wmm_s_t));
    
    wmm_s_t *wmm = *data;
    wmm->ready = 0;
    
    if(!MAG_robustReadMagModels(MODEL_DATA_FILE, &(wmm->mmodel), 1/*epoch*/)) 
        return;
    
    int nMax = wmm->mmodel[0]->nMax;
    int nTerms = ((nMax + 1) * (nMax + 2) / 2);
    wmm->tmodel = MAG_AllocateModelMemory(nTerms); 
    
    // adjust model forward for today's date
    
    MAGtype_Date today;
    /* TODO time.h is busted in galileo? or im doing osomethign stupid
    time_t rawtime;   
    time ( &rawtime );
    struct tm *timeinfo = localtime ( &rawtime );
    today.Month = timeinfo->tm_mon;
    today.Day = timeinfo->tm_mday;
    today.Year = timeinfo->tm_year;
    today.DecimalYear = 99999;
    */
    today.DecimalYear = 2016.75; // HACK for now
    MAG_TimelyModifyMagneticModel(today, wmm->mmodel[0], wmm->tmodel);

    MAG_SetDefaults(&(wmm->ellip), &(wmm->geoid)); /* Set default values and constants */
    
    /* BEGIN Set EGM96 Geoid parameters */
    //wmm->geoid.GeoidHeightBuffer = EGM9615_GeoidHeightBuffer;
    //wmm->geoid.Geoid_Initialized = 1;
    /* END Set EGM96 Geoid parameters */
    
    MAG_FreeMagneticModelMemory(wmm->mmodel[0]); // not needed anymore (TODO move it out of struct)
    
    wmm->ready = 1;
}

void wmm_destroy(wmm_t *data)
{
    wmm_s_t *wmm = (wmm_s_t*) *data;
    MAG_FreeMagneticModelMemory(wmm->tmodel);
    free(*data);
    *data = 0;
}

float wmm_declination_deg(wmm_t data, float lon_deg, float lat_deg, float alt)
{
    wmm_s_t *wmm = (wmm_s_t*) data;
    
    if(!wmm->ready)
        return 0.0f;
    
    MAGtype_CoordGeodetic CoordGeodetic;
    MAGtype_CoordSpherical CoordSpherical;
    MAGtype_GeoMagneticElements GeoMagneticElements;
    
    CoordGeodetic.lambda = lon_deg;
    CoordGeodetic.phi = lat_deg;
    CoordGeodetic.HeightAboveEllipsoid = alt;
    CoordGeodetic.UseGeoid = 0;
    
    MAG_GeodeticToSpherical(wmm->ellip, CoordGeodetic, &CoordSpherical); /*Convert from geodetic to Spherical Equations: 17-18, WMM Technical report*/
    MAG_Geomag(wmm->ellip, CoordSpherical, CoordGeodetic, wmm->tmodel, &GeoMagneticElements); /* Computes the geoMagnetic field elements and their time change*/
    
    return GeoMagneticElements.Decl;
    
}