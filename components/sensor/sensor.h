/**
 * @file sensor.h
 * @defgroup sensor sensor
 * @{
 * ESP-IDF/JLR sensor types
 * Copyright (C) 2020 jlrt  <https://https://github.com/jlrtrevenga>\n
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <time.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TEMPERATURE = 0,  
    PRESSURE = 1, 
    HUMIDITY = 2  
} measure_te;

typedef enum {
    NOT_INIT = 0,  
    GOOD_QUALITY = 1,       
    TOL_QUALITY = 2,        // read error, previous value is still usable (TOLERABLE)
    BAD_QUALITY = 3,        // bad readout, invalid data
    NOT_CONFIGURED = 4      // Check if signal is configured before using it
} quality_te;

/**
 * Configuration parameters for BMP280 module.
 * Use function bmp280_init_default_params to use default configuration.
 */
typedef struct {
    float       value;
    quality_te  quality; 
    TickType_t  tickTime;              
    char        tagId[5];
    measure_te  type;
    char        unit[10];
    char        displayUnit[10];
} measure_t;
//    tm          timestamp;

struct Instrument {
   int      instrumentID;
   char     model[10];
   char     location[10];
   char     unit[10];
   double   value;
   time_t   timestamp;
};


typedef struct {
    int value_actual;
    int value_prev;
} DO_t;






#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __SENSOR_H__