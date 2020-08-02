/* esp_event (event loop library) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef GPIO_OBJECTS_H_
#define GPIO_OBJECTS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>

// Declarations for the event source
#define TASK_ITERATIONS_COUNT        10      // number of times the task iterates
#define TASK_PERIOD                2000      // period of the task loop in milliseconds
/*
struct Instrument {
   int      instrumentID;
   char     model[10];
   char     location[10];
   char     unit[10];
   double   value;
   time_t   timestamp;
};
*/
#ifdef __cplusplus
}
#endif

#endif // #ifndef GPIO_OBJECTS_H_