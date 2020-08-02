/**
 * @file task_programmer01.h
 * ESP-IDF task programmer 
 * 
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */
#ifndef __TASK_PROGRAMMER01_H__
#define __TASK_PROGRAMMER01_H__


#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>

struct pattern_weekly{
  int PW_ID;
  int day;
  int PD_ID;
};

struct pattern_daily{
  int PD_ID;
  int PD_ID2;
  int hour;
  int minute;
  int target_var_ID;  
};

// Consider UNION to cope with float variable types
struct target_var{
  int target_var_ID;
  int target_var_value;
};

struct pattern_pgm{
  int day;
  int hour;
  int minute;
  int target_var_value;
};

// Aux struct to keep track of relative position
struct pattern_pgm_aux{
  int prev_idx;
  int next_idx;
  int last_idx;
};


/****************************************************************************** 
* tp_init_structures - initialize week/day time_pattern structures and temp. targets
*******************************************************************************
 * @brief Initializes customized global structures (tv_cust, pd_cust, pw_cust)
 * @brief with predefined programmed structure values (tv_predef, pd_predef, pw_predef)
*************************************************************************************/
void tp_init_structures();


/****************************************************************************** 
 TODO: add functions to add, update and delete new records:
       Note: Predefined records cannot be deleted, only some values can be modified.

 int reset_default_patterns();                                    // resets default patterns
 int add_target_temp(struct target_var target_temp);             // (exactly 1 record)
 int add_pattern_daily(struct pattern_daily[6], int records);    // (group, Maximum 6 transitions x day)
 int add_pattern_weekly(struct pattern_weekly[7]);               // (group. Exactly 7 records (days))
******************************************************************************/


/****************************************************************************** 
* tp_activate_weekly_pattern - Creates active program based on customize structures and selected weekly pattern
*******************************************************************************
 * @brief look for weekly pattern in struct list. Supuesto: es una lista ordenada, corregir para caso general 
 * @param[in]  weekly pattern -> first index value to pw_predef[PW_PREDEF_ELEMENTS]
 * @brief Error: 0-ok, Not handled, TODO
*************************************************************************************/
int tp_activate_weekly_pattern(int weekly_pattern);


/****************************************************************************** 
* tp_get_target_value
*******************************************************************************
 * @brief Check programme and detect time area transitions.
 * @brief If No_change and override_temp set, send this value. Else, return target value.
 * @param[in] actual_time 
 * @param[in] weekly_program_id: active program in use,Index to pw_predef (pattern_weekly.PW_ID).
 * @param[in] override_value: manually changed value, replaces target value until next transition.
 * @param[out] target_value: target value found. 0-No_change / 1-Transition / 2-Error / 3-Invalid time
*******************************************************************************/
int tp_get_target_value(time_t actual_time, bool *poverride_active, int *override_temp, int *target_value);



#ifdef __cplusplus
}
#endif

#endif  // __TASK_PROGRAMMER01_H__
