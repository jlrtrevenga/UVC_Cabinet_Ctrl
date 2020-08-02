#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "task_programmer01.h"
#include "esp_log.h"


// PREDEFINED target_value, daily pattern and weekly patterns

#define TV_PREDEF_ELEMENTS 5
/*
static struct target_var tv_predef[TV_PREDEF_ELEMENTS] = {
  {1, 5},     // ºC, antihielo
  {2, 10},    // ºC, temp. baja
  {3, 18},    // ºC, temp. mantenimiento
  {4, 21},    // ºC, temp. comfort
  {5, 23}    // ºC, temp. alta
  };
*/

static struct target_var tv_predef[TV_PREDEF_ELEMENTS] = {
  {1, 5},     // ºC, antihielo
  {2, 10},    // ºC, temp. baja
  {3, 20},    // ºC, temp. mantenimiento
  {4, 24},    // ºC, temp. comfort
  {5, 30}    // ºC, temp. alta
  };

#define PD_PREDEF_ELEMENTS 26 
static struct pattern_daily pd_predef[PD_PREDEF_ELEMENTS] = {
  {0,1,0,0,1},    // Temperatura Continua, Antihielo
  {1,1,0,0,2},    // Temperatura Continua, Baja
  {2,1,0,0,3},    // Temperatura Continua, Mantenimiento
  {3,1,0,0,4},    // Temperatura Continua, Confort
  {4,1,0,0,3},    // Oficina, Confort L-V
  {4,2,6,45,4},   // Oficina, Confort L-V
  {4,3,19,0,3},   // Oficina, Confort L-V
  {5,1,0,00,3},    // Casa, Horario trabajador L-V
  {5,2,10,10,4},    // Casa, Horario trabajador L-V
  {5,3,12,15,3},   // Casa, Horario trabajador L-V
  {5,4,13,20,4},   // Casa, Horario trabajador L-V
  {5,5,16,01,3},   // Casa, Horario trabajador L-V 
  {5,6,16,30,4},   // Casa, Horario trabajador L-V
  {5,7,18,00,3},    // Casa, Horario trabajador L-V
  {5,8,18,35,4},    // Casa, Horario trabajador L-V
  {5,9,18,44,3},   // Casa, Horario trabajador L-V
  {5,10,20,05,4},   // Casa, Horario trabajador L-V
  {5,11,20,20,3},   // Casa, Horario trabajador L-V 
  {5,12,21,30,4},   // Casa, Horario trabajador L-V  
  {5,13,22,15,3},   // Casa, Horario trabajador L-V    
  {6,1,0,0,3},    // Casa, Ocupación permanente
  {6,2,8,0,4},    // Casa, Ocupación permanente
  {6,3,23,0,3},   // Casa, Ocupación permanente
  {7,1,0,0,2},    // Casa finde, templar viernes tarde
  {7,2,16,0,4},   // Casa finde, templar viernes tarde
  {7,3,23,0,2},   // Casa finde, templar viernes tarde
  };

/*
static struct pattern_daily pd_predef[PD_PREDEF_ELEMENTS] = {
  {0,1,0,0,1},    // Temperatura Continua, Antihielo
  {1,1,0,0,2},    // Temperatura Continua, Baja
  {2,1,0,0,3},    // Temperatura Continua, Mantenimiento
  {3,1,0,0,4},    // Temperatura Continua, Confort
  {4,1,0,0,3},    // Oficina, Confort L-V
  {4,2,6,45,4},   // Oficina, Confort L-V
  {4,3,19,0,3},   // Oficina, Confort L-V
  {5,1,0,0,3},    // Casa, Horario trabajador L-V
  {5,2,6,0,4},    // Casa, Horario trabajador L-V
  {5,3,7,30,3},   // Casa, Horario trabajador L-V 
  {5,4,18,30,4},   // Casa, Horario trabajador L-V
  {5,5,22,30,3},   // Casa, Horario trabajador L-V 
  {6,1,0,0,3},    // Casa, Ocupación permanente
  {6,2,8,0,4},    // Casa, Ocupación permanente
  {6,3,23,0,3},   // Casa, Ocupación permanente
  {7,1,0,0,2},    // Casa finde, templar viernes tarde
  {7,2,16,0,4},   // Casa finde, templar viernes tarde
  {7,3,23,0,2},   // Casa finde, templar viernes tarde
  };
  */

#define PW_PREDEF_ELEMENTS 28
static struct pattern_weekly pw_predef[PW_PREDEF_ELEMENTS] = {
  {1,1,4},    // Oficina: confort L-V, Mmto S-D
  {1,2,4},    // Oficina: confort L-V, Mmto S-D
  {1,3,4},    // Oficina: confort L-V, Mmto S-D
  {1,4,4},    // Oficina: confort L-V, Mmto S-D
  {1,5,4},    // Oficina: confort L-V, Mmto S-D
  {1,6,2},    // Oficina: confort L-V, Mmto S-D
  {1,7,2},    // Oficina: confort L-V, Mmto S-D
  {2,1,5},    // Casa: trabajo L-V, confort S-D
  {2,2,5},    // Casa: trabajo L-V, confort S-D
  {2,3,5},    // Casa: trabajo L-V, confort S-D
  {2,4,5},    // Casa: trabajo L-V, confort S-D
  {2,5,5},    // Casa: trabajo L-V, confort S-D
  {2,6,6},    // Casa: trabajo L-V, confort S-D
  {2,7,6},    // Casa: trabajo L-V, confort S-D
  {3,1,6},    // Casa: Ocupación L-D
  {3,2,6},    // Casa: Ocupación L-D
  {3,3,6},    // Casa: Ocupación L-D
  {3,4,6},    // Casa: Ocupación L-D
  {3,5,6},    // Casa: Ocupación L-D
  {3,6,6},    // Casa: Ocupación L-D
  {3,7,6},    // Casa: Ocupación L-D
  {4,1,1},    // Casa de Finde: Calentar el viernes tarde
  {4,2,1},    // Casa de Finde: Calentar el viernes tarde
  {4,3,1},    // Casa de Finde: Calentar el viernes tarde
  {4,4,1},    // Casa de Finde: Calentar el viernes tarde
  {4,5,7},    // Casa de Finde: Calentar el viernes tarde
  {4,6,6},    // Casa de Finde: Calentar el viernes tarde
  {4,7,6}    // Casa de Finde: Calentar el viernes tarde
};

/*
#define PW_PREDEF_ELEMENTS 28
static struct pattern_weekly pw_predef[PW_PREDEF_ELEMENTS] = {
  {1,1,4},    // Oficina: confort L-V, Mmto S-D
  {1,2,4},    // Oficina: confort L-V, Mmto S-D
  {1,3,4},    // Oficina: confort L-V, Mmto S-D
  {1,4,4},    // Oficina: confort L-V, Mmto S-D
  {1,5,4},    // Oficina: confort L-V, Mmto S-D
  {1,6,2},    // Oficina: confort L-V, Mmto S-D
  {1,7,2},    // Oficina: confort L-V, Mmto S-D
  {2,1,5},    // Casa: trabajo L-V, confort S-D
  {2,2,5},    // Casa: trabajo L-V, confort S-D
  {2,3,5},    // Casa: trabajo L-V, confort S-D
  {2,4,5},    // Casa: trabajo L-V, confort S-D
  {2,5,7},    // Casa: trabajo L-V, confort S-D
  {2,6,6},    // Casa: trabajo L-V, confort S-D
  {2,7,6},    // Casa: trabajo L-V, confort S-D
  {3,1,6},    // Casa: Ocupación L-D
  {3,2,6},    // Casa: Ocupación L-D
  {3,3,6},    // Casa: Ocupación L-D
  {3,4,6},    // Casa: Ocupación L-D
  {3,5,6},    // Casa: Ocupación L-D
  {3,6,6},    // Casa: Ocupación L-D
  {3,7,6},    // Casa: Ocupación L-D
  {4,1,1},    // Casa de Finde: Calentar el viernes tarde
  {4,2,1},    // Casa de Finde: Calentar el viernes tarde
  {4,3,1},    // Casa de Finde: Calentar el viernes tarde
  {4,4,1},    // Casa de Finde: Calentar el viernes tarde
  {4,5,7},    // Casa de Finde: Calentar el viernes tarde
  {4,6,6},    // Casa de Finde: Calentar el viernes tarde
  {4,7,6}    // Casa de Finde: Calentar el viernes tarde
  };
*/

// Customized patter structures, initialized as copy of predefined structures.
// New programs can be added, date/time can be changed as well as target temperature
// this changes are updated on customized variables, to enable "reset" to predefined values at any time.

#define TV_CUST_ELEMENTS 10
static struct target_var     tv_cust[TV_CUST_ELEMENTS];           // initialized as copy of const values. May be personalized.

#define PD_CUST_ELEMENTS 30
static struct pattern_daily  pd_cust[PD_CUST_ELEMENTS];           // initialized as copy of const values. May be personalized.

#define PW_CUST_ELEMENTS 50
static struct pattern_weekly pw_cust[PW_CUST_ELEMENTS];           // initialized as copy of const values. May be personalized.


// ACTIVE PROGRAM structure

#define ACT_PGM_ELEMENTS 100
static struct pattern_pgm    active_pgm[ACT_PGM_ELEMENTS];      // initialized based on previous structures 
static struct pattern_pgm_aux ppa;


// internal functions
int lftv(struct target_var *tv);
int lfwp(struct pattern_weekly *pw);
int lfdp(struct pattern_daily *pd);
int tp_eval_pos(int type, struct pattern_pgm tp2);
void tp_relocate_indexes(struct pattern_pgm now);

static const char *TAG = "TASK_PROGRAMMER01";


/****************************************************************************** 
* tp_init_structures - initialize week/day time_pattern structures and temp. targets
*******************************************************************************
 * @brief Initializes customized global structures (tv_cust, pd_cust, pw_cust)
 * @brief with predefined programmed structure values (tv_predef, pd_predef, pw_predef)
*************************************************************************************/
void tp_init_structures(){

    // init target variable customized
    for (int i=0; i < TV_CUST_ELEMENTS; i++){
        if (i < TV_PREDEF_ELEMENTS) {
            tv_cust[i].target_var_ID = tv_predef[i].target_var_ID;
            tv_cust[i].target_var_value = tv_predef[i].target_var_value;
            } else {
            tv_cust[i].target_var_ID = (int) NULL;
            tv_cust[i].target_var_value = (int) NULL;            
            }
        }

    // init patter daily customized
    for (int i=0; i<PD_CUST_ELEMENTS; i++){
        if (i < PD_PREDEF_ELEMENTS) {
            pd_cust[i].PD_ID = pd_predef[i].PD_ID;
            pd_cust[i].PD_ID2 = pd_predef[i].PD_ID2;
            pd_cust[i].hour = pd_predef[i].hour;
            pd_cust[i].minute = pd_predef[i].minute;
            pd_cust[i].target_var_ID = pd_predef[i].target_var_ID;        
            } else {
            pd_cust[i].PD_ID =  (int) NULL;
            pd_cust[i].PD_ID2 =  (int) NULL;
            pd_cust[i].hour =  (int) NULL;
            pd_cust[i].minute =  (int) NULL;
            pd_cust[i].target_var_ID =  (int) NULL;
            }
        }

    // init patter weekly customized
    for (int i=0; i<PW_CUST_ELEMENTS; i++){
        if (i < PW_PREDEF_ELEMENTS) {
            pw_cust[i].PW_ID = pw_predef[i].PW_ID;
            pw_cust[i].day = pw_predef[i].day;
            pw_cust[i].PD_ID = pw_predef[i].PD_ID;        
            } else {
            pw_cust[i].PW_ID = (int) NULL;
            pw_cust[i].day = (int) NULL;
            pw_cust[i].PD_ID = (int) NULL;        
            }
        }
}


/****************************************************************************** 
* tp_activate_weekly_pattern - Creates active program based on customize structures and selected weekly pattern
*******************************************************************************
 * @brief look for weekly pattern in struct list. Supuesto: es una lista ordenada, corregir para caso general 
 * @param[in]  weekly pattern -> first index value to pw_predef[PW_PREDEF_ELEMENTS]
 * @brief Error: 0-ok, Not handled, TODO
*************************************************************************************/
int tp_activate_weekly_pattern(int weekly_pattern){

    struct pattern_weekly pw_record;
    struct pattern_daily pd_record;
    struct target_var tv_record;

    int program_records = 0;
    int p_index = 0;
    int error = 0;
    bool pattern_weekly_found = 1;
    bool pattern_daily_found  = 1;

    ppa.prev_idx = 0;
    ppa.next_idx = 1;
    ppa.last_idx = 0;

        pw_record.PW_ID = weekly_pattern;       
        pw_record.day = 1;                      // El segundo indice empieza siempre en 1   
        do {
            //Foreach week look for day records ...
            error = lfwp(&pw_record);
            if (error == 0){            
                // for each day look for daytime changes ...
                pattern_weekly_found = 1;
                pd_record.PD_ID = pw_record.PD_ID;
                pd_record.PD_ID2 = 1;                        
                do {
                    error = lfdp(&pd_record);
                    if (error == 0){
                        pattern_daily_found = 1;
                        tv_record.target_var_ID = pd_record.target_var_ID;
                        error = lftv(&tv_record);
                        //add new temperature change time setting
                        if ((error == 0) & (program_records < ACT_PGM_ELEMENTS)){
                            active_pgm[p_index].day = pw_record.day;
                            active_pgm[p_index].hour = pd_record.hour;
                            active_pgm[p_index].minute = pd_record.minute;
                            active_pgm[p_index].target_var_value = tv_record.target_var_value;
                            ESP_LOGI(TAG, "idx: %d - active pattern = {%d, %d, %d, %d}", 
                                    p_index, pw_record.day, pd_record.hour, pd_record.minute, tv_record.target_var_value);
                            p_index++;
                            program_records++;
                            ppa.last_idx++;  // Number of records in pattern_pgm
                        }
                        else if (program_records >= ACT_PGM_ELEMENTS){
                            ESP_LOGE(TAG, "ACTIVE_PATTERN has more than %d records.", ACT_PGM_ELEMENTS);
                        }
                    } else pattern_daily_found = 0;
                    pd_record.PD_ID2++;
                } while (pattern_daily_found);
            } else pattern_weekly_found = 0;
            pw_record.day++;
        } while (pattern_weekly_found);
        ppa.last_idx--;  // Decrease the last update as it was the last record

        //get actual day/time 
        
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        struct pattern_pgm now_temp;
        now_temp.day = timeinfo.tm_wday;
        now_temp.hour = timeinfo.tm_hour;
        now_temp.minute = timeinfo.tm_min;

        tp_relocate_indexes(now_temp);         // Reevaluate prev/next idx and exit
        return(0);
}


/****************************************************************************** 
* tp_get_target_value
*******************************************************************************
 * @brief Check programme and detect time area transitions.
 * @brief If No_change and override_temp set, send this value. Else, return target value.
 * @param[in] time_t actual_time 
 * @param[in] bool *poverride_active: flag. Active if override temperature has been selected.
 * @param[in] int  *p_override_temp: override temperature, replaces program temperature if override_active==1 till next trasition
 * @param[in] int  *p_target_value: temperature recovered from program/override_temperature. 
 * @param[return]  Values: 0:No_change / 1:Transition / 2:Error / 3:Invalid time
*******************************************************************************/
int tp_get_target_value(time_t actual_time, bool *poverride_active, int *p_override_temp, int *p_target_value){

    int error = 0;

    ESP_LOGI(TAG, "tp_get_target_value: INPUT : ppa.prev_idx: %d, ppa.next_idx: %d, ppa.last_idx: %d", ppa.prev_idx, ppa.next_idx, ppa.last_idx);

    //time(&now);
    struct tm timeinfo;
    localtime_r(&actual_time, &timeinfo);
    struct pattern_pgm now_temp;
    now_temp.day = timeinfo.tm_wday;
    now_temp.hour = timeinfo.tm_hour;
    now_temp.minute = timeinfo.tm_min;

    // If year < 2020 => RETURN(3) (invalid time, probably SNTP has not yet synchronized time)
    if (timeinfo.tm_year < (2020 - 1900)) { 
        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

        ESP_LOGE(TAG, "tp_get_target_value: TIME ERROR - Localtime in Madrid (UTC-1,M3.5.0/2,M10.4.0/2) is: %s", strftime_buf);
        //ESP_LOGE(TAG, "Date: %d / %d / %d", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday );
        //ESP_LOGE(TAG, "tm_year: %d, tm_yday: %d, tm_wday: %d, tm_mday: %d", timeinfo.tm_year, timeinfo.tm_yday, timeinfo.tm_wday, timeinfo.tm_mday  );
        //ESP_LOGE(TAG, "tm_mon: %d, tm_hour: %d, tm_min: %d, tm_sec: %d", timeinfo.tm_mon, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);        
        //ESP_LOGE(TAG, "tm_tm_isdst: %d", timeinfo.tm_isdst);                
        //ESP_LOGE(TAG, "WeekDay: %d, Time: %d:%d:%d", timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        error = 3; 
        } 
    else {
        // Check relative position: now temp vs. previous and next transition
        int p1, p2;
        p1 = tp_eval_pos(1, now_temp);      // now_temp vs. previous time transition
        p2 = tp_eval_pos(2, now_temp);      // now_temp vs. next time transition
        ESP_LOGI(TAG, "tp_get_target_value: Day %d, %d:%d:%d.............p1: %d, p2: %d", timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, p1, p2);

        // evaluate transitions
        if ((p1==2) || (p1==3 && p2==1)){          // RETURN(0): No change, keep temperature setpoint
            if (*poverride_active) { (*p_target_value = *p_override_temp);}
            else { *p_target_value = active_pgm[ppa.prev_idx].target_var_value; }
            error = 0;
            ESP_LOGI(TAG, "tp_get_target_value: -> No Change, keep temp. setpoint: %d", *p_target_value);   
            }
        else if (p2 == 2){                         // RETURN(1): Transition to new time area, update prev & next indexes
            if (ppa.next_idx == ppa.last_idx) 
                { ppa.prev_idx++; ppa.next_idx = 0;} 
            else if (ppa.prev_idx == ppa.last_idx)
                { ppa.prev_idx = 0; ppa.next_idx = 1;} 
            else { ppa.prev_idx++; ppa.next_idx++; }

            *poverride_active = false;
            *p_target_value = active_pgm[ppa.prev_idx].target_var_value;
            error = 1;
            ESP_LOGI(TAG, "tp_get_target_value: -> Transition. Update temp. setpoint %d, Update idx. ppa.prev_idx=%d, ppa.next_idx=%d", *p_target_value, ppa.prev_idx, ppa.next_idx);
            } 
        else if (p1 == 1 || p2 == 3){              // RETURN(2): ERROR
            tp_relocate_indexes(now_temp);         // Reevaluate prev/next idx and exit
            error = 2;
            ESP_LOGE(TAG, "tp_get_target_value: -> ERROR, Relocate indexes: ppa.prev_idx=%d, ppa.next_idx=%d", ppa.prev_idx, ppa.next_idx);        
            }
    }
    ESP_LOGI(TAG, "tp_get_target_value: OUTPUT: ppa.prev_idx: %d, ppa.next_idx: %d, ppa.last_idx: %d", ppa.prev_idx, ppa.next_idx, ppa.last_idx);

    return(error);
};


/****************************************************************************** 
* lfwp - look for weekly pattern
*******************************************************************************/
// look for weekly pattern in struct list. Supuesto: es una lista ordenada, corregir para caso general 
// @param[in]   *pw (pattern_week): pw->PW_ID and pw->day are the keys to find records.
// @param[out]  *pw (pattern_week): pw->PD_ID.
// Error: 0:ok, <> 0:Error 
int lfwp(struct pattern_weekly *pw){
    int pw_idx = -1;
    int salir = 0;
    int error = 0;
    while (salir == 0) {
        pw_idx++;
        ESP_LOGV(TAG, "lfwp 0 = {%d, %d}", pw->PW_ID, pw->day);
        if (pw_cust[pw_idx].PW_ID == pw->PW_ID) {
            if (pw_cust[pw_idx].day == pw->day) {
                pw->PD_ID = pw_cust[pw_idx].PD_ID;
                salir = 1;
                error = 0;
                ESP_LOGV(TAG, "lfwp 1 = {%d, %d, %d}", pw->PW_ID, pw->day, pw->PD_ID);
                }
            else if (pw_idx == PW_PREDEF_ELEMENTS) {
                pw->day =  (int) NULL;
                pw->PD_ID = (int) NULL;            
                salir = 1;
                error = 1;
                ESP_LOGV(TAG, "lfwp 2 = {%d, %d, %d}", pw->PW_ID, pw->day, pw->PD_ID);
                }
            }
        else if (pw_idx == PW_PREDEF_ELEMENTS) {
            pw->day =  (int) NULL;
            pw->PD_ID = (int) NULL;            
            salir = 1;
            error = 1;
            ESP_LOGV(TAG, "lfwp 3 = {%d, %d, %d}", pw->PW_ID, pw->day, pw->PD_ID);
            }
    }
    return(error);
}



/****************************************************************************** 
* lfdp - look for dayly pattern
*******************************************************************************/
// look for dayly pattern in struct list. Supuesto: es una lista ordenada, corregir para caso general
// @param[in]   *pd (pattern_day): pd->PW_ID and pd->ID2 are the keys to find records.
// @param[out]  *pd (pattern_day): pd->hour, pd->minute, pd->target_var_ID
// Error: 0:ok, <> 0:Error 
int lfdp(struct pattern_daily *pd){
    int pd_idx = -1;
    int salir = 0;
    int error = 0;
    while (salir == 0) {
        pd_idx++;
        ESP_LOGV(TAG, "lfdp 0 = {%d, %d}", pd->PD_ID, pd->PD_ID2);
        if (pd_cust[pd_idx].PD_ID == pd->PD_ID) {
            if (pd_cust[pd_idx].PD_ID2 == pd->PD_ID2) {
                pd->hour = pd_cust[pd_idx].hour;
                pd->minute = pd_cust[pd_idx].minute;
                pd->target_var_ID = pd_cust[pd_idx].target_var_ID;          
                salir = 1;
                error = 0;
                ESP_LOGV(TAG, "lfdp 1 = {%d, %d, %d, %d, %d}", pd->PD_ID, pd->PD_ID2, pd->hour, pd->minute, pd->target_var_ID);
                }
            else if (pd_idx == PD_PREDEF_ELEMENTS) {
                pd->hour =  (int) NULL;
                pd->minute =  (int) NULL;
                pd->target_var_ID =  (int) NULL;                     
                salir = 1;
                error = 1;
                ESP_LOGV(TAG, "lfdp 2 = {%d, %d, %d, %d, %d}", pd->PD_ID, pd->PD_ID2, pd->hour, pd->minute, pd->target_var_ID);
                }
            }
        else if (pd_idx == PD_PREDEF_ELEMENTS) {
            pd->hour =  (int) NULL;
            pd->minute =  (int) NULL;
            pd->target_var_ID =  (int) NULL;                     
            salir = 1;
            error = 1;
            ESP_LOGV(TAG, "lfdp 3 = {%d, %d, %d, %d, %d}", pd->PD_ID, pd->PD_ID2, pd->hour, pd->minute, pd->target_var_ID);
            }
    }
    return(error);
}



/****************************************************************************** 
* lftv - look for target value
*******************************************************************************/
// look for target variable in struct list. Supuesto: es una lista ordenada, corregir para caso general 
// @param[in]  *tv->target_var_ID
// @param[out] *tv->target_var_value = value or NULL
// Error: 0:ok, <> 0:Error 
int lftv(struct target_var *tv){
    int tv_idx = -1;
    int salir = 0;
    int error = 0;
    while (salir == 0) {
        tv_idx++;
        if (tv_cust[tv_idx].target_var_ID == tv->target_var_ID) {
            tv->target_var_value = tv_cust[tv_idx].target_var_value;
            salir = 1;
            error = 0;
            }
        else if (tv_idx == TV_PREDEF_ELEMENTS) {
            tv->target_var_value = (int) NULL;             
            salir = 1;
            error = 1;
            }
    }
    return(error);
}




/****************************************************************************** 
* tp_eval_pos
*******************************************************************************
 * @brief checks relative position between the two day/time points. 
 * @param[in] type (1:prev, 2:next) 
 * @param[in] tp2 (now time)
 * @return: 0:Not_Valid, 1:Before, 2:Equal, 3:After 
*******************************************************************************/
int tp_eval_pos(int type, struct pattern_pgm tp2){

    struct pattern_pgm tp1;
    if (tp2.day == 0) { (tp2.day = 7); }           // Sunday is recoded from 0 to 7 to simplify comparison

    ESP_LOGI(TAG, "tp_eval_pos - INPUT : ppa.prev_idx: %d, ppa.next_idx: %d, ppa.last_idx: %d", ppa.prev_idx, ppa.next_idx, ppa.last_idx);

    //Restart index when max_index reached
    if (type == 1) {
        tp1 = active_pgm[ppa.prev_idx];
        }
    else if (type == 2) {
        if (ppa.prev_idx < ppa.last_idx) { 
            //ppa.next_idx = ppa.prev_idx++ ; //CHECK
            tp1 = active_pgm[ppa.next_idx];
            }
        else {
            (ppa.next_idx = 0);                     // If next day restarts the cycle, index is reset to 0 to recover record ...
            tp1 = active_pgm[ppa.next_idx];
            tp1.day = 8;                            
            }
        }

    ESP_LOGI(TAG, "tp_eval_pos - Check: %d ---tp1: %d-%d:%d", type, tp1.day, tp1.hour, tp1.minute);
    ESP_LOGI(TAG, "tp_eval_pos - Check: %d ---tp2: %d-%d:%d", type, tp2.day, tp2.hour, tp2.minute);

    ESP_LOGI(TAG, "tp_eval_pos - OUTPUT: ppa.prev_idx: %d, ppa.next_idx: %d, ppa.last_idx: %d", ppa.prev_idx, ppa.next_idx, ppa.last_idx);    

    // compare times and assign relative position
    if (tp2.day < tp1.day) { return(1); }
    else if (tp2.day > tp1.day) { return(3); }
    else {                                                      // Same day, check hour
        if (tp2.hour < tp1.hour) { return(1); }
        else if (tp2.hour > tp1.hour) { return(3); }    
        else {                                                  // Same hour, check minute
            if (tp2.minute < tp1.minute) { return(1); }
            else if (tp2.minute > tp1.minute) { return(3); }
            else if (tp2.minute == tp1.minute) { return(2); }
            else { return(0);}
            }
        }

     
}

/****************************************************************************** 
* tp_relocate_indexes
*******************************************************************************
 * @brief Goes though active_pgm[] with now date/time and resets ppa.prev_idx, ppa.next_idx. 
 * @brief Updates global structures.  
 * @param[in] now time/date
*******************************************************************************/
void tp_relocate_indexes(struct pattern_pgm now){

    if (now.day == 0) { now.day = 7; }            // Sunday correction: 0 -> 7

    //for (int i = 0; i< ppa.last_idx; i++){
    //int result = 0;    
    int idx = 0;
    int salir = 0;

    ESP_LOGI(TAG, "tp_relocate_indexes-INPUT : ppa.prev_idx: %d, ppa.next_idx: %d, ppa.last_idx: %d", ppa.prev_idx, ppa.next_idx, ppa.last_idx);
    ESP_LOGI(TAG, "tp_relocate_indexes-Relocating idx ---> Day %d, %d:%d", now.day, now.hour, now.minute);
    do {
        // Check day
        if (active_pgm[idx].day < now.day ) {           // day not reached, keep going
            idx++;
             }
        else if (active_pgm[idx].day > now.day ) {      // Day overpassed, there are undefined days: allocate next_idx and end
            ppa.prev_idx = idx-1;
            ppa.next_idx = idx;
            salir = 1;
            }        
        else if (active_pgm[idx].day == now.day ) {      // Same day, check hour
            // Check hour
            if (active_pgm[idx].hour < now.hour ) {     // Hour not reached, keep going
                idx++;
                 }
            else if (active_pgm[idx].hour > now.hour ) {     // Hour overpassed, there are undefined hours: allocate next_idx and end
                ppa.prev_idx = idx-1;
                ppa.next_idx = idx;
                salir = 1;
                }
            else if (active_pgm[idx].hour == now.hour ) {     // Same hour, keep going
                // Check minute
                if (active_pgm[idx].minute < now.minute ) {     // Minute not reached, keep going
                    idx++;
                    }
                else if (active_pgm[idx].minute > now.minute ) {     // Minute overpassed, there are undefined minutes: allocate next_idx and end
                    ppa.prev_idx = idx-1;
                    ppa.next_idx = idx;
                    salir = 1;
                    }
                else if (active_pgm[idx].minute == now.minute ) {     // Same day:hour:minute -> allocate next_idx and end
                    ppa.prev_idx = idx-1;
                    ppa.next_idx = idx;
                    salir = 1;
                    }
                }
            }
        } while (salir == 0);
    
        ESP_LOGI(TAG, "tp_relocate_indexes-OUTPUT: ppa.prev_idx: %d, ppa.next_idx: %d, ppa.last_idx: %d", ppa.prev_idx, ppa.next_idx, ppa.last_idx);

}



