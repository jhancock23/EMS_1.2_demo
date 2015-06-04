#include <stdlib.h>
#include <stdio.h>
#include <semLib.h>
#include <unistd.h>
#include <wra.h>

#include <string.h>
#ifdef INCLUDE_WR_EMS_REF_APP
#include "wra-HD44780U-lcd.h"
#include "wra-galileo-io.h"
#define REF_HW_SUPPORT 1
#endif

#define REF_APP_NAME "reference-demo"
/*
 * define it when dataitem changed to device from cloud
 * and send dataitem change back to cloud
 */
#define REF_REPORT_BACK

#define DEBUG_REF_APP
#define REF_LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#if defined(DEBUG_REF_APP)
#define REF_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define REF_DEBUG(fmt, ...) 
#endif
#define SENSOR_ANALOG_PIN 1
#define REF_REPORT_RATE   5
#define REF_MAX_READERS   3
#define REF_APP_ALARM_SEVERITY        100
#define REF_APP_SIM_TEMP_MAX          40
#define REF_APP_DEFAULT_TEMP_THRSHOLD 40
#define REF_APP_DEFAULT_TEST          "DefaultText"
enum {
	APP_NOT_INIT,
	APP_RUNNING,
	APP_STOPPED
};
static int ref_app_status = APP_STOPPED;

enum ref_data_item {
    DATA_SHOW_TEXT,
    DATA_TEMPERATURE,
    DATA_THRESHOLD,
    DATA_LAST
};

enum ref_alarm_item {
    ALARM_TEMPERATURE,
    ALARM_LAST
};

struct ref_data {
    char *name;
    int type;
    union {
        double f;
        char  *s;
        int   b;
    }d;
};

struct ref_alarm {
    char *name;
    char *description;
    enum ref_data_item data;
    enum ref_data_item threshold;
};

struct ref_ctl
{
    int        ref_sensor_reading;
    wra_handle handle;

    /* data items */
    wra_tm_handle data[DATA_LAST];
    struct ref_data d[DATA_LAST];
    void       *dops[DATA_LAST];
    SEM_ID     semRW;

    /* alarms */
    struct ref_alarm a[ALARM_LAST];
    wra_tm_handle alarm[ALARM_LAST];
};

STATUS set_sensor_reading(int action);
STATUS get_sensor_reading(int *action);
LOCAL STATUS init_input_output(struct ref_ctl*);
LOCAL STATUS register_to_agent(struct ref_ctl * ctl);
LOCAL STATUS unregister_to_agent(struct ref_ctl * ctl);
#if defined(REF_HW_SUPPORT)
LOCAL STATUS lcd_output_string(const char * p, uint8_t col, uint8_t row);
LOCAL STATUS lcd_output_temperature(const double * pTemp, uint8_t col, uint8_t row);
LOCAL STATUS lm35v2_convert_temperature(uint16_t meter, double * pTemp);
#endif
LOCAL STATUS simulator_read_analog(uint8_t pin, uint16_t * val);
LOCAL STATUS simulator_convert_temperature(uint16_t meter, double *pTemp);
LOCAL STATUS console_output_string(const char * p, uint8_t col, uint8_t row);
LOCAL STATUS console_output_temperature(const double * pTemp, uint8_t col, uint8_t row);
LOCAL STATUS generic_change_string(const char **ps, const char * p);
LOCAL STATUS generic_change_float(double *o, const double *n);

LOCAL STATUS post_data_item(struct ref_ctl *ctl, int index);
LOCAL STATUS ref_register_application_hook(void);

typedef STATUS (*output_string)(const char *, uint8_t col, uint8_t row);
typedef STATUS (*change_string)(const char **, const char *);
typedef STATUS (*change_float)(double *, const double *);
typedef STATUS (*convert_temperature)(uint16_t, double *);
typedef STATUS (*output_temperature)(const double *, uint8_t col, uint8_t row);
typedef STATUS (*read_analog)(uint8_t, uint16_t*);
typedef STATUS (*post_temperature)(struct ref_ctl *, int);
typedef STATUS (*post_data)(struct ref_ctl *ctl, int index);

struct string_ops
{
    /* output location on lcd */
    uint8_t col, row;

    output_string poutput;
    change_string pchange;
    post_data     ppost;
};

struct analog_ops
{
    /* output location on lcd */
    uint8_t col, row;

    /* analogPin: like A1, A2, not the GPIO pin */
    uint8_t              analogPin;
    read_analog          pread;
    convert_temperature  pconvert;
    output_temperature   poutput;
    post_data            ppost;
    change_float         pchange;
};
static wra_handle agent_h = NULL;

LOCAL struct ref_ctl ref_ctl;
#if defined(REF_HW_SUPPORT)
LOCAL struct string_ops lcd_output_ops = {
    0, 0,
    &lcd_output_string,
    &generic_change_string,
    &post_data_item,
};

LOCAL struct analog_ops lm35v2_real_sensor_ops = {
    0, 1,
    1,
    &analogRead,
    &lm35v2_convert_temperature,
    &lcd_output_temperature,
    &post_data_item,
    &generic_change_float,
};
#endif

LOCAL struct analog_ops threshold_ops = {
    16, 1,
    1,
    NULL,
    NULL,
    NULL,
    &post_data_item,
    &generic_change_float,
};

LOCAL struct string_ops console_output_ops = {
    0, 0,
    &console_output_string,
    &generic_change_string,
    &post_data_item,
};

LOCAL struct analog_ops simulator_sensor_ops = {
    0, 1,
    10,
    &simulator_read_analog,
    &simulator_convert_temperature,
    &console_output_temperature,
    &post_data_item,
    &generic_change_float,
};

LOCAL STATUS init_ctrl(void)
{
    memset(&ref_ctl, 0, sizeof(ref_ctl));

    ref_ctl.semRW = 
        semRWCreate(SEM_Q_PRIORITY|SEM_DELETE_SAFE|SEM_INVERSION_SAFE,
                    REF_MAX_READERS);
    if (ref_ctl.semRW == SEM_ID_NULL) {
        REF_LOG("%s failed to create read/write lock\n", __FUNCTION__);
        return ERROR;
    }

    ref_ctl.d[DATA_SHOW_TEXT].name = "ShowText";
    ref_ctl.d[DATA_SHOW_TEXT].type = WRA_TM_DATATYPE_STRING;
    ref_ctl.d[DATA_SHOW_TEXT].d.s = strdup(REF_APP_DEFAULT_TEST);
#if defined(REF_HW_SUPPORT)
    ref_ctl.dops[DATA_SHOW_TEXT] = &lcd_output_ops;
#else
    ref_ctl.dops[DATA_SHOW_TEXT] = &console_output_ops;
#endif

    ref_ctl.d[DATA_TEMPERATURE].name = "temperature";
    ref_ctl.d[DATA_TEMPERATURE].type = WRA_TM_DATATYPE_DOUBLE;
    ref_ctl.d[DATA_TEMPERATURE].d.f = 0.0;
#if defined(REF_HW_SUPPORT)
    ref_ctl.dops[DATA_TEMPERATURE] = &lm35v2_real_sensor_ops;
#else
    ref_ctl.dops[DATA_TEMPERATURE] = &simulator_sensor_ops;
#endif

    ref_ctl.d[DATA_THRESHOLD].name = "Threshold";
    ref_ctl.d[DATA_THRESHOLD].type = WRA_TM_DATATYPE_DOUBLE;
    ref_ctl.d[DATA_THRESHOLD].d.f = REF_APP_DEFAULT_TEMP_THRSHOLD;
    ref_ctl.dops[DATA_THRESHOLD] = &threshold_ops;

    ref_ctl.a[ALARM_TEMPERATURE].name = "TemperatureAlarm";
    ref_ctl.a[ALARM_TEMPERATURE].description = "Temperature exceeds threshold";
    ref_ctl.a[ALARM_TEMPERATURE].data = DATA_TEMPERATURE;
    ref_ctl.a[ALARM_TEMPERATURE].threshold = DATA_THRESHOLD;

    return 0;
}

LOCAL STATUS deinit_ctrl(void)
{
    semDelete(ref_ctl.semRW);

    if (ref_ctl.d[DATA_SHOW_TEXT].d.s) {
        free(ref_ctl.d[DATA_SHOW_TEXT].d.s);
        ref_ctl.d[DATA_SHOW_TEXT].d.s = NULL;
    }
    
    return OK;    
}

LOCAL STATUS post_string(struct ref_ctl *ctl, wra_tm_handle data, const char* s, const char* name)
{
    int rc;

    rc = wra_tm_setvalue_string(data,WRA_TM_ATTR_DATA,(char*)s);
    if (rc != WRA_SUCCESS) {
        REF_LOG("%s failed to setstringvalue\n", __FUNCTION__);
        return ERROR;
    }

    rc = wra_tm_post_default(ctl->handle,data);
    if (rc != WRA_SUCCESS) {
        REF_LOG("%s failed to post_default\n", __FUNCTION__);
        return ERROR;
    }
    else
        return OK;
}

LOCAL STATUS post_float(struct ref_ctl *ctl, wra_tm_handle data, double *pTemp, const char* name)
{
    int rc;

    rc = wra_tm_setvalue_double(data,WRA_TM_ATTR_DATA,*pTemp);
    if (rc != WRA_SUCCESS) {
        REF_LOG("%s failed to set double value\n", __FUNCTION__);
        return ERROR;
    }

    rc = wra_tm_post_default(ctl->handle,data);
    if (rc != WRA_SUCCESS) {
        REF_LOG("%s failed to post_default\n", __FUNCTION__);
        return ERROR;
    }
    else
        return OK;
}

LOCAL STATUS post_data_item(struct ref_ctl *ctl, int index)
{
    STATUS ret = OK;

    switch(ctl->d[index].type) {
        case WRA_TM_DATATYPE_STRING:
            ret = post_string(ctl, ctl->data[index], ctl->d[index].d.s,
                               ctl->d[index].name);
            break;
        case WRA_TM_DATATYPE_DOUBLE:
            ret = post_float(ctl, ctl->data[index], &ctl->d[index].d.f,
                              ctl->d[index].name);
            break;
        default:
            break;
    }

    return ret;
}

LOCAL STATUS post_data_items(struct ref_ctl *ctl)
{
    int i;
    STATUS ret = OK;

    for (i = 0; i < sizeof(ctl->d)/sizeof(ctl->d[0]); i++) {
        ret += post_data_item(ctl, i);
    }

    return ret;
}

LOCAL STATUS post_alarm_item(struct ref_ctl *ctl, int index)
{
    int rc;
    double tp, th;
    wra_tm_handle alarm;

    if (!ctl || index >= ALARM_LAST)
        return ERROR;
 
    alarm = ctl->alarm[index];

    if (ctl->a[index].data >= DATA_LAST ||
        ctl->a[index].threshold >= DATA_LAST)
        return ERROR;

    semRTake(ref_ctl.semRW, WAIT_FOREVER);
    tp = ctl->d[ctl->a[index].data].d.f;
    th = ctl->d[ctl->a[index].threshold].d.f;
    semGive(ref_ctl.semRW);

    switch (ctl->d[ctl->a[index].data].type) {
        case WRA_TM_DATATYPE_DOUBLE:
            if ((tp - th) > 1e-5) {
                REF_LOG("Temperature %.2f exceeds threshold %.2f\n", tp, th);
                rc = wra_tm_setvalue_int(ctl->alarm[index],WRA_TM_ATTR_SEVERITY, REF_APP_ALARM_SEVERITY);
                if (rc != WRA_SUCCESS) {
                    REF_LOG("Failed to set alarm %s severity to %d\n", 
                            ctl->a[index].name, REF_APP_ALARM_SEVERITY);
                    return ERROR;
                }
                rc = wra_tm_setvalue_bool(ctl->alarm[index],WRA_TM_ATTR_ACTIVE,TRUE);
                if (rc != WRA_SUCCESS) {
                    REF_LOG("Failed to set alarm %s to active\n", 
                            ctl->a[index].name);
                    return ERROR;
                }
                rc = wra_tm_setaux(ctl->alarm[index],WRA_TM_ATTR_DATATM,ctl->data[DATA_TEMPERATURE]);
                if (rc != WRA_SUCCESS) {
                    REF_LOG("Failed to set alarm %s auxiliary data\n", 
                            ctl->a[index].name);
                    return ERROR;
                }
            }
            else
                return OK;
            break;
        case WRA_TM_DATATYPE_STRING:
        default:
            return ERROR;
    }
              
    rc = wra_tm_post_default(ctl->handle,alarm);
    if (rc != WRA_SUCCESS) {
        REF_LOG("%s failed to post_default\n", __FUNCTION__);
        return ERROR;
    }
    REF_LOG("Sent Alarm to EMS server\n");
    return OK;
}

LOCAL STATUS post_alarm_items(struct ref_ctl *ctl)
{
    int i;
    STATUS ret = OK;

    for (i = 0; i < sizeof(ctl->a)/sizeof(ctl->a[0]); i++) {
        ret += post_alarm_item(ctl, i);
    }

    return ret;
}

LOCAL void data_handling(struct ref_ctl *ctl)
{
    STATUS ret;
    int i;
    uint16_t val = 0;
    struct string_ops *strOps;
    struct analog_ops *analogOps;

    semRTake(ref_ctl.semRW, WAIT_FOREVER);
    for (i = 0; i < sizeof(ctl->data)/sizeof(ctl->data[0]); i++) {
        switch(ctl->d[i].type) {
            case WRA_TM_DATATYPE_STRING:
                strOps = (struct string_ops*)ctl->dops[i];
                if (!strOps)
                   continue;
                if (strOps->poutput)
                    ret = (*strOps->poutput)(ctl->d[i].d.s, strOps->col, strOps->row);
                break;
            case WRA_TM_DATATYPE_DOUBLE:
                analogOps = (struct analog_ops*)ctl->dops[i];
                if (!analogOps)
                   continue;
                if (analogOps->pread) {
                    ret = (*analogOps->pread)(analogOps->analogPin, &val);
                    if (ret != ERROR) {
                        if (analogOps->pconvert) {
                            ret = (*analogOps->pconvert)(val, &ctl->d[i].d.f);
                            if (ret == OK) {
                                if (analogOps->poutput)
                                    ret = (*analogOps->poutput)(&ctl->d[i].d.f,
                                             analogOps->col, analogOps->row);

                                if (analogOps->ppost)
                                    ret = (*analogOps->ppost)(ctl, i);
                            }
                        }
                    }
                    else
                        REF_LOG("failed to read analog from pin %d\n", analogOps->analogPin);
                }
                break;
            default:
                REF_LOG("no support for type %d\n", ctl->d[i].type);
                break;
        }
    }

    semGive(ref_ctl.semRW);
}

LOCAL void alarm_handling(struct ref_ctl *ctl)
{
    post_alarm_items(ctl);
}

/* if want to use A1 pin, use SENSOR_ANALOG_PIN */
LOCAL void ref_main_loop(struct ref_ctl *ctl)
{
	wra_timestamp tm;
    volatile int reading = 0;
    ref_app_status = APP_RUNNING;
    tm.tv_sec = 5;
    tm.tv_usec = 0;

    for(;;) {
        get_sensor_reading((int *)&reading);
        if (reading == 0)
            break;

        data_handling(ctl);
        alarm_handling(ctl);
        wra_action_wait(agent_h,&tm);
        /* sleep(REF_REPORT_RATE); */
    }

}

LOCAL STATUS initial_posting(struct ref_ctl *ctl)
{
    STATUS ret;

    if (ERROR == semRTake(ref_ctl.semRW, WAIT_FOREVER)) {
        REF_LOG("%s failed to take read/write lock\n", __FUNCTION__);
        return WRA_ERR_FAILED;
    }

    /* post data items to server */
    ret = post_data_items(ctl);
    ret += post_alarm_items(ctl);

    semGive(ref_ctl.semRW);

    return ret;
}

STATUS ref_main_task(void)
{
    STATUS ret = ERROR;

    init_ctrl();

    if (init_input_output(&ref_ctl) != OK)
        goto out;

    if (register_to_agent(&ref_ctl) != OK)
        goto out;

    if (ref_register_application_hook() != OK)
        goto out;

    /* enable sensor reading */
    set_sensor_reading(1);

    /* initial data posting */
    initial_posting(&ref_ctl);
#if 0
    /* main loop */
    ref_main_loop(&ref_ctl);
#endif
    ret = OK;
    return ret;
out:
    unregister_to_agent(&ref_ctl);
    deinit_ctrl();
    ref_app_status = APP_STOPPED;
    return ret;
}

#if defined(REF_HW_SUPPORT)
LOCAL STATUS init_input_output(struct ref_ctl *ctl)
{
    (void)ctl;

    if (ERROR == pinInit())
        return ERROR;

    if (ERROR == lcdInit())
        return ERROR;

    REF_DEBUG("%s\n", "Successfully initialized input and output");

    return OK;
}
#else
LOCAL STATUS init_input_output(struct ref_ctl *ctl)
{
    int i;
    unsigned int pins = 0;
    struct analog_ops *analogOps;

    for (i = 0; i < sizeof(ctl->data)/sizeof(ctl->data[0]); i++) {
        analogOps = (struct analog_ops*)ctl->dops[i];
        if (analogOps)
            pins += analogOps->analogPin;
    }

    srand(pins);

    REF_DEBUG("%s\n", "Successfully initialized input and output");

    return OK;
}
#endif

STATUS set_sensor_reading(int action)
{
    /* VX_MEM_BARRIER_W(); */
    if (action == 0)
       ref_ctl.ref_sensor_reading = 0;
    else
       ref_ctl.ref_sensor_reading = 1;

    return OK;
}

STATUS get_sensor_reading(int *action)
{
    if (!action)
        return ERROR;

    *action = ref_ctl.ref_sensor_reading;
    /* VX_MEM_BARRIER_R(); */

    return OK;
}

#define LCD_MAX_COL 16
#if defined(REF_HW_SUPPORT)
LOCAL STATUS lcd_output_string(const char * p, uint8_t col, uint8_t row)
{
    if (!p)
        return ERROR;

    lcdTake();

    lcdSetCursor(col, row);
    lcdPrint(p);

    lcdGive();

    return OK;

}

LOCAL STATUS lcd_output_temperature(const double * pTemp, uint8_t col, uint8_t row)
{
    char buffer[256];

    if (!pTemp)
        return ERROR;

    snprintf(buffer, sizeof(buffer) - 1,  "Temp:%.2fC", *pTemp);
    buffer [ sizeof(buffer) - 1 ] = 0;

    lcdTake();
    lcdSetCursor(col, row);
    lcdPrint(buffer);
    lcdGive();

    return OK;
}

LOCAL STATUS lm35v2_convert_temperature(uint16_t meter, double *pTemp)
{
    if (!pTemp)
        return ERROR;
#ifdef EXTRA_DEBUG
    REF_LOG("read back from sensor %hu\n", meter);
#endif    
    *pTemp = meter/8.192;
    *pTemp = ((*pTemp * 100.0) + 0.5) /100.0;
#ifdef EXTRA_DEBUG
    REF_LOG("temp is %.4f\n", *pTemp);
#endif    

    return OK;
}
#endif

/*
 * software generated fake temperature
 *
 */
LOCAL STATUS simulator_read_analog(uint8_t pin, uint16_t * val)
{
    int r;

    r = rand();
    if (val)
       *val = r & 0xffff;

    return OK;
}

LOCAL STATUS simulator_convert_temperature(uint16_t meter, double *pTemp)
{
    if (!pTemp)
        return ERROR;

#ifdef EXTRA_DEBUG
    REF_LOG("read back from software sensor %hu\n", meter);
#endif
    *pTemp = meter % REF_APP_SIM_TEMP_MAX;
#ifdef EXTRA_DEBUG
    REF_LOG("temp is %.4f\n", *pTemp);
#endif

    return OK;
}

LOCAL STATUS console_output_string(const char * p, uint8_t col, uint8_t row)
{
    (void)col, (void)row;

    if (!p)
        return ERROR;

    fprintf(stderr, ">>> %s\n", p);

    return OK;

}

LOCAL STATUS generic_change_string(const char **ps, const char * p)
{
    if (!ps)
        return ERROR;

    if (*ps)
        free((void*)*ps);

    if (!p)
        *ps = p;
    else {
        *ps = strdup(p);
        if (*ps == NULL)
            return ERROR;
    }

    return OK;

}

LOCAL STATUS generic_change_float(double *o, const double *n)
{
    if (!o || !n)
        return ERROR;

    *o = *n;

    return OK;

}

LOCAL STATUS console_output_temperature(const double * pTemp, uint8_t col, uint8_t row)
{
    char buffer[256];

    if (!pTemp)
        return ERROR;

    snprintf(buffer, sizeof(buffer) - 1,  "Temp:%.2fC", *pTemp);
    buffer [ sizeof(buffer) - 1 ] = 0;

    fprintf(stderr, ">>> %s\n\n", buffer);

    return OK;
}

LOCAL STATUS register_to_agent(struct ref_ctl * ctl)
{
    int i;
    int rc = WRA_ERR_FAILED;

    ctl->handle = agent_h;
    if (ctl->handle == WRA_NULL) {
        REF_LOG("Failed to get the api handle\n");
        return WRA_ERR_FAILED;
    }

    for (i = 0; i < sizeof(ctl->data)/sizeof(ctl->data[0]); i++) {
        ctl->data[i] = wra_tm_create(WRA_TM_DATATM,ctl->d[i].name);
        if (ctl->data[i] == WRA_NULL)
            goto err;
    }

    for (i = 0; i < sizeof(ctl->alarm)/sizeof(ctl->alarm[0]); i++) {
        ctl->alarm[i] = wra_tm_create(WRA_TM_ALARMTM, ctl->a[i].name);
        if (ctl->alarm[i] == WRA_NULL)
            goto err;
        rc = wra_tm_setvalue_string(ctl->alarm[i],WRA_TM_ATTR_DESC,ctl->a[i].description);
        if (rc != WRA_SUCCESS)
            goto err;
        rc = wra_tm_setvalue_bool(ctl->alarm[i],WRA_TM_ATTR_ACTIVE,FALSE);
        if (rc != WRA_SUCCESS)
            goto err;
    }

    REF_DEBUG("%s\n", "Successfully registered to WRA agent");

    return OK;
err:
    REF_DEBUG("%s\n", "Failed to register to WRA agent");

    unregister_to_agent(ctl);
    return ERROR;
}

LOCAL STATUS unregister_to_agent(struct ref_ctl * ctl)
{
    int i;

    ctl->handle = agent_h;
    if (ctl->handle == WRA_NULL) {
        REF_LOG("Failed to get the api handle\n");
        return WRA_ERR_FAILED;
    }

    for (i = 0; i < sizeof(ctl->data)/sizeof(ctl->data[0]); i++) {
        if (ctl->data[i]) {
            wra_tm_destroy(ctl->data[i]);
            ctl->data[i] = NULL;
        }
    }

    for (i = 0; i < sizeof(ctl->alarm)/sizeof(ctl->alarm[0]); i++) {
        if (ctl->alarm[i]) {
            wra_tm_destroy(ctl->alarm[i]);
            ctl->alarm[i] = NULL;
        }
    }
    wra_delete_handle(ctl->handle);
    return OK;
}

/*#define TEST_APP_HANDLER */
struct ref_apps
{
    char * name;
    int (*handler)(void*, const char*, const char*);
    enum ref_data_item item;
};

int ref_app_lcd_handler(void *device_handle, const char *appname,
                       const char *args);
int ref_app_threshold_handler(void *device_handle, const char *appname,
                              const char *args);
int ref_app_alarm_handler(void *device_handle, const char *appname,
                          const char *args);
LOCAL struct ref_apps ref_app_reg[] = {
    {"data_show", &ref_app_lcd_handler, DATA_SHOW_TEXT},
    {"set_threshold", &ref_app_threshold_handler, DATA_THRESHOLD},
    {"ack_alarm", &ref_app_alarm_handler, DATA_TEMPERATURE},
};

int ref_app_lcd_handler(void *device_handle, const char *appname,
                       const char *args)
{
    (void)device_handle;
    struct string_ops *ops;
    char space[LCD_MAX_COL + 1];

    if (!appname || !args) {
        REF_LOG("%s is called with empty arg: appname:\"%s\" args:\"%s\"\n",
                __FUNCTION__, appname?appname:"", args?args:"");
        return WRA_ERR_BAD_PARAM;
    }

    if (strncmp(appname, "data_show", sizeof("data_show")))
        return WRA_ERR_BAD_PARAM;

    REF_DEBUG("%s reached: %s %s\n", __FUNCTION__, appname, args);

    memset(space, ' ', sizeof(space) - 1);
    space[sizeof(space) - 1] = 0;

    ops = (struct string_ops *)ref_ctl.dops[DATA_SHOW_TEXT];
    if (!ops)
        return WRA_SUCCESS;

    if (ERROR == semWTake(ref_ctl.semRW, WAIT_FOREVER)) {
        REF_LOG("%s failed to take read/write lock\n", __FUNCTION__);
        return WRA_ERR_FAILED;
    }

    if (ops->pchange)
        (*ops->pchange)((const char**)&ref_ctl.d[DATA_SHOW_TEXT].d.s, args);

    if (ops->poutput) {
        /* clear the row */
        (*ops->poutput)(space, ops->col, ops->row);
        /* output */
        (*ops->poutput)(args, ops->col, ops->row);
    }

    semGive(ref_ctl.semRW);

#if defined(REF_REPORT_BACK)
    if (ERROR == semRTake(ref_ctl.semRW, WAIT_FOREVER)) {
        REF_LOG("%s failed to take read/write lock\n", __FUNCTION__);
        return WRA_ERR_FAILED;
    }

    if (ops->ppost) {
        (*ops->ppost)(&ref_ctl, DATA_SHOW_TEXT);
    }

    semGive(ref_ctl.semRW);
#endif

    return WRA_SUCCESS;
}

int ref_app_threshold_handler(void *device_handle, const char *appname,
                              const char *args)
{
    (void)device_handle;
    struct analog_ops *ops;
    double newf;

    if (!appname || !args) {
        REF_LOG("%s is called with empty arg: appname:\"%s\" args:\"%s\"\n",
                __FUNCTION__, appname?appname:"", args?args:"");
        return WRA_ERR_BAD_PARAM;
    }

    if (strncmp(appname, "set_threshold", sizeof("set_threshold")))
        return WRA_ERR_BAD_PARAM;

    REF_DEBUG("%s reached: %s %s\n", __FUNCTION__, appname, args);

    ops = (struct analog_ops *)ref_ctl.dops[DATA_THRESHOLD];
    if (!ops)
        return WRA_SUCCESS;

    newf = strtod(args, NULL);
    if (ERROR == semWTake(ref_ctl.semRW, WAIT_FOREVER)) {
        REF_LOG("%s failed to take read/write lock\n", __FUNCTION__);
        return WRA_ERR_FAILED;
    }

    if (ops->pchange)
        (*ops->pchange)(&ref_ctl.d[DATA_THRESHOLD].d.f, &newf);

    semGive(ref_ctl.semRW);

#if defined(REF_REPORT_BACK)
    if (ERROR == semRTake(ref_ctl.semRW, WAIT_FOREVER)) {
        REF_LOG("%s failed to take read/write lock\n", __FUNCTION__);
        return WRA_ERR_FAILED;
    }

    if (ops->ppost) {
        (*ops->ppost)(&ref_ctl, DATA_THRESHOLD);
    }

    semGive(ref_ctl.semRW);
#endif
    return WRA_SUCCESS;
}

int ref_app_alarm_handler(void *device_handle, const char *appname,
                          const char *args)
{
    (void)device_handle;

    if (!appname || !args) {
        REF_LOG("%s is called with empty arg: appname:\"%s\" args:\"%s\"\n",
                __FUNCTION__, appname?appname:"", args?args:"");
        return WRA_ERR_BAD_PARAM;
    }

    if (strncmp(appname, "ack_alarm", sizeof("ack_alarm")))
        return WRA_ERR_BAD_PARAM;

    REF_DEBUG("%s reached: %s %s\n", __FUNCTION__, appname, args);

    return WRA_SUCCESS;
}

LOCAL STATUS ref_register_application_hook(void)
{
    int ret, i;
    wra_handle handle;

    handle = agent_h;
    if (handle == WRA_NULL) {
        REF_LOG("%s failed to get the api handle\n", __FUNCTION__);
        return WRA_ERR_FAILED;
    }

    for (i = 0; i < sizeof(ref_app_reg)/sizeof(ref_app_reg[0]); i++) {
        ret = wra_action_subscribe(handle, ref_app_reg[i].handler,
                                     ref_app_reg[i].name);
        if (ret != WRA_SUCCESS) {
            REF_LOG("Failed to register %s handler.\n", ref_app_reg[i].name);
            return ret;
        }
    }

    REF_DEBUG("%s\n", "Successfully register application hooks");

    return OK;
}

int ref_app_demo_handler(void *device_handle, const char *appname,
                          const char *args)
{
    (void)device_handle;

    if (!appname) {
        REF_LOG("%s is called with empty appname:\"%s\" \n",
                __FUNCTION__, appname?appname:"" );
        return WRA_ERR_BAD_PARAM;
    }

    if (strncmp(appname, REF_APP_NAME, sizeof(REF_APP_NAME)))
        return WRA_ERR_BAD_PARAM;

    REF_DEBUG("%s reached: %s \"%s\"\n", __FUNCTION__, appname, args?args:"");

    (void)args;
    if (ref_app_status == APP_RUNNING) {
        REF_DEBUG("Reference app already running.\n");
        return WRA_ERR_EXISTS;
    }
    ref_app_status = APP_NOT_INIT;
    
    if ( ref_main_task() != OK)
    {
        printf("Failed to run reference app\n");
        return WRA_ERR_FAILED;
    }

    return WRA_SUCCESS;
}

STATUS ref_register_application_self(void)
{
    int ret;
    wra_handle handle = NULL;

    while (!handle) {
        /* Retry until agent initializes */
        handle = wra_gethandle();
        sleep(1);
    }
    if (handle == WRA_NULL) {
        REF_LOG("%s failed to get the api handle\n", __FUNCTION__);
        return WRA_ERR_FAILED;
    }
    agent_h = handle;
    ret = wra_action_subscribe(handle, ref_app_demo_handler,
                                 REF_APP_NAME);
    if (ret != WRA_SUCCESS) {
        REF_LOG("Failed to register handler for app  %s  (ret = %d).\n", 
                REF_APP_NAME, ret);
        return ret;
    }
    REF_LOG("%s :: Successfully registered the demo app handler\n",
            __FUNCTION__);

    return wra_action_wait(handle,NULL);
}

/* Registration Function for Reference App 
 * MUST be called after the agent initialized
 */

void register_demo_app(void) {

	/* register reference demo app */
    if (ref_register_application_self() == WRA_SUCCESS)
    {
        ref_main_loop(&ref_ctl);
    }else
        printf("Fail to Start reference application \n");
}
