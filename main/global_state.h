#ifndef GLOBAL_STATE_H_
#define GLOBAL_STATE_H_

#include <stdbool.h>
#include <stdint.h>
#include "asic_task.h"
#include "bm1366.h"
#include "common.h"
#include "power_management_task.h"
#include "serial.h"
#include "stratum_api.h"
#include "work_queue.h"

#define STRATUM_USER CONFIG_STRATUM_USER
#define FALLBACK_STRATUM_USER CONFIG_FALLBACK_STRATUM_USER

#define HISTORY_LENGTH 512
#define DIFF_STRING_SIZE 10

typedef enum
{
    DEVICE_UNKNOWN = -1,
    DEVICE_HEX
} DeviceModel;

typedef enum
{
    ASIC_UNKNOWN = -1,
    ASIC_BM1366
} AsicModel;

typedef struct
{
    uint8_t (*init_fn)(uint64_t, uint16_t);
    task_result * (*receive_result_fn)(void * GLOBAL_STATE);
    int (*set_max_baud_fn)(void);
    void (*set_difficulty_mask_fn)(int);
    void (*send_work_fn)(void * GLOBAL_STATE, bm_job * next_bm_job);
    bool (*send_hash_frequency_fn)(float);
} AsicFunctions;

typedef struct
{
    double duration_start;
    int historical_hashrate_rolling_index;
    double historical_hashrate_time_stamps[HISTORY_LENGTH];
    double historical_hashrate[HISTORY_LENGTH];
    int historical_hashrate_init;
    double current_hashrate;
    double current_hashrate_10m;
    double current_hashrate_1h;
    double current_hashrate_1d;
    int64_t start_time;
    uint64_t shares_accepted;
    uint64_t shares_rejected;
    int screen_page;
    char oled_buf[20];
    uint64_t best_nonce_diff;
    char best_diff_string[DIFF_STRING_SIZE];
    uint64_t best_session_nonce_diff;
    char best_session_diff_string[DIFF_STRING_SIZE];
    bool FOUND_BLOCK;
    bool startup_done;
    char ssid[32];
    char wifi_status[20];
    char * pool_url;
    char * fallback_pool_url;
    uint16_t pool_port;
    uint16_t fallback_pool_port;
    bool is_using_fallback;
    uint16_t overheat_mode;

    uint32_t lastClockSync;
} SystemModule;

typedef struct
{
    DeviceModel device_model;
    char * device_model_str;
    int board_version;
    AsicModel asic_model;
    char * asic_model_str;
    uint16_t asic_count;
    uint16_t voltage_domain;
    AsicFunctions ASIC_functions;
    double asic_job_frequency_ms;
    uint32_t initial_ASIC_difficulty;

    work_queue stratum_queue;
    work_queue ASIC_jobs_queue;

    SystemModule SYSTEM_MODULE;
    AsicTaskModule ASIC_TASK_MODULE;
    PowerManagementModule POWER_MANAGEMENT_MODULE;

    char * extranonce_str;
    int extranonce_2_len;
    int abandon_work;

    uint8_t * valid_jobs;
    pthread_mutex_t valid_jobs_lock;

    uint32_t stratum_difficulty;
    uint32_t version_mask;

    int sock;

} GlobalState;

#endif /* GLOBAL_STATE_H_ */
