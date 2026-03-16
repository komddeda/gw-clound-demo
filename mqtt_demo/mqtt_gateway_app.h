#ifndef MQTT_GATEWAY_APP_H
#define MQTT_GATEWAY_APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <signal.h>
#include <pthread.h>

#include "MQTTClient.h"
#include "cJSON.h"

#ifndef LOG_LEVEL
#define LOG_LEVEL 3
#endif

void log_write(const char *level, const char *fmt, ...);

#define LOGE(fmt, ...) do { if (LOG_LEVEL >= 1) log_write("E", fmt, ##__VA_ARGS__); } while (0)
#define LOGW(fmt, ...) do { if (LOG_LEVEL >= 2) log_write("W", fmt, ##__VA_ARGS__); } while (0)
#define LOGI(fmt, ...) do { if (LOG_LEVEL >= 3) log_write("I", fmt, ##__VA_ARGS__); } while (0)
#define LOGD(fmt, ...) do { if (LOG_LEVEL >= 4) log_write("D", fmt, ##__VA_ARGS__); } while (0)

typedef enum {
    TAIL_FORMAT_AUTO = 0,
    TAIL_FORMAT_TEXT = 1,
    TAIL_FORMAT_JSON = 2,
    TAIL_FORMAT_XML  = 3,
} tail_format_t;

typedef enum {
    TEST_VALUE_INT = 0,
    TEST_VALUE_FLOAT = 1,
    TEST_VALUE_BOOL = 2,
    TEST_VALUE_STRING = 3,
    TEST_VALUE_JSON = 4,
} test_value_type_t;

typedef enum {
    TEST_PATTERN_FIXED = 0,
    TEST_PATTERN_INC = 1,
    TEST_PATTERN_DEC = 2,
} test_pattern_t;

typedef struct {
    char broker[256];
    char client_id[64];
    char device_id[64];
    char username[64];
    char password[64];
    char cafile[256];
    char certfile[256];
    char keyfile[256];
    char ctrl_path[256];
    int keepalive_s;
    int clean_session;
    bool ssl_enable;
    bool ssl_insecure;

    bool tail_enable;
    int tail_poll_ms;
    int tail_idle_ms;
    char data_dir[256];
    char data_ext[16];
    tail_format_t tail_format;

    bool telemetry_enable;
    int telemetry_interval_ms;
    char telemetry_disk_path[256];
    char telemetry_temp_path[256];

    bool test_mode_enable;
    int test_interval_ms;
    test_value_type_t test_value_type;
    test_pattern_t test_pattern;
    char test_value[256];
    double test_step;
    char cpu_serial[64];
} app_cfg_t;

typedef struct {
    app_cfg_t cfg;
    MQTTClient mqtt;
    pthread_mutex_t mqtt_lock;
    bool mqtt_connected;

    pthread_t tail_thread;
    pthread_t telemetry_thread;
    pthread_t test_thread;

    char topic_cmd_down[128];
    char topic_cmd_resp[128];
    char topic_cfg_down[128];
    char topic_cfg_resp[128];
    char topic_status[128];
    char topic_data_up[128];
    char topic_test_up[128];
    char topic_test_down[128];
} app_t;

extern volatile sig_atomic_t g_running;

void sleep_ms(int ms);
uint64_t monotonic_ms(void);
void format_iso8601(char *buf, size_t cap, int64_t *epoch_ms);

void cfg_defaults(app_cfg_t *cfg);
void build_topics(app_t *app);
void parse_args_full(app_cfg_t *cfg, int argc, char **argv);

int save_ctrl_config_file(const app_t *app, cJSON *root);

int mqtt_publish_json(app_t *app, const char *topic, int qos, int retained, const char *payload);
void publish_status(app_t *app, const char *status, int heartbeat_interval_ms);
int mqtt_message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void mqtt_connection_lost(void *context, char *cause);
void mqtt_delivery_complete(void *context, MQTTClient_deliveryToken token);
int mqtt_connect_and_subscribe(app_t *app);

void *file_tailer_main(void *arg);
void *telemetry_publisher_main(void *arg);
void *test_publisher_main(void *arg);

#endif
