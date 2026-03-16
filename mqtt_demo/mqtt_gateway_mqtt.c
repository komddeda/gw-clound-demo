#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/wait.h>
#include <unistd.h>

#include "mqtt_gateway_app.h"

static const char *k_user_buzzer_path = "/dev/user-buzzer";
static const char *k_user_led_path = "/dev/user-led";

typedef struct {
    char req_id[64];
    char action[32];
    char target[32];
} cmd_meta_t;

static bool has_prefix(const char *s, const char *p) {
    size_t lp = strlen(p);
    return strncmp(s, p, lp) == 0;
}

static int write_toggle_file(const char *path, int val) {
    FILE *fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "%d\n", val ? 1 : 0);
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
    return 0;
}

static bool json_get_boolish(cJSON *item, int *out) {
    if (cJSON_IsBool(item)) {
        *out = cJSON_IsTrue(item) ? 1 : 0;
        return true;
    }
    if (cJSON_IsNumber(item)) {
        *out = item->valueint ? 1 : 0;
        return true;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        const char *s = item->valuestring;
        if (strcasecmp(s, "1") == 0 || strcasecmp(s, "true") == 0 ||
            strcasecmp(s, "on") == 0 || strcasecmp(s, "yes") == 0) {
            *out = 1;
            return true;
        }
        if (strcasecmp(s, "0") == 0 || strcasecmp(s, "false") == 0 ||
            strcasecmp(s, "off") == 0 || strcasecmp(s, "no") == 0) {
            *out = 0;
            return true;
        }
    }
    return false;
}

static bool str_eq_any(const char *s, const char *const *choices, size_t n) {
    if (!s || !*s) return false;
    for (size_t i = 0; i < n; i++) {
        if (choices[i] && strcasecmp(s, choices[i]) == 0) return true;
    }
    return false;
}

static bool resolve_named_target(const char *name, const char **target_name, const char **target_path) {
    static const char *k_led_aliases[] = {"led", "set_led", "lamp", "user_led"};
    static const char *k_buzzer_aliases[] = {"buzzer", "set_buzzer", "beeper", "beep"};

    if (str_eq_any(name, k_led_aliases, sizeof(k_led_aliases) / sizeof(k_led_aliases[0]))) {
        *target_name = "led";
        *target_path = k_user_led_path;
        return true;
    }
    if (str_eq_any(name, k_buzzer_aliases, sizeof(k_buzzer_aliases) / sizeof(k_buzzer_aliases[0]))) {
        *target_name = "buzzer";
        *target_path = k_user_buzzer_path;
        return true;
    }
    return false;
}

static bool resolve_power_action(const char *action, const char **command) {
    if (!action || !*action || !command) return false;
    if (strcasecmp(action, "shutdown") == 0 || strcasecmp(action, "poweroff") == 0) {
        *command = "sleep 2; sync; (systemctl poweroff || poweroff || shutdown -h now) >/dev/null 2>&1";
        return true;
    }
    if (strcasecmp(action, "suspend") == 0 || strcasecmp(action, "sleep") == 0) {
        *command = "sleep 2; sync; (systemctl suspend || sh -c 'echo mem > /sys/power/state') >/dev/null 2>&1";
        return true;
    }
    return false;
}

static int launch_detached_shell(const char *command) {
    if (!command || !*command) return -1;
    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        if (setsid() < 0) _exit(127);
        pid_t grandchild = fork();
        if (grandchild < 0) _exit(127);
        if (grandchild > 0) _exit(0);
        execl("/bin/sh", "sh", "-c", command, (char *)NULL);
        _exit(127);
    }
    (void)waitpid(pid, NULL, 0);
    return 0;
}

static void fill_cmd_meta(cJSON *root, cmd_meta_t *meta) {
    if (!meta) return;
    memset(meta, 0, sizeof(*meta));
    if (!root || !cJSON_IsObject(root)) return;

    cJSON *req = cJSON_GetObjectItemCaseSensitive(root, "req_id");
    if (cJSON_IsString(req) && req->valuestring) {
        snprintf(meta->req_id, sizeof(meta->req_id), "%s", req->valuestring);
    }

    cJSON *act = cJSON_GetObjectItemCaseSensitive(root, "action");
    if (cJSON_IsString(act) && act->valuestring) {
        snprintf(meta->action, sizeof(meta->action), "%s", act->valuestring);
    }

    cJSON *target = cJSON_GetObjectItemCaseSensitive(root, "target");
    if (!cJSON_IsString(target) || !target->valuestring) {
        target = cJSON_GetObjectItemCaseSensitive(root, "name");
    }
    if (cJSON_IsString(target) && target->valuestring) {
        snprintf(meta->target, sizeof(meta->target), "%s", target->valuestring);
    }
}

static void publish_command_response(app_t *app,
                                     const cmd_meta_t *meta,
                                     const char *status,
                                     const char *error,
                                     const char *target,
                                     int val) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    char ts[32];
    int64_t epoch_ms = 0;
    format_iso8601(ts, sizeof(ts), &epoch_ms);

    cJSON_AddStringToObject(root, "ts", ts);
    cJSON_AddNumberToObject(root, "epoch_ms", (double)epoch_ms);
    cJSON_AddStringToObject(root, "req_id", (meta && meta->req_id[0]) ? meta->req_id : "");
    cJSON_AddStringToObject(root, "action", (meta && meta->action[0]) ? meta->action : "");
    cJSON_AddStringToObject(root, "status", status ? status : "error");
    cJSON_AddStringToObject(root, "data_class", "control");
    cJSON_AddStringToObject(root, "thread_kind", "remote_control");
    if (error && *error) cJSON_AddStringToObject(root, "error", error);
    if (target && *target) cJSON_AddStringToObject(root, "target", target);
    if (val >= 0) cJSON_AddNumberToObject(root, "val", val);

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return;

    (void)mqtt_publish_json(app, app->topic_cmd_resp, 1, 0, payload);
    cJSON_free(payload);
}

static void publish_config_response(app_t *app, const char *status, const char *error) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    char ts[32];
    int64_t epoch_ms = 0;
    format_iso8601(ts, sizeof(ts), &epoch_ms);

    cJSON_AddStringToObject(root, "ts", ts);
    cJSON_AddNumberToObject(root, "epoch_ms", (double)epoch_ms);
    cJSON_AddStringToObject(root, "device_id", app->cfg.device_id);
    cJSON_AddStringToObject(root, "status", status ? status : "error");
    cJSON_AddStringToObject(root, "data_class", "control");
    cJSON_AddStringToObject(root, "thread_kind", "remote_control");
    cJSON_AddStringToObject(root, "ctrl_file", app->cfg.ctrl_path);
    if (error && *error) cJSON_AddStringToObject(root, "error", error);

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return;

    (void)mqtt_publish_json(app, app->topic_cfg_resp, 1, 0, payload);
    cJSON_free(payload);
}

static int parse_local_command(cJSON *root,
                               const cmd_meta_t *meta,
                               const char **path,
                               char *target,
                               size_t target_cap,
                               int *val,
                               const char **error) {
    if (!root || !cJSON_IsObject(root)) {
        if (error) *error = "bad_json";
        return -1;
    }
    if (!meta || !meta->action[0]) {
        if (error) *error = "missing_action";
        return -1;
    }
    if (!path || !target || target_cap == 0 || !val) {
        if (error) *error = "bad_state";
        return -1;
    }

    const char *target_name = NULL;
    const char *target_path = NULL;

    if (meta->target[0]) {
        (void)resolve_named_target(meta->target, &target_name, &target_path);
    }

    if (!target_name) {
        (void)resolve_named_target(meta->action, &target_name, &target_path);
    }

    if (!target_name &&
        (strcasecmp(meta->action, "write_coil") == 0 ||
         strcasecmp(meta->action, "write") == 0 ||
         strcasecmp(meta->action, "coil") == 0 ||
         strcasecmp(meta->action, "set_do") == 0 ||
         strcasecmp(meta->action, "do") == 0)) {
        cJSON *addr = cJSON_GetObjectItemCaseSensitive(root, "addr");
        if (!cJSON_IsNumber(addr)) {
            if (error) *error = "missing_addr";
            return -1;
        }
        if (addr->valueint == 0 || addr->valueint == 10) {
            target_name = "buzzer";
            target_path = k_user_buzzer_path;
        } else if (addr->valueint == 1 || addr->valueint == 11) {
            target_name = "led";
            target_path = k_user_led_path;
        } else {
            if (error) *error = "modbus_direct_control_removed";
            return 0;
        }
    }

    if (!target_name || !target_path) {
        if (error) *error = "modbus_direct_control_removed";
        return 0;
    }

    cJSON *item_val = cJSON_GetObjectItemCaseSensitive(root, "val");
    if (!json_get_boolish(item_val, val)) {
        if (error) *error = "missing_val";
        return -1;
    }

    snprintf(target, target_cap, "%s", target_name);
    *path = target_path;
    return 1;
}

int mqtt_publish_json(app_t *app, const char *topic, int qos, int retained, const char *payload) {
    if (!topic || !payload) return -1;
    pthread_mutex_lock(&app->mqtt_lock);
    bool ok = app->mqtt_connected;
    pthread_mutex_unlock(&app->mqtt_lock);
    if (!ok) return -1;

    MQTTClient_message msg = MQTTClient_message_initializer;
    msg.payload = (void *)payload;
    msg.payloadlen = (int)strlen(payload);
    msg.qos = qos;
    msg.retained = retained;

    MQTTClient_deliveryToken token;

    pthread_mutex_lock(&app->mqtt_lock);
    int rc = MQTTClient_publishMessage(app->mqtt, topic, &msg, &token);
    pthread_mutex_unlock(&app->mqtt_lock);
    if (rc != MQTTCLIENT_SUCCESS) return rc;

    if (qos > 0) {
        MQTTClient_waitForCompletion(app->mqtt, token, 3000L);
    }
    return MQTTCLIENT_SUCCESS;
}

void publish_status(app_t *app, const char *status, int heartbeat_interval_ms) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    if (heartbeat_interval_ms < 500) heartbeat_interval_ms = 500;
    char ts[32];
    int64_t epoch_ms = 0;
    format_iso8601(ts, sizeof(ts), &epoch_ms);
    cJSON_AddStringToObject(root, "ts", ts);
    cJSON_AddNumberToObject(root, "epoch_ms", (double)epoch_ms);
    cJSON_AddStringToObject(root, "status", status);
    cJSON_AddStringToObject(root, "event_type", strcmp(status ? status : "", "online") == 0 ? "online" : "status");
    cJSON_AddBoolToObject(root, "online", strcmp(status ? status : "", "online") == 0);
    cJSON_AddStringToObject(root, "data_class", "status");
    cJSON_AddStringToObject(root, "thread_kind", "device_status");
    cJSON_AddStringToObject(root, "device_id", app->cfg.device_id);
    cJSON_AddStringToObject(root, "client_id", app->cfg.client_id);
    cJSON_AddBoolToObject(root, "telemetry_enable", app->cfg.telemetry_enable);
    cJSON_AddBoolToObject(root, "test_mode_enable", app->cfg.test_mode_enable);
    cJSON_AddNumberToObject(root, "telemetry_interval_ms", heartbeat_interval_ms);
    cJSON_AddNumberToObject(root, "heartbeat_interval_ms", heartbeat_interval_ms);
    cJSON_AddNumberToObject(root, "test_interval_ms", app->cfg.test_interval_ms);
    cJSON_AddStringToObject(root, "telemetry_topic", app->topic_data_up);
    cJSON_AddStringToObject(root, "test_topic", app->topic_test_up);
    cJSON_AddStringToObject(root, "command_down_topic", app->topic_cmd_down);
    cJSON_AddStringToObject(root, "config_down_topic", app->topic_cfg_down);
    cJSON_AddStringToObject(root, "command_resp_topic", app->topic_cmd_resp);
    cJSON_AddStringToObject(root, "config_resp_topic", app->topic_cfg_resp);
    cJSON *controls = cJSON_AddArrayToObject(root, "controls");
    if (controls) {
        cJSON_AddItemToArray(controls, cJSON_CreateString("led"));
        cJSON_AddItemToArray(controls, cJSON_CreateString("buzzer"));
        cJSON_AddItemToArray(controls, cJSON_CreateString("write_coil"));
        cJSON_AddItemToArray(controls, cJSON_CreateString("write"));
        cJSON_AddItemToArray(controls, cJSON_CreateString("shutdown"));
        cJSON_AddItemToArray(controls, cJSON_CreateString("suspend"));
    }
    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return;
    (void)mqtt_publish_json(app, app->topic_status, 1, 1, payload);
    cJSON_free(payload);
}

static void print_test_down_payload(const char *payload, size_t payload_len) {
    if (!payload) {
        LOGI("test down received: (null)");
        return;
    }

    cJSON *root = cJSON_ParseWithLength(payload, payload_len);
    if (root && cJSON_IsObject(root)) {
        cJSON *data = cJSON_GetObjectItemCaseSensitive(root, "data");
        if (data) {
            if (cJSON_IsString(data) && data->valuestring) {
                LOGI("test down received: %s", data->valuestring);
            } else {
                char *printed = cJSON_PrintUnformatted(data);
                if (printed) {
                    LOGI("test down received: %s", printed);
                    cJSON_free(printed);
                } else {
                    LOGI("test down received: %s", payload);
                }
            }
            cJSON_Delete(root);
            return;
        }
    }

    if (root) cJSON_Delete(root);
    LOGI("test down received: %s", payload[0] ? payload : "(empty)");
}

int mqtt_message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    app_t *app = (app_t *)context;

    char topic_buf[256];
    const char *topic = topicName;
    if (topicLen > 0) {
        size_t n = (size_t)topicLen;
        if (n >= sizeof(topic_buf)) n = sizeof(topic_buf) - 1;
        memcpy(topic_buf, topicName, n);
        topic_buf[n] = '\0';
        topic = topic_buf;
    }

    size_t plen = (size_t)message->payloadlen;
    char *payload = (char *)malloc(plen + 1u);
    if (payload) {
        memcpy(payload, message->payload, plen);
        payload[plen] = '\0';
    }

    if (payload && !strcmp(topic, app->topic_cmd_down)) {
        cJSON *root = cJSON_ParseWithLength(payload, plen);
        cmd_meta_t meta;
        fill_cmd_meta(root, &meta);
        if (!root || !cJSON_IsObject(root)) {
            LOGW("command JSON parse failed");
            publish_command_response(app, &meta, "error", "bad_json", NULL, -1);
        } else {
            const char *power_command = NULL;
            if (resolve_power_action(meta.action, &power_command)) {
                if (launch_detached_shell(power_command) != 0) {
                    LOGW("power action schedule failed: %s", meta.action);
                    publish_command_response(app, &meta, "error", "schedule_failed", "system", -1);
                } else {
                    LOGI("system power action scheduled: %s", meta.action);
                    publish_command_response(app, &meta, "ok", NULL, "system", -1);
                }
            } else {
                const char *path = NULL;
                const char *error = NULL;
                char target[16] = {0};
                int val = -1;
                int kind = parse_local_command(root, &meta, &path, target, sizeof(target), &val, &error);
                if (kind > 0) {
                    if (write_toggle_file(path, val) != 0) {
                        LOGW("device action failed: %s (%s)", target, strerror(errno));
                        publish_command_response(app, &meta, "error", "apply_failed", target, val);
                    } else {
                        LOGI("device action applied: %s=%d", target, val ? 1 : 0);
                        publish_command_response(app, &meta, "ok", NULL, target, val);
                    }
                } else {
                    LOGW("ignore command, unsupported action: %s", meta.action[0] ? meta.action : "(empty)");
                    publish_command_response(app, &meta, "error", error ? error : "unsupported_action", NULL, -1);
                }
            }
            cJSON_Delete(root);
        }
    } else if (payload && !strcmp(topic, app->topic_cfg_down)) {
        cJSON *root = cJSON_ParseWithLength(payload, plen);
        if (root && cJSON_IsObject(root)) {
            int rc = save_ctrl_config_file(app, root);
            if (rc > 0) {
                LOGI("modbus ctrl config saved: %s", app->cfg.ctrl_path);
                publish_config_response(app, "ok", NULL);
            } else if (rc == 0) {
                LOGW("config ignored: no modbus-related fields found");
                publish_config_response(app, "ignored", "no_supported_fields");
            } else {
                LOGW("config save failed: %s", app->cfg.ctrl_path);
                publish_config_response(app, "error", "save_failed");
            }
            cJSON_Delete(root);
        } else {
            if (root) cJSON_Delete(root);
            LOGW("config JSON parse failed");
            publish_config_response(app, "error", "bad_json");
        }
    } else if (payload && !strcmp(topic, app->topic_test_down)) {
        print_test_down_payload(payload, plen);
    } else {
        LOGD("message ignored: %s", topic ? topic : "(null)");
    }

    if (payload) free(payload);
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void mqtt_connection_lost(void *context, char *cause) {
    app_t *app = (app_t *)context;
    pthread_mutex_lock(&app->mqtt_lock);
    app->mqtt_connected = false;
    pthread_mutex_unlock(&app->mqtt_lock);
    LOGW("mqtt connection lost: %s", cause ? cause : "unknown");
}

void mqtt_delivery_complete(void *context, MQTTClient_deliveryToken token) {
    (void)context;
    (void)token;
}

int mqtt_connect_and_subscribe(app_t *app) {
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;

    conn_opts.keepAliveInterval = app->cfg.keepalive_s;
    conn_opts.cleansession = app->cfg.clean_session;
    if (app->cfg.username[0]) conn_opts.username = app->cfg.username;
    if (app->cfg.password[0]) conn_opts.password = app->cfg.password;
    if (app->cfg.ssl_enable || has_prefix(app->cfg.broker, "ssl://")) {
        if (app->cfg.cafile[0]) ssl_opts.trustStore = app->cfg.cafile;
        if (app->cfg.certfile[0]) ssl_opts.keyStore = app->cfg.certfile;
        if (app->cfg.keyfile[0]) ssl_opts.privateKey = app->cfg.keyfile;
        ssl_opts.enableServerCertAuth = app->cfg.ssl_insecure ? 0 : 1;
        conn_opts.ssl = &ssl_opts;
    }

    pthread_mutex_lock(&app->mqtt_lock);
    int rc = MQTTClient_connect(app->mqtt, &conn_opts);
    pthread_mutex_unlock(&app->mqtt_lock);

    if (rc != MQTTCLIENT_SUCCESS) return rc;

    pthread_mutex_lock(&app->mqtt_lock);
    MQTTClient_subscribe(app->mqtt, app->topic_cmd_down, 1);
    MQTTClient_subscribe(app->mqtt, app->topic_cfg_down, 1);
    MQTTClient_subscribe(app->mqtt, app->topic_test_down, 1);
    app->mqtt_connected = true;
    pthread_mutex_unlock(&app->mqtt_lock);

    publish_status(app, "online", app->cfg.telemetry_interval_ms);

    return MQTTCLIENT_SUCCESS;
}
