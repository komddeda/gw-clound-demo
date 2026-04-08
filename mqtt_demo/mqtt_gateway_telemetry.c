#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <unistd.h>

#include "mqtt_gateway_app.h"

#ifndef TELEMETRY_CTRL_FILE_BUFFER_SIZE
#define TELEMETRY_CTRL_FILE_BUFFER_SIZE 4096
#endif

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

#define MAX_IFACES 16

typedef struct {
    uint64_t idle;
    uint64_t total;
} cpu_ticks_t;

typedef struct {
    char name[32];
    char ipv4[64];
    char mac[32];
    bool up;
} iface_info_t;

typedef struct {
    double numeric_value;
    bool numeric_ready;
    int decimal_places;
    bool decimal_places_ready;
} test_value_state_t;

static long long round_to_long_long(double value) {
    return (long long)(value >= 0.0 ? value + 0.5 : value - 0.5);
}

static bool parse_bool_text(const char *text, bool default_value) {
    if (!text || !*text) return default_value;
    if (strcasecmp(text, "1") == 0 || strcasecmp(text, "true") == 0 ||
        strcasecmp(text, "yes") == 0 || strcasecmp(text, "on") == 0) {
        return true;
    }
    if (strcasecmp(text, "0") == 0 || strcasecmp(text, "false") == 0 ||
        strcasecmp(text, "no") == 0 || strcasecmp(text, "off") == 0) {
        return false;
    }
    return default_value;
}

static double normalize_test_step(double step) {
    if (step == 0.0) return 1.0;
    return step < 0.0 ? -step : step;
}

static int detect_decimal_places(const char *text) {
    if (!text || !*text) return 0;

    const char *dot = strchr(text, '.');
    if (!dot) return 0;

    const char *end = text + strlen(text);
    const char *exp = strpbrk(dot + 1, "eE");
    if (exp) end = exp;
    while (end > dot + 1 && end[-1] == '0') {
        end--;
    }

    int places = (int)(end - dot - 1);
    if (places < 0) return 0;
    if (places > 9) return 9;
    return places;
}

static double decimal_scale(int places) {
    double scale = 1.0;
    while (places-- > 0) scale *= 10.0;
    return scale;
}

static double quantize_decimal(double value, int places) {
    if (places <= 0) return (double)round_to_long_long(value);
    double scale = decimal_scale(places);
    return (double)round_to_long_long(value * scale) / scale;
}

static int resolve_test_decimal_places(const app_cfg_t *cfg) {
    if (!cfg) return 0;
    int value_places = detect_decimal_places(cfg->test_value);

    char step_text[64];
    snprintf(step_text, sizeof(step_text), "%.9f", normalize_test_step(cfg->test_step));
    int step_places = detect_decimal_places(step_text);
    return value_places > step_places ? value_places : step_places;
}

static int add_test_value(cJSON *root, const app_cfg_t *cfg, test_value_state_t *state) {
    if (!root || !cfg || !state) return -1;

    switch (cfg->test_value_type) {
        case TEST_VALUE_INT:
        case TEST_VALUE_FLOAT: {
            if (!state->numeric_ready) {
                state->numeric_value = strtod(cfg->test_value, NULL);
                state->numeric_ready = true;
            }
            double current = state->numeric_value;
            if (cfg->test_value_type == TEST_VALUE_INT) {
                cJSON_AddNumberToObject(root, "data", (double)round_to_long_long(current));
            } else {
                if (!state->decimal_places_ready) {
                    state->decimal_places = resolve_test_decimal_places(cfg);
                    state->decimal_places_ready = true;
                }
                current = quantize_decimal(current, state->decimal_places);
                cJSON_AddNumberToObject(root, "data", current);
            }
            if (cfg->test_pattern != TEST_PATTERN_FIXED) {
                double step = normalize_test_step(cfg->test_step);
                double next = current + (cfg->test_pattern == TEST_PATTERN_DEC ? -step : step);
                if (cfg->test_value_type == TEST_VALUE_FLOAT) {
                    next = quantize_decimal(next, state->decimal_places);
                }
                state->numeric_value = next;
            }
            return 0;
        }
        case TEST_VALUE_BOOL:
            cJSON_AddBoolToObject(root, "data", parse_bool_text(cfg->test_value, false));
            return 0;
        case TEST_VALUE_STRING:
            cJSON_AddStringToObject(root, "data", cfg->test_value);
            return 0;
        case TEST_VALUE_JSON: {
            cJSON *item = cJSON_Parse(cfg->test_value);
            if (item) {
                cJSON_AddItemToObject(root, "data", item);
            } else {
                cJSON_AddStringToObject(root, "data", cfg->test_value);
            }
            return 0;
        }
        default:
            cJSON_AddStringToObject(root, "data", cfg->test_value);
            return 0;
    }
}

static bool read_cpu_ticks(cpu_ticks_t *ticks) {
    if (!ticks) return false;

    FILE *fp = fopen("/proc/stat", "r");
    if (!fp) return false;

    char line[256];
    bool ok = false;
    while (fgets(line, sizeof(line), fp) != NULL) {
        if (strncmp(line, "cpu ", 4) != 0) continue;

        unsigned long long user = 0, nice = 0, system = 0, idle = 0;
        unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;
        int n = sscanf(
            line,
            "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
            &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal
        );
        if (n >= 4) {
            ticks->idle = (uint64_t)idle + (uint64_t)iowait;
            ticks->total = (uint64_t)user + (uint64_t)nice + (uint64_t)system +
                           (uint64_t)idle + (uint64_t)iowait + (uint64_t)irq +
                           (uint64_t)softirq + (uint64_t)steal;
            ok = true;
        }
        break;
    }

    fclose(fp);
    return ok;
}

static bool calc_cpu_usage(const cpu_ticks_t *prev, const cpu_ticks_t *cur, double *usage_pct) {
    if (!prev || !cur || !usage_pct) return false;
    if (cur->total <= prev->total || cur->idle < prev->idle) return false;

    uint64_t total_delta = cur->total - prev->total;
    uint64_t idle_delta = cur->idle - prev->idle;
    if (total_delta == 0) return false;

    *usage_pct = (double)(total_delta - idle_delta) * 100.0 / (double)total_delta;
    return true;
}

static bool read_mem_usage(double *usage_pct) {
    if (!usage_pct) return false;

    FILE *fp = fopen("/proc/meminfo", "r");
    if (!fp) return false;

    unsigned long long mem_total_kb = 0;
    unsigned long long mem_available_kb = 0;
    unsigned long long mem_free_kb = 0;
    unsigned long long buffers_kb = 0;
    unsigned long long cached_kb = 0;
    char key[64];
    unsigned long long value = 0;
    char unit[32];

    while (fscanf(fp, "%63s %llu %31s", key, &value, unit) == 3) {
        if (strcmp(key, "MemTotal:") == 0) {
            mem_total_kb = value;
        } else if (strcmp(key, "MemAvailable:") == 0) {
            mem_available_kb = value;
        } else if (strcmp(key, "MemFree:") == 0) {
            mem_free_kb = value;
        } else if (strcmp(key, "Buffers:") == 0) {
            buffers_kb = value;
        } else if (strcmp(key, "Cached:") == 0) {
            cached_kb = value;
        }
    }

    fclose(fp);

    if (mem_total_kb == 0) return false;
    if (mem_available_kb == 0) {
        mem_available_kb = mem_free_kb + buffers_kb + cached_kb;
    }
    if (mem_available_kb > mem_total_kb) {
        mem_available_kb = mem_total_kb;
    }

    *usage_pct = (double)(mem_total_kb - mem_available_kb) * 100.0 / (double)mem_total_kb;
    return true;
}

static bool read_disk_usage(const char *path, double *usage_pct) {
    if (!path || !*path || !usage_pct) return false;

    struct statvfs st;
    if (statvfs(path, &st) != 0) return false;
    if (st.f_blocks == 0) return false;

    unsigned long long total = (unsigned long long)st.f_blocks;
    unsigned long long free_blocks = (unsigned long long)st.f_bavail;
    if (free_blocks > total) free_blocks = total;

    *usage_pct = (double)(total - free_blocks) * 100.0 / (double)total;
    return true;
}

static bool read_number_file(const char *path, long long *out) {
    if (!path || !*path || !out) return false;

    FILE *fp = fopen(path, "r");
    if (!fp) return false;

    long long value = 0;
    int rc = fscanf(fp, "%lld", &value);
    fclose(fp);
    if (rc != 1) return false;

    *out = value;
    return true;
}

static bool json_get_bool(cJSON *item, bool *out) {
    if (!item || !out) return false;
    if (cJSON_IsBool(item)) {
        *out = cJSON_IsTrue(item) ? true : false;
        return true;
    }
    if (cJSON_IsNumber(item)) {
        *out = item->valueint ? true : false;
        return true;
    }
    return false;
}

static bool read_temp_from_path(const char *path, double *temp_c) {
    long long raw = 0;
    if (!read_number_file(path, &raw)) return false;

    double value = (double)raw;
    if (value > 1000.0 || value < -1000.0) {
        value /= 1000.0;
    }

    *temp_c = value;
    return true;
}

static bool discover_temp_path(char *path, size_t cap) {
    if (!path || cap == 0) return false;

    DIR *dir = opendir("/sys/class/thermal");
    if (!dir) return false;

    struct dirent *ent;
    bool found = false;
    while ((ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, "thermal_zone", 12) != 0) continue;

        char candidate[PATH_MAX];
        snprintf(candidate, sizeof(candidate), "/sys/class/thermal/%s/temp", ent->d_name);

        long long raw = 0;
        if (read_number_file(candidate, &raw)) {
            snprintf(path, cap, "%s", candidate);
            found = true;
            break;
        }
    }

    closedir(dir);
    return found;
}

static bool read_first_line(const char *path, char *out, size_t cap) {
    if (!path || !*path || !out || cap == 0) return false;
    FILE *fp = fopen(path, "r");
    if (!fp) return false;
    if (!fgets(out, (int)cap, fp)) {
        fclose(fp);
        return false;
    }
    fclose(fp);
    size_t len = strlen(out);
    while (len > 0 && (out[len - 1] == '\n' || out[len - 1] == '\r')) {
        out[len - 1] = '\0';
        len--;
    }
    return out[0] != '\0';
}

static int find_iface_index(iface_info_t *items, int count, const char *name) {
    if (!items || !name) return -1;
    for (int i = 0; i < count; i++) {
        if (strcmp(items[i].name, name) == 0) return i;
    }
    return -1;
}

static int collect_interfaces(iface_info_t *items, int max_items, bool *has_online) {
    if (!items || max_items <= 0) return 0;
    struct ifaddrs *ifaddr = NULL;
    if (getifaddrs(&ifaddr) != 0) return 0;

    int count = 0;
    for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_name || !ifa->ifa_name[0]) continue;
        int idx = find_iface_index(items, count, ifa->ifa_name);
        if (idx < 0) {
            if (count >= max_items) continue;
            idx = count++;
            memset(&items[idx], 0, sizeof(items[idx]));
            snprintf(items[idx].name, sizeof(items[idx].name), "%s", ifa->ifa_name);
        }
        if (ifa->ifa_flags & IFF_UP) {
            items[idx].up = true;
            if (has_online && strcmp(items[idx].name, "lo") != 0) {
                *has_online = true;
            }
        }
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET && !items[idx].ipv4[0]) {
            struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;
            (void)inet_ntop(AF_INET, &addr->sin_addr, items[idx].ipv4, sizeof(items[idx].ipv4));
        }
    }
    freeifaddrs(ifaddr);

    for (int i = 0; i < count; i++) {
        char path[PATH_MAX];
        snprintf(path, sizeof(path), "/sys/class/net/%s/address", items[i].name);
        (void)read_first_line(path, items[i].mac, sizeof(items[i].mac));
    }
    return count;
}

static int publish_test_telemetry(app_t *app, test_value_state_t *state) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return -1;

    char ts[32];
    int64_t epoch_ms = 0;
    format_iso8601(ts, sizeof(ts), &epoch_ms);
    (void)epoch_ms;

    const char *cpuinfo = app->cfg.cpu_serial[0] ? app->cfg.cpu_serial : app->cfg.device_id;
    cJSON_AddStringToObject(root, "ts", ts);
    cJSON_AddStringToObject(root, "cpuinfo", cpuinfo);
    if (add_test_value(root, &app->cfg, state) != 0) {
        cJSON_Delete(root);
        return -1;
    }

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return -1;

    int rc = mqtt_publish_json(app, app->topic_test_up, 1, 0, payload);
    cJSON_free(payload);
    return rc;
}

static int publish_network_telemetry(app_t *app) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return -1;

    char ts[32];
    int64_t epoch_ms = 0;
    format_iso8601(ts, sizeof(ts), &epoch_ms);

    iface_info_t items[MAX_IFACES];
    memset(items, 0, sizeof(items));
    bool has_online = false;
    int iface_count = collect_interfaces(items, MAX_IFACES, &has_online);

    char brief[512];
    brief[0] = '\0';
    for (int i = 0; i < iface_count; i++) {
        char segment[160];
        snprintf(
            segment,
            sizeof(segment),
            "%s%s%s%s%s%s%s",
            items[i].name,
            items[i].up ? " up" : " down",
            items[i].ipv4[0] ? " " : "",
            items[i].ipv4,
            items[i].mac[0] ? " " : "",
            items[i].mac[0] ? items[i].mac : "",
            (i + 1 < iface_count) ? " | " : ""
        );
        strncat(brief, segment, sizeof(brief) - strlen(brief) - 1);
    }

    pthread_mutex_lock(&app->mqtt_lock);
    bool mqtt_connected = app->mqtt_connected;
    pthread_mutex_unlock(&app->mqtt_lock);

    cJSON_AddStringToObject(root, "ts", ts);
    cJSON_AddNumberToObject(root, "epoch_ms", (double)epoch_ms);
    cJSON_AddStringToObject(root, "source", "network_metrics");
    cJSON_AddStringToObject(root, "data_class", "network");
    cJSON_AddStringToObject(root, "thread_kind", "network_runtime");
    cJSON_AddStringToObject(root, "cpuinfo", app->cfg.device_id);
    cJSON_AddStringToObject(root, "device_id", app->cfg.device_id);
    cJSON_AddStringToObject(root, "client_id", app->cfg.client_id);
    cJSON_AddStringToObject(root, "net_status", has_online ? "online" : "offline");
    cJSON_AddStringToObject(root, "mqtt_status", mqtt_connected ? "connected" : "disconnected");
    cJSON_AddStringToObject(root, "cloud_status", mqtt_connected ? "connected" : "disconnected");
    if (brief[0]) {
        cJSON_AddStringToObject(root, "ifconfig_brief", brief);
    }
    cJSON *ifaces = cJSON_AddArrayToObject(root, "interfaces");
    if (ifaces) {
        for (int i = 0; i < iface_count; i++) {
            cJSON *iface = cJSON_CreateObject();
            if (!iface) continue;
            cJSON_AddStringToObject(iface, "name", items[i].name);
            cJSON_AddStringToObject(iface, "state", items[i].up ? "up" : "down");
            if (items[i].ipv4[0]) cJSON_AddStringToObject(iface, "ipv4", items[i].ipv4);
            if (items[i].mac[0]) cJSON_AddStringToObject(iface, "mac", items[i].mac);
            cJSON_AddItemToArray(ifaces, iface);
        }
    }

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return -1;

    int rc = mqtt_publish_json(app, app->topic_data_up, 1, 0, payload);
    cJSON_free(payload);
    return rc;
}

static int publish_system_telemetry(
    app_t *app,
    double cpu_usage,
    bool has_cpu_usage,
    double mem_usage,
    bool has_mem_usage,
    double disk_usage,
    bool has_disk_usage,
    double board_temp_c,
    bool has_board_temp_c
) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return -1;

    char ts[32];
    int64_t epoch_ms = 0;
    format_iso8601(ts, sizeof(ts), &epoch_ms);

    cJSON_AddStringToObject(root, "ts", ts);
    cJSON_AddNumberToObject(root, "epoch_ms", (double)epoch_ms);
    cJSON_AddStringToObject(root, "source", "system_metrics");
    cJSON_AddStringToObject(root, "data_class", "system");
    cJSON_AddStringToObject(root, "thread_kind", "system_runtime");
    cJSON_AddStringToObject(root, "cpuinfo", app->cfg.device_id);
    cJSON_AddStringToObject(root, "device_id", app->cfg.device_id);
    cJSON_AddStringToObject(root, "client_id", app->cfg.client_id);
    if (has_cpu_usage) cJSON_AddNumberToObject(root, "cpu_usage", cpu_usage);
    if (has_mem_usage) cJSON_AddNumberToObject(root, "mem_usage", mem_usage);
    if (has_disk_usage) cJSON_AddNumberToObject(root, "disk_usage", disk_usage);
    if (has_board_temp_c) cJSON_AddNumberToObject(root, "board_temp_c", board_temp_c);

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return -1;

    int rc = mqtt_publish_json(app, app->topic_data_up, 1, 0, payload);
    cJSON_free(payload);
    return rc;
}

static void reload_ctrl_overrides(
    const char *ctrl_path,
    int *interval_ms,
    bool *telemetry_enable,
    time_t *last_mtime,
    off_t *last_size
) {
    if (!ctrl_path || !*ctrl_path || !interval_ms || !telemetry_enable || !last_mtime || !last_size) {
        return;
    }

    struct stat st;
    if (stat(ctrl_path, &st) != 0) return;
    if (st.st_mtime == *last_mtime && st.st_size == *last_size) return;

    FILE *fp = fopen(ctrl_path, "rb");
    if (!fp) return;

    if (fseeko(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return;
    }
    long size = ftell(fp);
    if (size < 0 || fseeko(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return;
    }

    size_t to_read = (size_t)size;
    if (to_read >= TELEMETRY_CTRL_FILE_BUFFER_SIZE) {
        to_read = TELEMETRY_CTRL_FILE_BUFFER_SIZE - 1u;
    }

    char buf[TELEMETRY_CTRL_FILE_BUFFER_SIZE];
    size_t rd = fread(buf, 1, to_read, fp);
    fclose(fp);
    buf[rd] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);
        return;
    }

    cJSON *interval = cJSON_GetObjectItemCaseSensitive(root, "interval_ms");
    if (!cJSON_IsNumber(interval)) {
        interval = cJSON_GetObjectItemCaseSensitive(root, "telemetry_interval_ms");
    }
    if (cJSON_IsNumber(interval)) {
        int candidate = interval->valueint;
        if (candidate >= 500 && candidate <= 3600000) {
            *interval_ms = candidate;
        }
    }

    cJSON *enable = cJSON_GetObjectItemCaseSensitive(root, "telemetry_enable");
    bool enabled = false;
    if (json_get_bool(enable, &enabled)) {
        *telemetry_enable = enabled;
    }

    cJSON_Delete(root);
    *last_mtime = st.st_mtime;
    *last_size = st.st_size;
}

void *telemetry_publisher_main(void *arg) {
    app_t *app = (app_t *)arg;
    app_cfg_t cfg = app->cfg;

    int interval_ms = cfg.telemetry_interval_ms;
    if (interval_ms < 500) interval_ms = 500;
    bool telemetry_enable = cfg.telemetry_enable;
    time_t ctrl_mtime = 0;
    off_t ctrl_size = 0;

    char temp_path[PATH_MAX];
    temp_path[0] = '\0';
    if (cfg.telemetry_temp_path[0]) {
        snprintf(temp_path, sizeof(temp_path), "%s", cfg.telemetry_temp_path);
    } else {
        (void)discover_temp_path(temp_path, sizeof(temp_path));
    }

    cpu_ticks_t prev_ticks;
    bool has_prev_ticks = read_cpu_ticks(&prev_ticks);
    uint64_t last_temp_probe_ms = monotonic_ms();
    uint64_t last_publish_log_ms = 0;

    while (g_running) {
        reload_ctrl_overrides(
            cfg.ctrl_path,
            &interval_ms,
            &telemetry_enable,
            &ctrl_mtime,
            &ctrl_size
        );

        if (!telemetry_enable) {
            sleep_ms(500);
            continue;
        }

        sleep_ms(interval_ms);
        if (!g_running) break;

        publish_status(app, "online", interval_ms);

        if (!telemetry_enable) {
            continue;
        }

        cpu_ticks_t cur_ticks;
        double cpu_usage = 0.0;
        double mem_usage = 0.0;
        double disk_usage = 0.0;
        double board_temp_c = 0.0;

        bool has_cpu_usage = false;
        bool has_mem_usage = read_mem_usage(&mem_usage);
        bool has_disk_usage = read_disk_usage(cfg.telemetry_disk_path, &disk_usage);
        bool has_board_temp_c = false;

        if (read_cpu_ticks(&cur_ticks)) {
            if (has_prev_ticks) {
                has_cpu_usage = calc_cpu_usage(&prev_ticks, &cur_ticks, &cpu_usage);
            }
            prev_ticks = cur_ticks;
            has_prev_ticks = true;
        }

        if (temp_path[0]) {
            has_board_temp_c = read_temp_from_path(temp_path, &board_temp_c);
        }
        if (!has_board_temp_c && monotonic_ms() - last_temp_probe_ms >= 30000ull) {
            temp_path[0] = '\0';
            (void)discover_temp_path(temp_path, sizeof(temp_path));
            last_temp_probe_ms = monotonic_ms();
            if (temp_path[0]) {
                has_board_temp_c = read_temp_from_path(temp_path, &board_temp_c);
            }
        }

        if (!has_cpu_usage && !has_mem_usage && !has_disk_usage && !has_board_temp_c) {
            uint64_t now = monotonic_ms();
            if (now - last_publish_log_ms >= 30000ull) {
                LOGW("%s", "system telemetry unavailable");
                last_publish_log_ms = now;
            }
            continue;
        }

        int rc = publish_system_telemetry(
            app,
            cpu_usage, has_cpu_usage,
            mem_usage, has_mem_usage,
            disk_usage, has_disk_usage,
            board_temp_c, has_board_temp_c
        );
        if (rc != MQTTCLIENT_SUCCESS) {
            uint64_t now = monotonic_ms();
            if (now - last_publish_log_ms >= 10000ull) {
                LOGW("system telemetry publish failed: rc=%d", rc);
                last_publish_log_ms = now;
            }
        }

        rc = publish_network_telemetry(app);
        if (rc != MQTTCLIENT_SUCCESS) {
            uint64_t now = monotonic_ms();
            if (now - last_publish_log_ms >= 10000ull) {
                LOGW("network telemetry publish failed: rc=%d", rc);
                last_publish_log_ms = now;
            }
        }
    }

    return NULL;
}

void *test_publisher_main(void *arg) {
    app_t *app = (app_t *)arg;
    app_cfg_t cfg = app->cfg;

    if (!cfg.test_mode_enable) {
        while (g_running) sleep_ms(500);
        return NULL;
    }

    int interval_ms = cfg.test_interval_ms;
    if (interval_ms < 200) interval_ms = 200;

    uint64_t last_publish_log_ms = 0;
    test_value_state_t test_state;
    memset(&test_state, 0, sizeof(test_state));

    while (g_running) {
        sleep_ms(interval_ms);
        if (!g_running) break;

        int rc = publish_test_telemetry(app, &test_state);
        if (rc != MQTTCLIENT_SUCCESS) {
            uint64_t now = monotonic_ms();
            if (now - last_publish_log_ms >= 10000ull) {
                LOGW("test telemetry publish failed: rc=%d", rc);
                last_publish_log_ms = now;
            }
        }
    }

    return NULL;
}
