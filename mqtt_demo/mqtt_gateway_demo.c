#define _DEFAULT_SOURCE
#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#include "../demo_config.h"
#include "mqtt_gateway_app.h"

volatile sig_atomic_t g_running = 1;

static void on_signal(int sig) {
    (void)sig;
    g_running = 0;
}

void log_write(const char *level, const char *fmt, ...) {
    char msg[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);

    fprintf(stderr, "[%s] %s %s\n", level, ts, msg);
}

void sleep_ms(int ms) {
    if (ms <= 0) return;
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000);
    ts.tv_nsec = (long)((ms % 1000) * 1000000L);
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
    }
}

uint64_t monotonic_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

void format_iso8601(char *buf, size_t cap, int64_t *epoch_ms) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    time_t sec = ts.tv_sec;
    struct tm tm_local;
    struct tm tm_utc;
    gmtime_r(&sec, &tm_utc);
    localtime_r(&sec, &tm_local);

    size_t n = strftime(buf, cap, "%Y-%m-%dT%H:%M:%S", &tm_local);
    if (n > 0 && n < cap) {
        time_t local_epoch = mktime(&tm_local);
        time_t utc_epoch = mktime(&tm_utc);
        long offset = (long)difftime(local_epoch, utc_epoch);
        int sign = (offset < 0) ? -1 : 1;
        long off = (offset < 0) ? -offset : offset;
        int oh = (int)(off / 3600);
        int om = (int)((off % 3600) / 60);
        snprintf(buf + n, cap - n, ".%03ld%c%02d:%02d",
                 ts.tv_nsec / 1000000,
                 (sign < 0) ? '-' : '+',
                 oh, om);
    }
    if (epoch_ms) *epoch_ms = (int64_t)ts.tv_sec * 1000 + (ts.tv_nsec / 1000000);
}

static char *lstrip(char *s) {
    while (s && *s && isspace((unsigned char)*s)) s++;
    return s;
}

static void rstrip(char *s) {
    if (!s) return;
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) {
        s[n - 1] = '\0';
        n--;
    }
}

static bool read_cpuinfo_serial(char *out, size_t cap) {
    if (!out || cap == 0) return false;
    out[0] = '\0';
    FILE *fp = fopen("/proc/cpuinfo", "r");
    if (!fp) return false;
    char line[256];
    while (fgets(line, sizeof(line), fp) != NULL) {
        if (strncasecmp(line, "Serial", 6) != 0) continue;
        char *colon = strchr(line, ':');
        if (!colon) continue;
        char *val = lstrip(colon + 1);
        rstrip(val);
        if (val && *val) {
            snprintf(out, cap, "%s", val);
            fclose(fp);
            return true;
        }
    }
    fclose(fp);
    return false;
}

static tail_format_t parse_tail_format(const char *s);
static test_value_type_t parse_test_value_type(const char *s);
static test_pattern_t parse_test_pattern(const char *s);

static bool parse_cfg_bool01(const char *text, bool *out) {
    long value = 0;
    if (demo_config_parse_bool(text, out)) return true;
    if (demo_config_parse_long(text, 0, 1, &value)) {
        *out = (value != 0);
        return true;
    }
    return false;
}

static bool parse_cfg_tail_format(const char *text, tail_format_t *out) {
    tail_format_t fmt = parse_tail_format(text);
    if (fmt == TAIL_FORMAT_AUTO && strcasecmp(text, "auto") != 0) return false;
    *out = fmt;
    return true;
}

static bool parse_cfg_test_type(const char *text, test_value_type_t *out) {
    test_value_type_t type = parse_test_value_type(text);
    if (type == TEST_VALUE_INT &&
        strcasecmp(text, "int") != 0 &&
        strcasecmp(text, "integer") != 0) {
        return false;
    }
    *out = type;
    return true;
}

static bool parse_cfg_test_pattern(const char *text, test_pattern_t *out) {
    test_pattern_t pattern = parse_test_pattern(text);
    if (pattern == TEST_PATTERN_FIXED && strcasecmp(text, "fixed") != 0) {
        return false;
    }
    *out = pattern;
    return true;
}

static bool is_common_section(const char *section) {
    return !section || !section[0] ||
           demo_config_name_eq(section, "mqtt") ||
           demo_config_name_eq(section, "common");
}

static int mqtt_apply_config_entry(const char *section,
                                   const char *key,
                                   const char *value,
                                   void *user,
                                   char *errbuf,
                                   size_t errcap) {
    app_cfg_t *cfg = (app_cfg_t *)user;
    long lv = 0;
    double dv = 0.0;
    bool bv = false;

    if (!cfg || !key || !value) return -1;

    if (is_common_section(section)) {
        if (demo_config_name_eq(key, "broker")) {
            snprintf(cfg->broker, sizeof(cfg->broker), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "client_id")) {
            snprintf(cfg->client_id, sizeof(cfg->client_id), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "device_id")) {
            snprintf(cfg->device_id, sizeof(cfg->device_id), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "username")) {
            snprintf(cfg->username, sizeof(cfg->username), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "password")) {
            snprintf(cfg->password, sizeof(cfg->password), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "keepalive") || demo_config_name_eq(key, "keepalive_s")) {
            if (!demo_config_parse_long(value, 1, 3600, &lv)) goto bad_value;
            cfg->keepalive_s = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "clean_session")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->clean_session = bv ? 1 : 0;
            return 0;
        }
        if (demo_config_name_eq(key, "ctrl_file") || demo_config_name_eq(key, "ctrl_path")) {
            snprintf(cfg->ctrl_path, sizeof(cfg->ctrl_path), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "ssl") || demo_config_name_eq(key, "ssl_enable")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->ssl_enable = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "ssl_insecure")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->ssl_insecure = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "cafile")) {
            snprintf(cfg->cafile, sizeof(cfg->cafile), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "cert") || demo_config_name_eq(key, "certfile")) {
            snprintf(cfg->certfile, sizeof(cfg->certfile), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "key") || demo_config_name_eq(key, "keyfile")) {
            snprintf(cfg->keyfile, sizeof(cfg->keyfile), "%s", value);
            return 0;
        }
    }

    if (demo_config_name_eq(section, "tail") || demo_config_name_eq(section, "tailer")) {
        if (demo_config_name_eq(key, "enable") || demo_config_name_eq(key, "tail_enable")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->tail_enable = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "data_dir")) {
            snprintf(cfg->data_dir, sizeof(cfg->data_dir), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "data_ext")) {
            snprintf(cfg->data_ext, sizeof(cfg->data_ext), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "format") || demo_config_name_eq(key, "tail_format")) {
            tail_format_t fmt;
            if (!parse_cfg_tail_format(value, &fmt)) goto bad_value;
            cfg->tail_format = fmt;
            return 0;
        }
        if (demo_config_name_eq(key, "poll_ms") || demo_config_name_eq(key, "tail_poll_ms")) {
            if (!demo_config_parse_long(value, 1, 3600000, &lv)) goto bad_value;
            cfg->tail_poll_ms = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "idle_ms") || demo_config_name_eq(key, "tail_idle_ms")) {
            if (!demo_config_parse_long(value, 1, 3600000, &lv)) goto bad_value;
            cfg->tail_idle_ms = (int)lv;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "telemetry")) {
        if (demo_config_name_eq(key, "enable") || demo_config_name_eq(key, "telemetry_enable")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->telemetry_enable = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "interval_ms") || demo_config_name_eq(key, "telemetry_interval_ms")) {
            if (!demo_config_parse_long(value, 1, 3600000, &lv)) goto bad_value;
            cfg->telemetry_interval_ms = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "disk_path") || demo_config_name_eq(key, "telemetry_disk_path")) {
            snprintf(cfg->telemetry_disk_path, sizeof(cfg->telemetry_disk_path), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "temp_path") || demo_config_name_eq(key, "telemetry_temp_path")) {
            snprintf(cfg->telemetry_temp_path, sizeof(cfg->telemetry_temp_path), "%s", value);
            return 0;
        }
    }

    if (demo_config_name_eq(section, "test")) {
        if (demo_config_name_eq(key, "enable") || demo_config_name_eq(key, "test_mode")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->test_mode_enable = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "interval_ms") || demo_config_name_eq(key, "test_interval_ms")) {
            if (!demo_config_parse_long(value, 1, 3600000, &lv)) goto bad_value;
            cfg->test_interval_ms = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "type") || demo_config_name_eq(key, "test_type")) {
            test_value_type_t type;
            if (!parse_cfg_test_type(value, &type)) goto bad_value;
            cfg->test_value_type = type;
            return 0;
        }
        if (demo_config_name_eq(key, "pattern") || demo_config_name_eq(key, "test_pattern")) {
            test_pattern_t pattern;
            if (!parse_cfg_test_pattern(value, &pattern)) goto bad_value;
            cfg->test_pattern = pattern;
            return 0;
        }
        if (demo_config_name_eq(key, "value") || demo_config_name_eq(key, "test_value")) {
            snprintf(cfg->test_value, sizeof(cfg->test_value), "%s", value);
            return 0;
        }
        if (demo_config_name_eq(key, "step") || demo_config_name_eq(key, "test_step")) {
            if (!demo_config_parse_double(value, &dv)) goto bad_value;
            cfg->test_step = dv;
            return 0;
        }
    }

    if (errbuf && errcap > 0) {
        snprintf(errbuf, errcap, "unknown config entry [%s] %s", section && section[0] ? section : "default", key);
    }
    return -1;

bad_value:
    if (errbuf && errcap > 0) {
        snprintf(errbuf, errcap, "bad value for [%s] %s: %s",
                 section && section[0] ? section : "default", key, value);
    }
    return -1;
}

static const char *mqtt_find_config_path(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-c") || !strcmp(argv[i], "--config")) {
            if (i + 1 >= argc) return NULL;
            return argv[i + 1];
        }
        if (!strncmp(argv[i], "--config=", 9)) {
            return argv[i] + 9;
        }
    }
    return NULL;
}

static bool mqtt_is_config_arg(const char *arg) {
    return !strcmp(arg, "-c") || !strcmp(arg, "--config") || !strncmp(arg, "--config=", 9);
}

void cfg_defaults(app_cfg_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    snprintf(cfg->broker, sizeof(cfg->broker), "%s", "tcp://127.0.0.1:1883");
    snprintf(cfg->client_id, sizeof(cfg->client_id), "%s", "gw001");
    snprintf(cfg->device_id, sizeof(cfg->device_id), "%s", "gw001");
    snprintf(cfg->ctrl_path, sizeof(cfg->ctrl_path), "%s", "/tmp/modbus_framer_ctrl.json");
    cfg->keepalive_s = 30;
    cfg->clean_session = 1;
    cfg->ssl_enable = false;
    cfg->ssl_insecure = false;

    cfg->tail_enable = true;
    cfg->tail_poll_ms = 200;
    cfg->tail_idle_ms = 3000;
    cfg->tail_format = TAIL_FORMAT_AUTO;
    snprintf(cfg->data_dir, sizeof(cfg->data_dir), "%s", "/userdata/modbus_data");
    cfg->data_ext[0] = '\0';

    cfg->telemetry_enable = true;
    cfg->telemetry_interval_ms = 5000;
    snprintf(cfg->telemetry_disk_path, sizeof(cfg->telemetry_disk_path), "%s", "/");
    cfg->telemetry_temp_path[0] = '\0';

    cfg->test_mode_enable = false;
    cfg->test_interval_ms = 1000;
    cfg->test_value_type = TEST_VALUE_INT;
    cfg->test_pattern = TEST_PATTERN_FIXED;
    snprintf(cfg->test_value, sizeof(cfg->test_value), "%s", "0");
    cfg->test_step = 1.0;
    cfg->cpu_serial[0] = '\0';
}

void build_topics(app_t *app) {
    snprintf(app->topic_cmd_down, sizeof(app->topic_cmd_down),
             "gateway/%s/command/down", app->cfg.device_id);
    snprintf(app->topic_cmd_resp, sizeof(app->topic_cmd_resp),
             "gateway/%s/command/resp", app->cfg.device_id);
    snprintf(app->topic_cfg_down, sizeof(app->topic_cfg_down),
             "gateway/%s/config/down", app->cfg.device_id);
    snprintf(app->topic_cfg_resp, sizeof(app->topic_cfg_resp),
             "gateway/%s/config/resp", app->cfg.device_id);
    snprintf(app->topic_status, sizeof(app->topic_status),
             "gateway/%s/status", app->cfg.device_id);
    snprintf(app->topic_data_up, sizeof(app->topic_data_up),
             "gateway/%s/data/up", app->cfg.device_id);
    snprintf(app->topic_test_up, sizeof(app->topic_test_up),
             "gateway/%s/test/up", app->cfg.device_id);
    snprintf(app->topic_test_down, sizeof(app->topic_test_down),
             "gateway/%s/test/down", app->cfg.device_id);
}

static void usage(const char *argv0) {
    fprintf(stderr,
            "Usage: %s [options]\n"
            "  -c, --config PATH  load startup config file\n"
            "Common:\n"
            "  --broker tcp://host:1883\n"
            "  --device-id ID\n"
            "  --client-id ID\n"
            "  --username USER\n"
            "  --password PASS\n"
            "  --keepalive N\n"
            "  --clean-session 0|1\n"
            "  --ctrl-file PATH   (default /tmp/modbus_framer_ctrl.json)\n"
            "\nTLS:\n"
            "  --ssl              (or use ssl:// in broker)\n"
            "  --cafile PATH\n"
            "  --cert PATH\n"
            "  --key PATH\n"
            "  --ssl-insecure\n"
            "\nFile tailer:\n"
            "  --tail-enable 0|1\n"
            "  --data-dir PATH    (default /userdata/modbus_data)\n"
            "  --data-ext EXT     (override, e.g. .jsonl)\n"
            "  --tail-format auto|text|json|xml\n"
            "  --tail-poll-ms N\n"
            "  --tail-idle-ms N\n"
            "\nSystem telemetry:\n"
            "  --telemetry-enable 0|1\n"
            "  --telemetry-interval-ms N\n"
            "  --telemetry-disk-path PATH  (default /)\n"
            "  --telemetry-temp-path PATH  (default auto-discover)\n"
            "\nTest mode:\n"
            "  --test-mode 0|1\n"
            "  --test-type int|float|bool|string|json\n"
            "  --test-pattern fixed|inc|dec\n"
            "  --test-value VALUE\n"
            "  --test-step N\n"
            "  --test-interval-ms N\n"
            "\nNotes:\n"
            "  - Direct Modbus read/write and Modbus startup are removed from this demo.\n"
            "  - command/down supports led/set_led, buzzer/set_buzzer, write_coil/write, shutdown, suspend.\n"
            "  - test mode publishes a minimal payload with cpuinfo, ts and data to gateway/<device>/test/up.\n"
            "  - test mode runs independently and does not disable normal telemetry or file tailing.\n"
            "  - test/down is always subscribed; received test payloads are printed directly.\n"
            "  - config/down saves Modbus control parameters to --ctrl-file and publishes config/resp.\n",
            argv0);
}

static tail_format_t parse_tail_format(const char *s) {
    if (!s || !*s) return TAIL_FORMAT_AUTO;
    if (strcasecmp(s, "auto") == 0) return TAIL_FORMAT_AUTO;
    if (strcasecmp(s, "text") == 0) return TAIL_FORMAT_TEXT;
    if (strcasecmp(s, "json") == 0) return TAIL_FORMAT_JSON;
    if (strcasecmp(s, "xml") == 0) return TAIL_FORMAT_XML;
    return TAIL_FORMAT_AUTO;
}

static test_value_type_t parse_test_value_type(const char *s) {
    if (!s || !*s) return TEST_VALUE_INT;
    if (strcasecmp(s, "int") == 0 || strcasecmp(s, "integer") == 0) return TEST_VALUE_INT;
    if (strcasecmp(s, "float") == 0 || strcasecmp(s, "double") == 0) return TEST_VALUE_FLOAT;
    if (strcasecmp(s, "bool") == 0 || strcasecmp(s, "boolean") == 0) return TEST_VALUE_BOOL;
    if (strcasecmp(s, "string") == 0 || strcasecmp(s, "text") == 0) return TEST_VALUE_STRING;
    if (strcasecmp(s, "json") == 0) return TEST_VALUE_JSON;
    return TEST_VALUE_INT;
}

static test_pattern_t parse_test_pattern(const char *s) {
    if (!s || !*s) return TEST_PATTERN_FIXED;
    if (strcasecmp(s, "fixed") == 0) return TEST_PATTERN_FIXED;
    if (strcasecmp(s, "inc") == 0 || strcasecmp(s, "increase") == 0) return TEST_PATTERN_INC;
    if (strcasecmp(s, "dec") == 0 || strcasecmp(s, "decrease") == 0) return TEST_PATTERN_DEC;
    return TEST_PATTERN_FIXED;
}

void parse_args_full(app_cfg_t *cfg, int argc, char **argv) {
    const char *config_path = mqtt_find_config_path(argc, argv);
    if ((config_path && !*config_path) || ((argc > 1) &&
        ((strcmp(argv[argc - 1], "-c") == 0) || (strcmp(argv[argc - 1], "--config") == 0)))) {
        usage(argv[0]);
        exit(2);
    }
    if (config_path) {
        char errbuf[256];
        if (demo_config_load(config_path, mqtt_apply_config_entry, cfg, errbuf, sizeof(errbuf)) != 0) {
            LOGE("load config failed: %s", errbuf[0] ? errbuf : config_path);
            exit(2);
        }
    }

    for (int i = 1; i < argc; i++) {
        if (mqtt_is_config_arg(argv[i])) {
            if (!strcmp(argv[i], "-c") || !strcmp(argv[i], "--config")) i++;
            continue;
        }
        if (!strcmp(argv[i], "--broker") && i + 1 < argc) {
            snprintf(cfg->broker, sizeof(cfg->broker), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--ssl")) { cfg->ssl_enable = true; continue; }
        if (!strcmp(argv[i], "--cafile") && i + 1 < argc) {
            snprintf(cfg->cafile, sizeof(cfg->cafile), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--cert") && i + 1 < argc) {
            snprintf(cfg->certfile, sizeof(cfg->certfile), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--key") && i + 1 < argc) {
            snprintf(cfg->keyfile, sizeof(cfg->keyfile), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--ssl-insecure")) { cfg->ssl_insecure = true; continue; }
        if (!strcmp(argv[i], "--client-id") && i + 1 < argc) {
            snprintf(cfg->client_id, sizeof(cfg->client_id), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--device-id") && i + 1 < argc) {
            snprintf(cfg->device_id, sizeof(cfg->device_id), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--username") && i + 1 < argc) {
            snprintf(cfg->username, sizeof(cfg->username), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--password") && i + 1 < argc) {
            snprintf(cfg->password, sizeof(cfg->password), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--keepalive") && i + 1 < argc) {
            cfg->keepalive_s = atoi(argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--clean-session") && i + 1 < argc) {
            cfg->clean_session = atoi(argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--ctrl-file") && i + 1 < argc) {
            snprintf(cfg->ctrl_path, sizeof(cfg->ctrl_path), "%s", argv[++i]);
            continue;
        }

        if (!strcmp(argv[i], "--tail-enable") && i + 1 < argc) {
            cfg->tail_enable = atoi(argv[++i]) ? true : false;
            continue;
        }
        if (!strcmp(argv[i], "--data-dir") && i + 1 < argc) {
            snprintf(cfg->data_dir, sizeof(cfg->data_dir), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--data-ext") && i + 1 < argc) {
            snprintf(cfg->data_ext, sizeof(cfg->data_ext), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--tail-format") && i + 1 < argc) {
            const char *fmt_s = argv[++i];
            tail_format_t fmt = parse_tail_format(fmt_s);
            if (fmt == TAIL_FORMAT_AUTO && strcasecmp(fmt_s, "auto") != 0) {
                LOGW("unknown tail format: %s (use auto|text|json|xml)", fmt_s);
            }
            cfg->tail_format = fmt;
            continue;
        }
        if (!strcmp(argv[i], "--tail-poll-ms") && i + 1 < argc) {
            cfg->tail_poll_ms = atoi(argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--tail-idle-ms") && i + 1 < argc) {
            cfg->tail_idle_ms = atoi(argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--telemetry-enable") && i + 1 < argc) {
            cfg->telemetry_enable = atoi(argv[++i]) ? true : false;
            continue;
        }
        if (!strcmp(argv[i], "--telemetry-interval-ms") && i + 1 < argc) {
            cfg->telemetry_interval_ms = atoi(argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--telemetry-disk-path") && i + 1 < argc) {
            snprintf(cfg->telemetry_disk_path, sizeof(cfg->telemetry_disk_path), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--telemetry-temp-path") && i + 1 < argc) {
            snprintf(cfg->telemetry_temp_path, sizeof(cfg->telemetry_temp_path), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--test-mode") && i + 1 < argc) {
            cfg->test_mode_enable = atoi(argv[++i]) ? true : false;
            continue;
        }
        if (!strcmp(argv[i], "--test-type") && i + 1 < argc) {
            cfg->test_value_type = parse_test_value_type(argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--test-pattern") && i + 1 < argc) {
            cfg->test_pattern = parse_test_pattern(argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--test-value") && i + 1 < argc) {
            snprintf(cfg->test_value, sizeof(cfg->test_value), "%s", argv[++i]);
            continue;
        }
        if (!strcmp(argv[i], "--test-step") && i + 1 < argc) {
            cfg->test_step = strtod(argv[++i], NULL);
            continue;
        }
        if (!strcmp(argv[i], "--test-interval-ms") && i + 1 < argc) {
            cfg->test_interval_ms = atoi(argv[++i]);
            continue;
        }

        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            usage(argv[0]);
            exit(0);
        }

        LOGW("ignore unknown arg: %s", argv[i]);
    }
}

int main(int argc, char **argv) {
    app_t app;
    memset(&app, 0, sizeof(app));

    cfg_defaults(&app.cfg);
    parse_args_full(&app.cfg, argc, argv);

    if (read_cpuinfo_serial(app.cfg.cpu_serial, sizeof(app.cfg.cpu_serial))) {
        if (!app.cfg.device_id[0] || strcmp(app.cfg.device_id, "gw001") == 0) {
            snprintf(app.cfg.device_id, sizeof(app.cfg.device_id), "%s", app.cfg.cpu_serial);
            if (!app.cfg.client_id[0] || strcmp(app.cfg.client_id, "gw001") == 0) {
                snprintf(app.cfg.client_id, sizeof(app.cfg.client_id), "%s", app.cfg.cpu_serial);
            }
            LOGI("device_id set from cpuinfo serial: %s", app.cfg.device_id);
        }
    }

    if (!app.cfg.client_id[0]) {
        snprintf(app.cfg.client_id, sizeof(app.cfg.client_id), "%s", app.cfg.device_id);
    }
    if (app.cfg.keepalive_s < 5) app.cfg.keepalive_s = 5;
    if (app.cfg.tail_poll_ms < 50) app.cfg.tail_poll_ms = 50;
    if (app.cfg.tail_idle_ms < 500) app.cfg.tail_idle_ms = 500;
    if (app.cfg.telemetry_interval_ms < 500) app.cfg.telemetry_interval_ms = 500;
    if (app.cfg.test_interval_ms < 200) app.cfg.test_interval_ms = 200;
    if (!app.cfg.data_dir[0]) app.cfg.tail_enable = false;
    if (!app.cfg.ctrl_path[0]) {
        snprintf(app.cfg.ctrl_path, sizeof(app.cfg.ctrl_path), "%s", "/tmp/modbus_framer_ctrl.json");
    }
    if (!app.cfg.telemetry_disk_path[0]) {
        snprintf(app.cfg.telemetry_disk_path, sizeof(app.cfg.telemetry_disk_path), "%s", "/");
    }

    build_topics(&app);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    pthread_mutex_init(&app.mqtt_lock, NULL);

    MQTTClient_create(&app.mqtt, app.cfg.broker, app.cfg.client_id,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_setCallbacks(app.mqtt, &app, mqtt_connection_lost, mqtt_message_arrived, mqtt_delivery_complete);

    bool telemetry_started = false;
    if (pthread_create(&app.telemetry_thread, NULL, telemetry_publisher_main, &app) != 0) {
        LOGE("telemetry thread create failed");
        MQTTClient_destroy(&app.mqtt);
        pthread_mutex_destroy(&app.mqtt_lock);
        return 1;
    }
    telemetry_started = true;
    if (!app.cfg.telemetry_enable) {
        LOGI("%s", "system telemetry disabled by config, online heartbeat remains enabled");
    }

    bool test_started = false;
    if (app.cfg.test_mode_enable) {
        if (pthread_create(&app.test_thread, NULL, test_publisher_main, &app) != 0) {
            LOGE("test publisher thread create failed");
            g_running = 0;
            if (telemetry_started) pthread_join(app.telemetry_thread, NULL);
            MQTTClient_destroy(&app.mqtt);
            pthread_mutex_destroy(&app.mqtt_lock);
            return 1;
        }
        test_started = true;
    } else {
        LOGI("%s", "test publisher disabled by config");
    }

    bool tail_started = false;
    if (app.cfg.tail_enable) {
        if (pthread_create(&app.tail_thread, NULL, file_tailer_main, &app) != 0) {
            LOGE("file tailer thread create failed");
            g_running = 0;
            if (telemetry_started) pthread_join(app.telemetry_thread, NULL);
            if (test_started) pthread_join(app.test_thread, NULL);
            MQTTClient_destroy(&app.mqtt);
            pthread_mutex_destroy(&app.mqtt_lock);
            return 1;
        }
        tail_started = true;
    } else {
        LOGI("%s", "file tailer disabled by config");
    }

    int backoff_ms = 500;
    while (g_running) {
        pthread_mutex_lock(&app.mqtt_lock);
        bool connected = app.mqtt_connected;
        pthread_mutex_unlock(&app.mqtt_lock);

        if (!connected) {
            int rc = mqtt_connect_and_subscribe(&app);
            if (rc != MQTTCLIENT_SUCCESS) {
                LOGW("mqtt connect failed (%d), retry in %dms", rc, backoff_ms);
                sleep_ms(backoff_ms);
                if (backoff_ms < 8000) backoff_ms *= 2;
                continue;
            }
            backoff_ms = 500;
            LOGI("mqtt connected: %s", app.cfg.broker);
        }
        sleep_ms(1000);
    }

    if (telemetry_started) pthread_join(app.telemetry_thread, NULL);
    if (test_started) pthread_join(app.test_thread, NULL);
    if (tail_started) pthread_join(app.tail_thread, NULL);

    pthread_mutex_lock(&app.mqtt_lock);
    if (app.mqtt_connected) {
        publish_status(&app, "offline", app.cfg.telemetry_interval_ms);
        MQTTClient_disconnect(app.mqtt, 2000);
        app.mqtt_connected = false;
    }
    pthread_mutex_unlock(&app.mqtt_lock);
    MQTTClient_destroy(&app.mqtt);
    pthread_mutex_destroy(&app.mqtt_lock);

    return 0;
}
