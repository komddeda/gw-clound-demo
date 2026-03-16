#include "modbus_framer.h"

#include "../demo_config.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    const char *out_dir;
    const char *device_id;
    const char *ctrl_path;
    int interval_ms;
    int timeout_ms;
    int retries;
    size_t max_file_bytes;
    size_t max_total_bytes;
    int max_records_per_file;
    int func_code; // 3=holding register, 4=input register
    uint16_t temp_addr;
    uint16_t hum_addr;
    int device_addr; // -1 disables "device" field
    bool temp_signed;
} collect_cfg_t;

typedef enum {
    CFG_CMD_NONE = 0,
    CFG_CMD_READ_HOLDING,
    CFG_CMD_READ_INPUT,
    CFG_CMD_WRITE_REG,
    CFG_CMD_WRITE_REGS,
    CFG_CMD_READ_COILS,
    CFG_CMD_WRITE_COIL,
    CFG_CMD_WRITE_COILS,
    CFG_CMD_SCAN,
    CFG_CMD_STRESS,
    CFG_CMD_COLLECT,
} cfg_cmd_t;

typedef struct {
    bool has_addr;
    bool has_qty;
    uint16_t addr;
    uint16_t qty;
} cfg_read_cmd_t;

typedef struct {
    bool has_addr;
    bool has_value;
    uint16_t addr;
    uint16_t value;
} cfg_write_single_cmd_t;

typedef struct {
    bool has_addr;
    bool has_values;
    uint16_t addr;
    uint16_t values[MB_MAX_WRITE_REGS];
    uint16_t count;
} cfg_write_regs_cmd_t;

typedef struct {
    bool has_addr;
    bool has_n_bits;
    bool has_bytes;
    uint16_t addr;
    uint16_t n_bits;
    uint8_t bytes[(MB_MAX_WRITE_COILS + 7u) / 8u];
    uint16_t byte_count;
} cfg_write_coils_cmd_t;

typedef struct {
    uint16_t start;
    uint16_t end;
    int timeout_ms;
} cfg_scan_cmd_t;

typedef struct {
    int duration_s;
    int interval_ms;
    unsigned long long seed;
    int report_s;
} cfg_stress_cmd_t;

typedef struct {
    char dev_buf[PATH_MAX];
    char collect_dir_buf[PATH_MAX];
    char collect_device_id_buf[128];
    char collect_ctrl_path_buf[PATH_MAX];
    cfg_cmd_t command;
    cfg_read_cmd_t read_holding;
    cfg_read_cmd_t read_input;
    cfg_write_single_cmd_t write_reg;
    cfg_write_regs_cmd_t write_regs;
    cfg_read_cmd_t read_coils;
    cfg_write_single_cmd_t write_coil;
    cfg_write_coils_cmd_t write_coils;
    cfg_scan_cmd_t scan;
    cfg_stress_cmd_t stress;
    collect_cfg_t collect;
} modbus_file_cfg_t;

typedef struct {
    mb_line_cfg_t *line;
    uint8_t *slave;
    modbus_file_cfg_t *cfg;
} modbus_config_apply_ctx_t;

// ------------------------ CLI demo ------------------------
static void usage(const char *argv0) {
    fprintf(stderr,
            "Usage: %s [-c PATH] -d DEV -b BAUD [-p N|E|O] [-D 7|8] [-S 1|2] [--rs485] [-v] [-a SLAVE] <cmd> ...\n"
            "       %s --config PATH\n"
            "  cmds:\n"
            "    read-holding <addr> <qty>\n"
            "    read-input   <addr> <qty>\n"
            "    write-reg    <addr> <val>\n"
            "    write-regs   <addr> <n> <v0> <v1> ...\n"
            "    read-coils   <addr> <qty>\n"
            "    write-coil   <addr> <0|1>\n"
            "    write-coils  <addr> <n_bits> <hexbyte0> <hexbyte1> ... (packed LSB-first)\n"
            "    scan [start] [end] [timeout_ms]\n"
            "    stress [seconds] [interval_ms] [seed] [report_s]\n"
            "      seconds=0 means run forever; interval_ms=0 means no sleep\n"
            "    collect [--interval-ms N] [--dir PATH]\n"
            "            [--file-kb N] [--total-kb N] [--records-per-file N] [--device-id STR]\n"
            "            [--func 3|4] [--temp-addr N] [--hum-addr N] [--device-addr N|-1] [--temp-signed]\n",
            argv0, argv0);
}

static long parse_long(const char *s, long minv, long maxv, bool *ok) {
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 0);
    if (errno || !end || *end != '\0' || v < minv || v > maxv) {
        *ok = false;
        return 0;
    }
    *ok = true;
    return v;
}

static bool parse_cfg_bool01(const char *text, bool *out) {
    long value = 0;
    if (demo_config_parse_bool(text, out)) return true;
    if (demo_config_parse_long(text, 0, 1, &value)) {
        *out = (value != 0);
        return true;
    }
    return false;
}

static void cfg_set_collect_string(char *buf, size_t cap, const char **field, const char *value) {
    if (!buf || cap == 0 || !field || !value) return;
    snprintf(buf, cap, "%s", value);
    *field = buf;
}

static void modbus_cfg_defaults(modbus_file_cfg_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->command = CFG_CMD_NONE;
    cfg->scan.start = 1;
    cfg->scan.end = 247;
    cfg->scan.timeout_ms = 50;
    cfg->stress.duration_s = 0;
    cfg->stress.interval_ms = 20;
    cfg->stress.seed = 0;
    cfg->stress.report_s = 5;
    cfg_set_collect_string(cfg->collect_dir_buf, sizeof(cfg->collect_dir_buf), &cfg->collect.out_dir, "./data");
    cfg_set_collect_string(cfg->collect_device_id_buf, sizeof(cfg->collect_device_id_buf), &cfg->collect.device_id, "modbus_framer");
    cfg_set_collect_string(cfg->collect_ctrl_path_buf, sizeof(cfg->collect_ctrl_path_buf), &cfg->collect.ctrl_path, "/tmp/modbus_framer_ctrl.json");
    cfg->collect.interval_ms = 1000;
    cfg->collect.timeout_ms = 1000;
    cfg->collect.retries = 2;
    cfg->collect.max_file_bytes = 200u * 1024u;
    cfg->collect.max_total_bytes = 1024u * 1024u;
    cfg->collect.max_records_per_file = 20;
    cfg->collect.func_code = 4;
    cfg->collect.temp_addr = 0;
    cfg->collect.hum_addr = 1;
    cfg->collect.device_addr = 2;
    cfg->collect.temp_signed = false;
}

static void modbus_set_line_dev(mb_line_cfg_t *line, modbus_file_cfg_t *cfg, const char *value) {
    if (!line || !cfg || !value) return;
    snprintf(cfg->dev_buf, sizeof(cfg->dev_buf), "%s", value);
    line->dev = cfg->dev_buf;
}

static cfg_cmd_t parse_cfg_command_name(const char *value) {
    if (!value || !*value) return CFG_CMD_NONE;
    if (demo_config_name_eq(value, "read-holding")) return CFG_CMD_READ_HOLDING;
    if (demo_config_name_eq(value, "read-input")) return CFG_CMD_READ_INPUT;
    if (demo_config_name_eq(value, "write-reg")) return CFG_CMD_WRITE_REG;
    if (demo_config_name_eq(value, "write-regs")) return CFG_CMD_WRITE_REGS;
    if (demo_config_name_eq(value, "read-coils")) return CFG_CMD_READ_COILS;
    if (demo_config_name_eq(value, "write-coil")) return CFG_CMD_WRITE_COIL;
    if (demo_config_name_eq(value, "write-coils")) return CFG_CMD_WRITE_COILS;
    if (demo_config_name_eq(value, "scan")) return CFG_CMD_SCAN;
    if (demo_config_name_eq(value, "stress")) return CFG_CMD_STRESS;
    if (demo_config_name_eq(value, "collect")) return CFG_CMD_COLLECT;
    return CFG_CMD_NONE;
}

static int parse_u16_csv(const char *text, uint16_t *out, size_t cap, uint16_t *count) {
    char buf[8192];
    char *saveptr = NULL;
    char *token = NULL;
    uint16_t parsed = 0;

    if (!text || !*text || !out || cap == 0 || !count) return -1;
    if (strlen(text) >= sizeof(buf)) return -1;
    snprintf(buf, sizeof(buf), "%s", text);

    token = strtok_r(buf, ",", &saveptr);
    while (token) {
        long value = 0;
        token = demo_config_trim(token);
        if (*token == '\0' || parsed >= cap || !demo_config_parse_long(token, 0, 65535, &value)) {
            return -1;
        }
        out[parsed++] = (uint16_t)value;
        token = strtok_r(NULL, ",", &saveptr);
    }

    if (parsed == 0) return -1;
    *count = parsed;
    return 0;
}

static int parse_u8_csv(const char *text, uint8_t *out, size_t cap, uint16_t *count) {
    char buf[8192];
    char *saveptr = NULL;
    char *token = NULL;
    uint16_t parsed = 0;

    if (!text || !*text || !out || cap == 0 || !count) return -1;
    if (strlen(text) >= sizeof(buf)) return -1;
    snprintf(buf, sizeof(buf), "%s", text);

    token = strtok_r(buf, ",", &saveptr);
    while (token) {
        long value = 0;
        token = demo_config_trim(token);
        if (*token == '\0' || parsed >= cap || !demo_config_parse_long(token, 0, 255, &value)) {
            return -1;
        }
        out[parsed++] = (uint8_t)value;
        token = strtok_r(NULL, ",", &saveptr);
    }

    if (parsed == 0) return -1;
    *count = parsed;
    return 0;
}

static int modbus_apply_config_entry(const char *section,
                                     const char *key,
                                     const char *value,
                                     void *user,
                                     char *errbuf,
                                     size_t errcap) {
    modbus_config_apply_ctx_t *ctx = (modbus_config_apply_ctx_t *)user;
    modbus_file_cfg_t *cfg;
    long lv = 0;
    bool bv = false;

    if (!ctx || !ctx->line || !ctx->slave || !ctx->cfg || !key || !value) return -1;
    cfg = ctx->cfg;

    if (!section || !section[0] ||
        demo_config_name_eq(section, "serial") ||
        demo_config_name_eq(section, "common")) {
        if (demo_config_name_eq(key, "dev") || demo_config_name_eq(key, "port")) {
            modbus_set_line_dev(ctx->line, cfg, value);
            return 0;
        }
        if (demo_config_name_eq(key, "baud")) {
            if (!demo_config_parse_long(value, 1, 4000000, &lv)) goto bad_value;
            ctx->line->baud = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "parity")) {
            char parity = value[0];
            if (value[1] != '\0') goto bad_value;
            if (parity >= 'a' && parity <= 'z') parity = (char)(parity - 'a' + 'A');
            if (parity != 'N' && parity != 'E' && parity != 'O') goto bad_value;
            ctx->line->parity = parity;
            return 0;
        }
        if (demo_config_name_eq(key, "data_bits")) {
            if (!demo_config_parse_long(value, 7, 8, &lv)) goto bad_value;
            ctx->line->data_bits = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "stop_bits")) {
            if (!demo_config_parse_long(value, 1, 2, &lv)) goto bad_value;
            ctx->line->stop_bits = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "rs485") || demo_config_name_eq(key, "rs485_enable")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            ctx->line->rs485_enable = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "debug")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            ctx->line->debug = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "slave")) {
            if (!demo_config_parse_long(value, 1, 247, &lv)) goto bad_value;
            *ctx->slave = (uint8_t)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "command")) {
            cfg->command = parse_cfg_command_name(value);
            if (cfg->command == CFG_CMD_NONE) goto bad_value;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "command")) {
        if (demo_config_name_eq(key, "name") || demo_config_name_eq(key, "cmd")) {
            cfg->command = parse_cfg_command_name(value);
            if (cfg->command == CFG_CMD_NONE) goto bad_value;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "read_holding")) {
        if (demo_config_name_eq(key, "addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->read_holding.addr = (uint16_t)lv;
            cfg->read_holding.has_addr = true;
            return 0;
        }
        if (demo_config_name_eq(key, "qty")) {
            if (!demo_config_parse_long(value, 1, 125, &lv)) goto bad_value;
            cfg->read_holding.qty = (uint16_t)lv;
            cfg->read_holding.has_qty = true;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "read_input")) {
        if (demo_config_name_eq(key, "addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->read_input.addr = (uint16_t)lv;
            cfg->read_input.has_addr = true;
            return 0;
        }
        if (demo_config_name_eq(key, "qty")) {
            if (!demo_config_parse_long(value, 1, 125, &lv)) goto bad_value;
            cfg->read_input.qty = (uint16_t)lv;
            cfg->read_input.has_qty = true;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "write_reg")) {
        if (demo_config_name_eq(key, "addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->write_reg.addr = (uint16_t)lv;
            cfg->write_reg.has_addr = true;
            return 0;
        }
        if (demo_config_name_eq(key, "value") || demo_config_name_eq(key, "val")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->write_reg.value = (uint16_t)lv;
            cfg->write_reg.has_value = true;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "write_regs")) {
        if (demo_config_name_eq(key, "addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->write_regs.addr = (uint16_t)lv;
            cfg->write_regs.has_addr = true;
            return 0;
        }
        if (demo_config_name_eq(key, "values")) {
            if (parse_u16_csv(value, cfg->write_regs.values, MB_MAX_WRITE_REGS, &cfg->write_regs.count) != 0) {
                goto bad_value;
            }
            cfg->write_regs.has_values = true;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "read_coils")) {
        if (demo_config_name_eq(key, "addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->read_coils.addr = (uint16_t)lv;
            cfg->read_coils.has_addr = true;
            return 0;
        }
        if (demo_config_name_eq(key, "qty")) {
            if (!demo_config_parse_long(value, 1, MB_MAX_READ_BITS, &lv)) goto bad_value;
            cfg->read_coils.qty = (uint16_t)lv;
            cfg->read_coils.has_qty = true;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "write_coil")) {
        if (demo_config_name_eq(key, "addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->write_coil.addr = (uint16_t)lv;
            cfg->write_coil.has_addr = true;
            return 0;
        }
        if (demo_config_name_eq(key, "value") || demo_config_name_eq(key, "val")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->write_coil.value = bv ? 1u : 0u;
            cfg->write_coil.has_value = true;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "write_coils")) {
        if (demo_config_name_eq(key, "addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->write_coils.addr = (uint16_t)lv;
            cfg->write_coils.has_addr = true;
            return 0;
        }
        if (demo_config_name_eq(key, "n_bits")) {
            if (!demo_config_parse_long(value, 1, MB_MAX_WRITE_COILS, &lv)) goto bad_value;
            cfg->write_coils.n_bits = (uint16_t)lv;
            cfg->write_coils.has_n_bits = true;
            return 0;
        }
        if (demo_config_name_eq(key, "bytes")) {
            if (parse_u8_csv(value, cfg->write_coils.bytes, sizeof(cfg->write_coils.bytes), &cfg->write_coils.byte_count) != 0) {
                goto bad_value;
            }
            cfg->write_coils.has_bytes = true;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "scan")) {
        if (demo_config_name_eq(key, "start")) {
            if (!demo_config_parse_long(value, 1, 247, &lv)) goto bad_value;
            cfg->scan.start = (uint16_t)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "end")) {
            if (!demo_config_parse_long(value, 1, 247, &lv)) goto bad_value;
            cfg->scan.end = (uint16_t)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "timeout_ms")) {
            if (!demo_config_parse_long(value, 1, 60000, &lv)) goto bad_value;
            cfg->scan.timeout_ms = (int)lv;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "stress")) {
        if (demo_config_name_eq(key, "duration_s")) {
            if (!demo_config_parse_long(value, 0, 86400, &lv)) goto bad_value;
            cfg->stress.duration_s = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "interval_ms")) {
            if (!demo_config_parse_long(value, 0, 10000, &lv)) goto bad_value;
            cfg->stress.interval_ms = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "seed")) {
            if (!demo_config_parse_ull(value, &cfg->stress.seed)) goto bad_value;
            return 0;
        }
        if (demo_config_name_eq(key, "report_s")) {
            if (!demo_config_parse_long(value, 1, 3600, &lv)) goto bad_value;
            cfg->stress.report_s = (int)lv;
            return 0;
        }
    }

    if (demo_config_name_eq(section, "collect")) {
        if (demo_config_name_eq(key, "interval_ms") || demo_config_name_eq(key, "telemetry_interval_ms")) {
            if (!demo_config_parse_long(value, 10, 600000, &lv)) goto bad_value;
            cfg->collect.interval_ms = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "dir") || demo_config_name_eq(key, "out_dir") || demo_config_name_eq(key, "data_dir")) {
            cfg_set_collect_string(cfg->collect_dir_buf, sizeof(cfg->collect_dir_buf), &cfg->collect.out_dir, value);
            return 0;
        }
        if (demo_config_name_eq(key, "device_id")) {
            cfg_set_collect_string(cfg->collect_device_id_buf, sizeof(cfg->collect_device_id_buf), &cfg->collect.device_id, value);
            return 0;
        }
        if (demo_config_name_eq(key, "ctrl_file") || demo_config_name_eq(key, "ctrl_path")) {
            cfg_set_collect_string(cfg->collect_ctrl_path_buf, sizeof(cfg->collect_ctrl_path_buf), &cfg->collect.ctrl_path, value);
            return 0;
        }
        if (demo_config_name_eq(key, "file_kb")) {
            if (!demo_config_parse_long(value, 1, 10240, &lv)) goto bad_value;
            cfg->collect.max_file_bytes = (size_t)lv * 1024u;
            return 0;
        }
        if (demo_config_name_eq(key, "total_kb")) {
            if (!demo_config_parse_long(value, 1, 10240, &lv)) goto bad_value;
            cfg->collect.max_total_bytes = (size_t)lv * 1024u;
            return 0;
        }
        if (demo_config_name_eq(key, "records_per_file")) {
            if (!demo_config_parse_long(value, 1, 100000, &lv)) goto bad_value;
            cfg->collect.max_records_per_file = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "func") || demo_config_name_eq(key, "func_code")) {
            if (!demo_config_parse_long(value, 3, 4, &lv)) goto bad_value;
            cfg->collect.func_code = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "temp_addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->collect.temp_addr = (uint16_t)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "hum_addr")) {
            if (!demo_config_parse_long(value, 0, 65535, &lv)) goto bad_value;
            cfg->collect.hum_addr = (uint16_t)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "device_addr")) {
            if (!demo_config_parse_long(value, -1, 65535, &lv)) goto bad_value;
            cfg->collect.device_addr = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "temp_signed")) {
            if (!parse_cfg_bool01(value, &bv)) goto bad_value;
            cfg->collect.temp_signed = bv;
            return 0;
        }
        if (demo_config_name_eq(key, "timeout_ms")) {
            if (!demo_config_parse_long(value, 1, 60000, &lv)) goto bad_value;
            cfg->collect.timeout_ms = (int)lv;
            return 0;
        }
        if (demo_config_name_eq(key, "retries")) {
            if (!demo_config_parse_long(value, 0, 100, &lv)) goto bad_value;
            cfg->collect.retries = (int)lv;
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

static const char *modbus_find_config_path(int argc, char **argv) {
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

static bool modbus_is_config_arg(const char *arg) {
    return !strcmp(arg, "-c") || !strcmp(arg, "--config") || !strncmp(arg, "--config=", 9);
}

static uint64_t cli_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void cli_sleep_us(uint64_t us) {
    struct timespec ts;
    ts.tv_sec = (time_t)(us / 1000000ull);
    ts.tv_nsec = (long)((us % 1000000ull) * 1000ull);
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}
}

static uint64_t xorshift64(uint64_t *state) {
    uint64_t x = *state;
    if (x == 0) x = 0x9e3779b97f4a7c15ull;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    return x * 2685821657736338717ull;
}

// ------------------------ collect demo ------------------------
static volatile sig_atomic_t g_collect_running = 1;
static void on_collect_signal(int sig) {
    (void)sig;
    g_collect_running = 0;
}

typedef struct {
    FILE *fp;
    char current_path[PATH_MAX];
    size_t current_size;
    uint32_t seq;
    int record_count;
} log_state_t;

static void format_iso8601(char *buf, size_t cap, int64_t *epoch_ms) {
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

static int mkdir_p(const char *path) {
    if (!path || !*path) return -1;
    char tmp[PATH_MAX];
    size_t len = strnlen(path, sizeof(tmp));
    if (len == 0 || len >= sizeof(tmp)) return -1;
    memcpy(tmp, path, len);
    tmp[len] = '\0';
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
    struct stat st;
    if (stat(tmp, &st) != 0 || !S_ISDIR(st.st_mode)) return -1;
    return 0;
}

static void join_path(char *dst, size_t cap, const char *dir, const char *name) {
    size_t len = strlen(dir);
    if (len > 0 && dir[len - 1] == '/') {
        snprintf(dst, cap, "%s%s", dir, name);
    } else {
        snprintf(dst, cap, "%s/%s", dir, name);
    }
}

static int open_new_log(log_state_t *st, const collect_cfg_t *cfg) {
    if (mkdir_p(cfg->out_dir) != 0) {
        LOGE("mkdir failed: %s", cfg->out_dir);
        return -1;
    }
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
    char name[64];
    snprintf(name, sizeof(name), "data_%s_%u.jsonl", ts, st->seq++);
    join_path(st->current_path, sizeof(st->current_path), cfg->out_dir, name);

    st->fp = fopen(st->current_path, "a");
    if (!st->fp) {
        LOGE("open log failed: %s", strerror(errno));
        return -1;
    }
    fseek(st->fp, 0, SEEK_END);
    long pos = ftell(st->fp);
    st->current_size = (pos > 0) ? (size_t)pos : 0;
    st->record_count = 0;
    return 0;
}

static int log_append(log_state_t *st, const char *line, size_t len) {
    if (!st->fp) return -1;
    if (fwrite(line, 1, len, st->fp) != len) return -1;
    if (fwrite("\n", 1, 1, st->fp) != 1) return -1;
    st->current_size += len + 1;
    fflush(st->fp);
    fsync(fileno(st->fp));
    return 0;
}

static int rotate_if_needed(log_state_t *st, const collect_cfg_t *cfg) {
    if (cfg->max_records_per_file > 0 && st->record_count >= cfg->max_records_per_file) {
        // rotate by count
    } else if (cfg->max_file_bytes == 0 || st->current_size < cfg->max_file_bytes) {
        return 0;
    }
    fflush(st->fp);
    fsync(fileno(st->fp));
    fclose(st->fp);
    st->fp = NULL;
    st->current_path[0] = '\0';
    return open_new_log(st, cfg);
}

static void json_escape(const char *src, char *dst, size_t cap) {
    size_t j = 0;
    if (!src) src = "";
    for (size_t i = 0; src[i] != '\0' && j + 1 < cap; i++) {
        unsigned char c = (unsigned char)src[i];
        if (c < 0x20) c = ' ';
        if (c == '"' || c == '\\') {
            if (j + 2 >= cap) break;
            dst[j++] = '\\';
            dst[j++] = (char)c;
        } else {
            dst[j++] = (char)c;
        }
    }
    dst[j] = '\0';
}

static int build_record_line(char *buf, size_t cap,
                             double temp_c, double hum_pct,
                             bool include_device, uint16_t dev,
                             uint8_t slave, const char *device_id) {
    char ts[32];
    int64_t epoch_ms = 0;
    char esc_id[64];
    if (!device_id) device_id = "modbus_framer";
    json_escape(device_id, esc_id, sizeof(esc_id));
    format_iso8601(ts, sizeof(ts), &epoch_ms);
    int n = 0;
    if (include_device) {
        n = snprintf(buf, cap,
                     "{\"ts\":\"%s\",\"epoch_ms\":%" PRId64 ",\"slave\":%u,"
                     "\"device_id\":\"%s\",\"temp_c\":%.1f,\"hum_pct\":%.1f,\"device\":%u}",
                     ts, epoch_ms, slave, esc_id, temp_c, hum_pct, (unsigned)dev);
    } else {
        n = snprintf(buf, cap,
                     "{\"ts\":\"%s\",\"epoch_ms\":%" PRId64 ",\"slave\":%u,"
                     "\"device_id\":\"%s\",\"temp_c\":%.1f,\"hum_pct\":%.1f}",
                     ts, epoch_ms, slave, esc_id, temp_c, hum_pct);
    }
    return (n > 0 && (size_t)n < cap) ? n : -1;
}

static int collect_read_reg(mb_ctx_t *ctx, uint8_t slave, const collect_cfg_t *cfg,
                            uint16_t addr, uint16_t *out, uint8_t *exc) {
    if (!ctx || !cfg || !out) return MB_ERR_PARAM;
    if (cfg->func_code == 3) {
        return mb_read_holding(ctx, slave, addr, 1, out, 1, cfg->timeout_ms, cfg->retries, exc);
    }
    return mb_read_input(ctx, slave, addr, 1, out, 1, cfg->timeout_ms, cfg->retries, exc);
}

static double collect_decode_temp_c(uint16_t raw, bool signed_temp) {
    int32_t v = signed_temp ? (int16_t)raw : (int32_t)raw;
    return v / 10.0;
}

static double collect_decode_hum_pct(uint16_t raw) {
    return raw / 10.0;
}

static int collect_read_snapshot(mb_ctx_t *ctx, uint8_t slave, const collect_cfg_t *cfg,
                                 uint16_t *temp_raw, uint16_t *hum_raw, uint16_t *dev_raw,
                                 uint8_t *exc) {
    if (!ctx || !cfg || !temp_raw || !hum_raw) return MB_ERR_PARAM;

    uint16_t min_addr = cfg->temp_addr;
    uint16_t max_addr = cfg->temp_addr;
    if (cfg->hum_addr < min_addr) min_addr = cfg->hum_addr;
    if (cfg->hum_addr > max_addr) max_addr = cfg->hum_addr;
    if (cfg->device_addr >= 0) {
        uint16_t da = (uint16_t)cfg->device_addr;
        if (da < min_addr) min_addr = da;
        if (da > max_addr) max_addr = da;
    }

    uint16_t qty = (uint16_t)(max_addr - min_addr + 1u);
    if (qty <= 125u) {
        uint16_t regs[125] = {0};
        int rc = (cfg->func_code == 3)
               ? mb_read_holding(ctx, slave, min_addr, qty, regs, 125, cfg->timeout_ms, cfg->retries, exc)
               : mb_read_input(ctx, slave, min_addr, qty, regs, 125, cfg->timeout_ms, cfg->retries, exc);
        if (rc != MB_OK) return rc;

        *hum_raw = regs[cfg->hum_addr - min_addr];
        *temp_raw = regs[cfg->temp_addr - min_addr];
        if (cfg->device_addr >= 0 && dev_raw) {
            *dev_raw = regs[(uint16_t)cfg->device_addr - min_addr];
        }
        return MB_OK;
    }

    int rc = collect_read_reg(ctx, slave, cfg, cfg->hum_addr, hum_raw, exc);
    if (rc == MB_OK) rc = collect_read_reg(ctx, slave, cfg, cfg->temp_addr, temp_raw, exc);
    if (rc == MB_OK && cfg->device_addr >= 0 && dev_raw) {
        rc = collect_read_reg(ctx, slave, cfg, (uint16_t)cfg->device_addr, dev_raw, exc);
    }
    return rc;
}

static bool has_prefix(const char *s, const char *p) {
    size_t lp = strlen(p);
    return strncmp(s, p, lp) == 0;
}

static bool has_suffix(const char *s, const char *suf) {
    size_t ls = strlen(s);
    size_t lf = strlen(suf);
    if (ls < lf) return false;
    return strcmp(s + (ls - lf), suf) == 0;
}

static bool json_find_int(const char *buf, const char *key, int *out) {
    if (!buf || !key || !out) return false;
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(buf, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p && isspace((unsigned char)*p)) p++;
    if (*p != ':') return false;
    p++;
    while (*p && isspace((unsigned char)*p)) p++;
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (p == end) return false;
    *out = (int)v;
    return true;
}

static void apply_ctrl_file(collect_cfg_t *cfg, time_t *last_mtime) {
    if (!cfg || !cfg->ctrl_path || cfg->ctrl_path[0] == '\0') return;
    struct stat st;
    if (stat(cfg->ctrl_path, &st) != 0) {
        return;
    }
    if (*last_mtime != 0 && st.st_mtime == *last_mtime) return;
    *last_mtime = st.st_mtime;

    FILE *fp = fopen(cfg->ctrl_path, "rb");
    if (!fp) return;
    char buf[512];
    size_t rd = fread(buf, 1, sizeof(buf) - 1, fp);
    fclose(fp);
    buf[rd] = '\0';

    int interval_ms = 0;
    int records = 0;
    bool has_interval = json_find_int(buf, "interval_ms", &interval_ms) ||
                        json_find_int(buf, "telemetry_interval_ms", &interval_ms);
    bool has_records = json_find_int(buf, "records_per_file", &records);

    if (has_interval && interval_ms > 0 && interval_ms != cfg->interval_ms) {
        cfg->interval_ms = interval_ms;
        LOGI("ctrl update applied: interval_ms=%d", interval_ms);
    }
    if (has_records && records > 0 && records != cfg->max_records_per_file) {
        cfg->max_records_per_file = records;
        LOGI("ctrl update applied: records_per_file=%d", records);
    }
}

static size_t calc_total_bytes(const char *dir) {
    DIR *d = opendir(dir);
    if (!d) return 0;
    size_t total = 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        if (!has_prefix(de->d_name, "data_") || !has_suffix(de->d_name, ".jsonl")) continue;
        char path[PATH_MAX];
        join_path(path, sizeof(path), dir, de->d_name);
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) continue;
        total += (size_t)st.st_size;
    }
    closedir(d);
    return total;
}

static int run_collect(mb_ctx_t *ctx, uint8_t slave, collect_cfg_t *cfg) {
    log_state_t st;
    memset(&st, 0, sizeof(st));
    g_collect_running = 1;
    if (open_new_log(&st, cfg) != 0) return MB_ERR_IO;

    signal(SIGINT, on_collect_signal);
    signal(SIGTERM, on_collect_signal);

    uint64_t last_size_check = 0;
    uint64_t last_ctrl_check = 0;
    uint64_t last_read_warn_us = 0;
    int last_read_warn_rc = MB_OK;
    uint8_t last_read_warn_exc = 0;
    time_t last_ctrl_mtime = 0;
    while (g_collect_running) {
        uint16_t temp_raw = 0;
        uint16_t hum_raw = 0;
        uint16_t dev_raw = 0;
        uint8_t exc = 0;
        int rc = collect_read_snapshot(ctx, slave, cfg, &temp_raw, &hum_raw, &dev_raw, &exc);
        if (rc == MB_OK) {
            char line[256];
            double temp_c = collect_decode_temp_c(temp_raw, cfg->temp_signed);
            double hum_pct = collect_decode_hum_pct(hum_raw);
            int n = build_record_line(line, sizeof(line),
                                      temp_c, hum_pct,
                                      cfg->device_addr >= 0, dev_raw,
                                      slave, cfg->device_id);
            if (n > 0) {
                if (log_append(&st, line, (size_t)n) != 0) {
                    LOGW("log write failed: %s", strerror(errno));
                } else {
                    st.record_count++;
                }
                if (rotate_if_needed(&st, cfg) != 0) {
                    LOGW("log rotate failed");
                }
            } else {
                LOGW("%s", "record format failed");
            }
        } else if (rc == MB_ERR_EXCEPTION) {
            uint64_t now = cli_now_us();
            if (last_read_warn_rc != rc ||
                last_read_warn_exc != exc ||
                now - last_read_warn_us >= 10000000ull) {
                LOGW("modbus exception: 0x%02X", exc);
                last_read_warn_us = now;
                last_read_warn_rc = rc;
                last_read_warn_exc = exc;
            }
        } else {
            uint64_t now = cli_now_us();
            if (last_read_warn_rc != rc || now - last_read_warn_us >= 10000000ull) {
                LOGW("modbus read failed: %s", mb_status_str(rc));
                last_read_warn_us = now;
                last_read_warn_rc = rc;
                last_read_warn_exc = 0;
            }
        }

        uint64_t now = cli_now_us();
        if (cfg->ctrl_path && now - last_ctrl_check >= 500000ull) {
            apply_ctrl_file(cfg, &last_ctrl_mtime);
            last_ctrl_check = now;
        }
        if (cfg->max_total_bytes > 0 && now - last_size_check >= 10000000ull) {
            size_t total = calc_total_bytes(cfg->out_dir);
            if (total > cfg->max_total_bytes) {
                LOGW("log size %zu exceeds limit %zu; wait for upload before deleting old files",
                     total, cfg->max_total_bytes);
            }
            last_size_check = now;
        }

        if (cfg->interval_ms > 0) {
            cli_sleep_us((uint64_t)cfg->interval_ms * 1000ull);
        }
    }

    if (st.fp) {
        fflush(st.fp);
        fsync(fileno(st.fp));
        fclose(st.fp);
    }
    return MB_OK;
}

static int run_stress(mb_ctx_t *ctx, uint8_t slave,
                      int duration_s, int interval_ms,
                      int timeout_ms, int retries,
                      uint64_t seed, int report_s) {
    uint64_t start = cli_now_us();
    uint64_t end_us = (duration_s <= 0) ? 0 : start + (uint64_t)duration_s * 1000000ull;
    uint64_t next_report = start + (uint64_t)report_s * 1000000ull;
    uint16_t mailbox[8];
    uint16_t echo[9];
    uint8_t coils[2];
    uint32_t loops = 0;
    uint32_t ok = 0;
    uint32_t err = 0;
    uint32_t exc = 0;
    uint32_t mismatch = 0;
    uint32_t ack_err = 0;
    uint32_t coil_err = 0;
    uint16_t last_ack = 0;
    bool ack_init = false;
    uint16_t seq = 0;

    if (seed == 0) seed = cli_now_us();
    if (report_s < 1) report_s = 5;

    while (end_us == 0 || cli_now_us() < end_us) {
        loops++;

        mailbox[0] = seq++;
        for (int i = 1; i < 8; i++) {
            mailbox[i] = (uint16_t)(xorshift64(&seed) & 0xFFFFu);
        }

        uint8_t exc_code = 0;
        int rc = mb_write_multi_regs(ctx, slave, 200, mailbox, 8, timeout_ms, retries, &exc_code);
        if (rc == MB_ERR_EXCEPTION) { exc++; goto stress_wait; }
        if (rc != MB_OK) { err++; goto stress_wait; }

        rc = mb_read_input(ctx, slave, 200, 9, echo, 9, timeout_ms, retries, &exc_code);
        if (rc == MB_ERR_EXCEPTION) { exc++; goto stress_wait; }
        if (rc != MB_OK) { err++; goto stress_wait; }

        for (int i = 0; i < 8; i++) {
            if (echo[i] != mailbox[i]) {
                mismatch++;
                err++;
                break;
            }
        }
        if (ack_init && echo[8] == last_ack) {
            ack_err++;
            err++;
        }
        ack_init = true;
        last_ack = echo[8];

        uint16_t bit = (uint16_t)(xorshift64(&seed) & 0x0Fu);
        bool on = (xorshift64(&seed) & 1u) != 0u;
        rc = mb_write_single_coil(ctx, slave, bit, on, timeout_ms, retries, &exc_code);
        if (rc == MB_ERR_EXCEPTION) { exc++; goto stress_wait; }
        if (rc != MB_OK) { err++; goto stress_wait; }

        rc = mb_read_coils(ctx, slave, 0, 16, coils, (uint16_t)sizeof(coils), timeout_ms, retries, &exc_code);
        if (rc == MB_ERR_EXCEPTION) { exc++; goto stress_wait; }
        if (rc != MB_OK) { err++; goto stress_wait; }

        int read_on = (coils[bit / 8u] >> (bit % 8u)) & 0x01;
        if (read_on != (int)on) {
            coil_err++;
            err++;
        }

        ok++;

stress_wait:
        uint64_t now = cli_now_us();
        if (now >= next_report) {
            LOGI("stress: loops=%u ok=%u err=%u exc=%u mismatch=%u ack_err=%u coil_err=%u",
                 loops, ok, err, exc, mismatch, ack_err, coil_err);
            next_report = now + (uint64_t)report_s * 1000000ull;
        }

        if (interval_ms > 0) {
            cli_sleep_us((uint64_t)interval_ms * 1000ull);
        }
    }

    LOGI("stress done: loops=%u ok=%u err=%u exc=%u mismatch=%u ack_err=%u coil_err=%u",
         loops, ok, err, exc, mismatch, ack_err, coil_err);
    return (err == 0) ? MB_OK : MB_ERR_PROTO;
}

static int run_config_command(mb_ctx_t *ctx, uint8_t slave, const modbus_file_cfg_t *cfg, uint8_t *exc) {
    const int timeout_ms = 1000;
    const int retries = 2;
    int rc = MB_ERR_PARAM;

    if (!ctx || !cfg || !exc) return MB_ERR_PARAM;

    switch (cfg->command) {
    case CFG_CMD_READ_HOLDING: {
        uint16_t regs[125];
        if (!cfg->read_holding.has_addr || !cfg->read_holding.has_qty) {
            LOGE("%s", "config command read-holding requires addr and qty");
            return MB_ERR_PARAM;
        }
        rc = mb_read_holding(ctx, slave, cfg->read_holding.addr, cfg->read_holding.qty,
                             regs, 125, timeout_ms, retries, exc);
        if (rc == MB_OK) {
            for (uint16_t k = 0; k < cfg->read_holding.qty; k++) {
                printf("reg[%u]=%u\n", (unsigned)(cfg->read_holding.addr + k), regs[k]);
            }
        }
        return rc;
    }
    case CFG_CMD_READ_INPUT: {
        uint16_t regs[125];
        if (!cfg->read_input.has_addr || !cfg->read_input.has_qty) {
            LOGE("%s", "config command read-input requires addr and qty");
            return MB_ERR_PARAM;
        }
        rc = mb_read_input(ctx, slave, cfg->read_input.addr, cfg->read_input.qty,
                           regs, 125, timeout_ms, retries, exc);
        if (rc == MB_OK) {
            for (uint16_t k = 0; k < cfg->read_input.qty; k++) {
                printf("in[%u]=%u\n", (unsigned)(cfg->read_input.addr + k), regs[k]);
            }
        }
        return rc;
    }
    case CFG_CMD_WRITE_REG:
        if (!cfg->write_reg.has_addr || !cfg->write_reg.has_value) {
            LOGE("%s", "config command write-reg requires addr and value");
            return MB_ERR_PARAM;
        }
        return mb_write_single_reg(ctx, slave, cfg->write_reg.addr, cfg->write_reg.value,
                                   timeout_ms, retries, exc);
    case CFG_CMD_WRITE_REGS:
        if (!cfg->write_regs.has_addr || !cfg->write_regs.has_values || cfg->write_regs.count == 0) {
            LOGE("%s", "config command write-regs requires addr and values");
            return MB_ERR_PARAM;
        }
        return mb_write_multi_regs(ctx, slave, cfg->write_regs.addr,
                                   cfg->write_regs.values, cfg->write_regs.count,
                                   timeout_ms, retries, exc);
    case CFG_CMD_READ_COILS: {
        uint8_t bits[(MB_MAX_READ_BITS + 7u) / 8u];
        if (!cfg->read_coils.has_addr || !cfg->read_coils.has_qty) {
            LOGE("%s", "config command read-coils requires addr and qty");
            return MB_ERR_PARAM;
        }
        rc = mb_read_coils(ctx, slave, cfg->read_coils.addr, cfg->read_coils.qty,
                           bits, sizeof(bits), timeout_ms, retries, exc);
        if (rc == MB_OK) {
            for (uint16_t k = 0; k < cfg->read_coils.qty; k++) {
                uint8_t b = bits[k / 8] >> (k % 8);
                printf("coil[%u]=%u\n", (unsigned)(cfg->read_coils.addr + k), (unsigned)(b & 1u));
            }
        }
        return rc;
    }
    case CFG_CMD_WRITE_COIL:
        if (!cfg->write_coil.has_addr || !cfg->write_coil.has_value) {
            LOGE("%s", "config command write-coil requires addr and value");
            return MB_ERR_PARAM;
        }
        return mb_write_single_coil(ctx, slave, cfg->write_coil.addr,
                                    cfg->write_coil.value != 0, timeout_ms, retries, exc);
    case CFG_CMD_WRITE_COILS: {
        uint16_t need_bytes;
        if (!cfg->write_coils.has_addr || !cfg->write_coils.has_n_bits || !cfg->write_coils.has_bytes) {
            LOGE("%s", "config command write-coils requires addr, n_bits and bytes");
            return MB_ERR_PARAM;
        }
        need_bytes = (uint16_t)((cfg->write_coils.n_bits + 7u) / 8u);
        if (cfg->write_coils.byte_count != need_bytes) {
            LOGE("config write-coils bytes count mismatch: need %u got %u",
                 (unsigned)need_bytes, (unsigned)cfg->write_coils.byte_count);
            return MB_ERR_PARAM;
        }
        return mb_write_multi_coils(ctx, slave, cfg->write_coils.addr,
                                    cfg->write_coils.bytes, cfg->write_coils.n_bits,
                                    timeout_ms, retries, exc);
    }
    case CFG_CMD_SCAN:
        if (cfg->scan.start > cfg->scan.end) {
            LOGE("%s", "config scan start must be <= end");
            return MB_ERR_PARAM;
        }
        return mb_scan_slaves(ctx, (uint8_t)cfg->scan.start, (uint8_t)cfg->scan.end, cfg->scan.timeout_ms);
    case CFG_CMD_STRESS:
        return run_stress(ctx, slave, cfg->stress.duration_s, cfg->stress.interval_ms,
                          timeout_ms, retries, (uint64_t)cfg->stress.seed, cfg->stress.report_s);
    case CFG_CMD_COLLECT: {
        collect_cfg_t collect = cfg->collect;
        if (collect.max_total_bytes < collect.max_file_bytes) {
            collect.max_total_bytes = collect.max_file_bytes;
        }
        return run_collect(ctx, slave, &collect);
    }
    case CFG_CMD_NONE:
    default:
        LOGE("%s", "config file does not define a valid command");
        return MB_ERR_PARAM;
    }
}

int main(int argc, char **argv) {
    mb_line_cfg_t line = {
        .dev = "/dev/ttyS1",
        .baud = 115200,
        .parity = 'E',
        .data_bits = 8,
        .stop_bits = 1,
        .rs485_enable = false,
        .debug = false,
        .t15_min_us = 0,
        .broadcast_delay_us = 0,
        .reconnect_delay_ms = 0,
        .reconnect_max = 0,
        .t15_us = 0,
        .t35_us = 0,
    };

    uint8_t slave = 1;
    modbus_file_cfg_t file_cfg;
    modbus_config_apply_ctx_t config_ctx = {
        .line = &line,
        .slave = &slave,
        .cfg = &file_cfg,
    };
    const char *config_path = NULL;
    bool use_config_command = false;

    modbus_cfg_defaults(&file_cfg);
    config_path = modbus_find_config_path(argc, argv);
    if ((config_path && !*config_path) || ((argc > 1) &&
        ((strcmp(argv[argc - 1], "-c") == 0) || (strcmp(argv[argc - 1], "--config") == 0)))) {
        usage(argv[0]);
        return 2;
    }
    if (config_path) {
        char errbuf[256];
        if (demo_config_load(config_path, modbus_apply_config_entry, &config_ctx, errbuf, sizeof(errbuf)) != 0) {
            LOGE("load config failed: %s", errbuf[0] ? errbuf : config_path);
            return 2;
        }
    }

    int i = 1;
    // Parse global options; command begins at first non-option token.
    while (i < argc) {
        if (modbus_is_config_arg(argv[i])) {
            if (!strcmp(argv[i], "-c") || !strcmp(argv[i], "--config")) i += 2;
            else i += 1;
            continue;
        }
        if (!strcmp(argv[i], "-d") && i + 1 < argc) { line.dev = argv[i + 1]; i += 2; continue; }
        if (!strcmp(argv[i], "-b") && i + 1 < argc) { line.baud = atoi(argv[i + 1]); i += 2; continue; }
        if (!strcmp(argv[i], "-p") && i + 1 < argc) { line.parity = argv[i + 1][0]; i += 2; continue; }
        if (!strcmp(argv[i], "-D") && i + 1 < argc) { line.data_bits = atoi(argv[i + 1]); i += 2; continue; }
        if (!strcmp(argv[i], "-S") && i + 1 < argc) { line.stop_bits = atoi(argv[i + 1]); i += 2; continue; }
        if (!strcmp(argv[i], "--rs485")) { line.rs485_enable = true; i += 1; continue; }
        if (!strcmp(argv[i], "-v")) { line.debug = true; i += 1; continue; }
        if (!strcmp(argv[i], "-a") && i + 1 < argc) { slave = (uint8_t)atoi(argv[i + 1]); i += 2; continue; }
        break;
    }

    if (i >= argc) {
        if (file_cfg.command == CFG_CMD_NONE) {
            usage(argv[0]);
            return 2;
        }
        use_config_command = true;
    }

    if ((line.parity == 'N' || line.parity == 'n') && line.stop_bits == 1) {
        LOGW("%s", "No parity often uses 2 stop bits for maximum compatibility.");
    }

    mb_ctx_t ctx;
    int rc = mb_ctx_init(&ctx, line);
    if (rc != MB_OK) {
        LOGE("Init failed: %s", mb_status_str(rc));
        return 1;
    }

    const char *cmd = NULL;
    uint8_t exc = 0;

    // Default timeouts/retries for industrial use (tune per link quality).
    const int timeout_ms = 1000;
    const int retries = 2;

    if (use_config_command) {
        rc = run_config_command(&ctx, slave, &file_cfg, &exc);
        goto out;
    }

    cmd = argv[i++];

    if (!strcmp(cmd, "read-holding")) {
        if (i + 1 >= argc) { usage(argv[0]); goto out; }
        bool ok1, ok2;
        uint16_t addr = (uint16_t)parse_long(argv[i++], 0, 65535, &ok1);
        uint16_t qty  = (uint16_t)parse_long(argv[i++], 1, 125, &ok2);
        if (!ok1 || !ok2) { LOGE("%s", "bad args"); goto out; }
        uint16_t regs[125];
        rc = mb_read_holding(&ctx, slave, addr, qty, regs, 125, timeout_ms, retries, &exc);
        if (rc == MB_OK) {
            for (uint16_t k = 0; k < qty; k++) printf("reg[%u]=%u\n", (unsigned)(addr + k), regs[k]);
        }
    } else if (!strcmp(cmd, "read-input")) {
        if (i + 1 >= argc) { usage(argv[0]); goto out; }
        bool ok1, ok2;
        uint16_t addr = (uint16_t)parse_long(argv[i++], 0, 65535, &ok1);
        uint16_t qty  = (uint16_t)parse_long(argv[i++], 1, 125, &ok2);
        if (!ok1 || !ok2) { LOGE("%s", "bad args"); goto out; }
        uint16_t regs[125];
        rc = mb_read_input(&ctx, slave, addr, qty, regs, 125, timeout_ms, retries, &exc);
        if (rc == MB_OK) {
            for (uint16_t k = 0; k < qty; k++) printf("in[%u]=%u\n", (unsigned)(addr + k), regs[k]);
        }
    } else if (!strcmp(cmd, "write-reg")) {
        if (i + 1 >= argc) { usage(argv[0]); goto out; }
        bool ok1, ok2;
        uint16_t addr = (uint16_t)parse_long(argv[i++], 0, 65535, &ok1);
        uint16_t val  = (uint16_t)parse_long(argv[i++], 0, 65535, &ok2);
        if (!ok1 || !ok2) { LOGE("%s", "bad args"); goto out; }
        rc = mb_write_single_reg(&ctx, slave, addr, val, timeout_ms, retries, &exc);
    } else if (!strcmp(cmd, "write-regs")) {
        if (i + 2 >= argc) { usage(argv[0]); goto out; }
        bool ok1, ok2;
        uint16_t addr = (uint16_t)parse_long(argv[i++], 0, 65535, &ok1);
        uint16_t n    = (uint16_t)parse_long(argv[i++], 1, (long)MB_MAX_WRITE_REGS, &ok2);
        if (!ok1 || !ok2) { LOGE("%s", "bad args"); goto out; }
        if (i + n > argc) { LOGE("need %u values", (unsigned)n); goto out; }
        uint16_t vals[MB_MAX_WRITE_REGS];
        for (uint16_t k = 0; k < n; k++) {
            bool ok;
            vals[k] = (uint16_t)parse_long(argv[i++], 0, 65535, &ok);
            if (!ok) { LOGE("%s", "bad value"); goto out; }
        }
        rc = mb_write_multi_regs(&ctx, slave, addr, vals, n, timeout_ms, retries, &exc);
    } else if (!strcmp(cmd, "read-coils")) {
        if (i + 1 >= argc) { usage(argv[0]); goto out; }
        bool ok1, ok2;
        uint16_t addr = (uint16_t)parse_long(argv[i++], 0, 65535, &ok1);
        uint16_t qty  = (uint16_t)parse_long(argv[i++], 1, (long)MB_MAX_READ_BITS, &ok2);
        if (!ok1 || !ok2) { LOGE("%s", "bad args"); goto out; }
        uint8_t bits[(MB_MAX_READ_BITS + 7u) / 8u];
        rc = mb_read_coils(&ctx, slave, addr, qty, bits, sizeof(bits), timeout_ms, retries, &exc);
        if (rc == MB_OK) {
            for (uint16_t k = 0; k < qty; k++) {
                uint8_t b = bits[k / 8] >> (k % 8);
                printf("coil[%u]=%u\n", (unsigned)(addr + k), (unsigned)(b & 1));
            }
        }
    } else if (!strcmp(cmd, "write-coil")) {
        if (i + 1 >= argc) { usage(argv[0]); goto out; }
        bool ok1, ok2;
        uint16_t addr = (uint16_t)parse_long(argv[i++], 0, 65535, &ok1);
        long on = parse_long(argv[i++], 0, 1, &ok2);
        if (!ok1 || !ok2) { LOGE("%s", "bad args"); goto out; }
        rc = mb_write_single_coil(&ctx, slave, addr, on ? true : false, timeout_ms, retries, &exc);
    } else if (!strcmp(cmd, "write-coils")) {
        if (i + 1 >= argc) { usage(argv[0]); goto out; }
        bool ok1;
        uint16_t addr = (uint16_t)parse_long(argv[i++], 0, 65535, &ok1);
        bool ok2;
        uint16_t n_bits = (uint16_t)parse_long(argv[i++], 1, (long)MB_MAX_WRITE_COILS, &ok2);
        if (!ok1 || !ok2) { LOGE("%s", "bad args"); goto out; }
        uint16_t bc = (uint16_t)((n_bits + 7u)/8u);
        if (i + bc > argc) { LOGE("need %u packed bytes", (unsigned)bc); goto out; }
        uint8_t packed[(MB_MAX_WRITE_COILS + 7u)/8u];
        memset(packed, 0, sizeof(packed));
        for (uint16_t k = 0; k < bc; k++) {
            bool ok;
            long v = parse_long(argv[i++], 0, 255, &ok);
            if (!ok) { LOGE("%s", "bad hexbyte"); goto out; }
            packed[k] = (uint8_t)v;
        }
        rc = mb_write_multi_coils(&ctx, slave, addr, packed, n_bits, timeout_ms, retries, &exc);
    } else if (!strcmp(cmd, "scan")) {
        bool ok1 = true;
        bool ok2 = true;
        bool ok3 = true;
        uint16_t start = 1;
        uint16_t end = 247;
        int scan_timeout_ms = 50;
        if (i < argc) start = (uint16_t)parse_long(argv[i++], 1, 247, &ok1);
        if (i < argc) end = (uint16_t)parse_long(argv[i++], 1, 247, &ok2);
        if (i < argc) scan_timeout_ms = (int)parse_long(argv[i++], 1, 60000, &ok3);
        if (i < argc || !ok1 || !ok2 || !ok3) { LOGE("%s", "bad args"); goto out; }
        rc = mb_scan_slaves(&ctx, (uint8_t)start, (uint8_t)end, scan_timeout_ms);
    } else if (!strcmp(cmd, "collect")) {
        collect_cfg_t cfg = {
            .out_dir = "./data",
            .device_id = "modbus_framer",
            .ctrl_path = "/tmp/modbus_framer_ctrl.json",
            .interval_ms = 1000,
            .timeout_ms = timeout_ms,
            .retries = retries,
            .max_file_bytes = 200u * 1024u,
            .max_total_bytes = 1024u * 1024u,
            .max_records_per_file = 20,
            .func_code = 4,
            .temp_addr = 0,
            .hum_addr = 1,
            .device_addr = 2,
            .temp_signed = false,
        };

        while (i < argc) {
            if (!strcmp(argv[i], "--interval-ms") && i + 1 < argc) {
                bool ok;
                cfg.interval_ms = (int)parse_long(argv[i + 1], 10, 600000, &ok);
                if (!ok) { LOGE("%s", "bad --interval-ms"); goto out; }
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--dir") && i + 1 < argc) { cfg.out_dir = argv[i + 1]; i += 2; continue; }
            if (!strcmp(argv[i], "--device-id") && i + 1 < argc) { cfg.device_id = argv[i + 1]; i += 2; continue; }
            if (!strcmp(argv[i], "--file-kb") && i + 1 < argc) {
                bool ok;
                long v = parse_long(argv[i + 1], 1, 10240, &ok);
                if (!ok) { LOGE("%s", "bad --file-kb"); goto out; }
                cfg.max_file_bytes = (size_t)v * 1024u;
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--records-per-file") && i + 1 < argc) {
                bool ok;
                cfg.max_records_per_file = (int)parse_long(argv[i + 1], 1, 100000, &ok);
                if (!ok) { LOGE("%s", "bad --records-per-file"); goto out; }
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--total-kb") && i + 1 < argc) {
                bool ok;
                long v = parse_long(argv[i + 1], 1, 10240, &ok);
                if (!ok) { LOGE("%s", "bad --total-kb"); goto out; }
                cfg.max_total_bytes = (size_t)v * 1024u;
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--func") && i + 1 < argc) {
                bool ok;
                cfg.func_code = (int)parse_long(argv[i + 1], 3, 4, &ok);
                if (!ok) { LOGE("%s", "bad --func (expect 3 or 4)"); goto out; }
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--temp-addr") && i + 1 < argc) {
                bool ok;
                cfg.temp_addr = (uint16_t)parse_long(argv[i + 1], 0, 65535, &ok);
                if (!ok) { LOGE("%s", "bad --temp-addr"); goto out; }
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--hum-addr") && i + 1 < argc) {
                bool ok;
                cfg.hum_addr = (uint16_t)parse_long(argv[i + 1], 0, 65535, &ok);
                if (!ok) { LOGE("%s", "bad --hum-addr"); goto out; }
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--device-addr") && i + 1 < argc) {
                bool ok;
                cfg.device_addr = (int)parse_long(argv[i + 1], -1, 65535, &ok);
                if (!ok) { LOGE("%s", "bad --device-addr"); goto out; }
                i += 2; continue;
            }
            if (!strcmp(argv[i], "--temp-signed")) {
                cfg.temp_signed = true;
                i += 1; continue;
            }
            LOGE("%s", "bad args");
            goto out;
        }

        if (cfg.max_total_bytes < cfg.max_file_bytes) cfg.max_total_bytes = cfg.max_file_bytes;
        rc = run_collect(&ctx, slave, &cfg);
    } else if (!strcmp(cmd, "stress")) {
        bool ok1 = true;
        bool ok2 = true;
        bool ok3 = true;
        bool ok4 = true;
        int duration_s = 0;
        int interval_ms = 20;
        long seed = 0;
        int report_s = 5;
        if (i < argc) duration_s = (int)parse_long(argv[i++], 0, 86400, &ok1);
        if (i < argc) interval_ms = (int)parse_long(argv[i++], 0, 10000, &ok2);
        if (i < argc) seed = parse_long(argv[i++], 0, LONG_MAX, &ok3);
        if (i < argc) report_s = (int)parse_long(argv[i++], 1, 3600, &ok4);
        if (i < argc || !ok1 || !ok2 || !ok3 || !ok4) { LOGE("%s", "bad args"); goto out; }
        rc = run_stress(&ctx, slave, duration_s, interval_ms, timeout_ms, retries, (uint64_t)seed, report_s);
    } else {
        usage(argv[0]);
        rc = MB_ERR_PARAM;
    }

out:
    if (rc == MB_ERR_EXCEPTION) {
        LOGW("Modbus exception: 0x%02X", exc);
    } else if (rc != MB_OK) {
        LOGE("Command failed: %s", mb_status_str(rc));
    }

    LOGI("stats: tx=%" PRIu64 " rx=%" PRIu64 " timeout=%" PRIu64 " crc=%" PRIu64 " proto=%" PRIu64,
         ctx.stat_tx, ctx.stat_rx, ctx.stat_timeouts, ctx.stat_crc_err, ctx.stat_proto_err);

    mb_ctx_shutdown(&ctx);
    return (rc == MB_OK) ? 0 : 1;
}
