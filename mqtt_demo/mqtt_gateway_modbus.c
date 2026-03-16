#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "mqtt_gateway_app.h"

static int write_file_atomic(const char *path, const char *buf, size_t len) {
    char tmp[300];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *fp = fopen(tmp, "wb");
    if (!fp) return -1;
    size_t wr = fwrite(buf, 1, len, fp);
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
    if (wr != len) {
        remove(tmp);
        return -1;
    }
    if (rename(tmp, path) != 0) {
        remove(tmp);
        return -1;
    }
    return 0;
}

static void put_object_item(cJSON *dst, const char *key, cJSON *item) {
    if (!dst || !key || !item) return;
    if (cJSON_GetObjectItemCaseSensitive(dst, key)) {
        cJSON_ReplaceItemInObjectCaseSensitive(dst, key, item);
    } else {
        cJSON_AddItemToObject(dst, key, item);
    }
}

static void copy_string_if_present(cJSON *dst, cJSON *src, const char *key, const char *dst_key, bool *copied) {
    cJSON *item = cJSON_GetObjectItemCaseSensitive(src, key);
    if (!cJSON_IsString(item) || !item->valuestring || !item->valuestring[0]) return;
    put_object_item(dst, dst_key, cJSON_CreateString(item->valuestring));
    *copied = true;
}

static void copy_bool_if_present(cJSON *dst, cJSON *src, const char *key, const char *dst_key, bool *copied) {
    cJSON *item = cJSON_GetObjectItemCaseSensitive(src, key);
    if (!item || (!cJSON_IsBool(item) && !cJSON_IsNumber(item))) return;
    put_object_item(dst, dst_key, cJSON_CreateBool(cJSON_IsTrue(item) || (cJSON_IsNumber(item) && item->valueint != 0)));
    *copied = true;
}

static void copy_number_if_present(cJSON *dst,
                                   cJSON *src,
                                   const char *key,
                                   const char *dst_key,
                                   int min_value,
                                   int max_value,
                                   bool *copied) {
    cJSON *item = cJSON_GetObjectItemCaseSensitive(src, key);
    if (!cJSON_IsNumber(item)) return;
    int value = item->valueint;
    if (value < min_value || value > max_value) return;
    put_object_item(dst, dst_key, cJSON_CreateNumber(value));
    *copied = true;
}

static void copy_number_alias_if_present(cJSON *dst,
                                         cJSON *src,
                                         const char *dst_key,
                                         const char *key1,
                                         const char *key2,
                                         int min_value,
                                         int max_value,
                                         bool *copied) {
    copy_number_if_present(dst, src, key1, dst_key, min_value, max_value, copied);
    copy_number_if_present(dst, src, key2, dst_key, min_value, max_value, copied);
}

static void merge_ctrl_fields(cJSON *dst, cJSON *src, bool *copied) {
    if (!dst || !src || !copied || !cJSON_IsObject(src)) return;

    copy_number_alias_if_present(dst, src, "interval_ms",
                                 "interval_ms", "telemetry_interval_ms",
                                 1, 600000, copied);
    copy_number_alias_if_present(dst, src, "records_per_file",
                                 "records_per_file", "max_records_per_file",
                                 1, 100000, copied);
    copy_number_alias_if_present(dst, src, "func",
                                 "func", "func_code",
                                 3, 4, copied);
    copy_number_if_present(dst, src, "temp_addr", "temp_addr", 0, 65535, copied);
    copy_number_if_present(dst, src, "hum_addr", "hum_addr", 0, 65535, copied);
    copy_number_if_present(dst, src, "device_addr", "device_addr", -1, 65535, copied);
    copy_number_if_present(dst, src, "slave", "slave", 1, 247, copied);
    copy_number_if_present(dst, src, "baud", "baud", 1, 4000000, copied);
    copy_number_if_present(dst, src, "data_bits", "data_bits", 5, 8, copied);
    copy_number_if_present(dst, src, "stop_bits", "stop_bits", 1, 2, copied);
    copy_number_if_present(dst, src, "file_kb", "file_kb", 1, 10240, copied);
    copy_number_if_present(dst, src, "total_kb", "total_kb", 1, 10240, copied);
    copy_number_if_present(dst, src, "timeout_ms", "timeout_ms", 1, 60000, copied);
    copy_number_if_present(dst, src, "retries", "retries", 0, 100, copied);

    copy_string_if_present(dst, src, "device_id", "device_id", copied);
    copy_string_if_present(dst, src, "dir", "dir", copied);
    copy_string_if_present(dst, src, "data_dir", "dir", copied);
    copy_string_if_present(dst, src, "dev", "dev", copied);
    copy_string_if_present(dst, src, "port", "dev", copied);
    copy_string_if_present(dst, src, "parity", "parity", copied);

    copy_bool_if_present(dst, src, "temp_signed", "temp_signed", copied);
    copy_bool_if_present(dst, src, "telemetry_enable", "telemetry_enable", copied);
    copy_bool_if_present(dst, src, "rs485", "rs485", copied);
    copy_bool_if_present(dst, src, "rs485_enable", "rs485", copied);
    copy_bool_if_present(dst, src, "debug", "debug", copied);
}

int save_ctrl_config_file(const app_t *app, cJSON *root) {
    if (!app || !root || !cJSON_IsObject(root)) return -1;

    cJSON *ctrl = cJSON_CreateObject();
    if (!ctrl) return -1;

    bool copied = false;
    merge_ctrl_fields(ctrl, root, &copied);

    cJSON *modbus = cJSON_GetObjectItemCaseSensitive(root, "modbus");
    merge_ctrl_fields(ctrl, modbus, &copied);
    if (cJSON_IsObject(modbus)) {
        cJSON *collect = cJSON_GetObjectItemCaseSensitive(modbus, "collect");
        merge_ctrl_fields(ctrl, collect, &copied);
    }

    cJSON *collect = cJSON_GetObjectItemCaseSensitive(root, "collect");
    merge_ctrl_fields(ctrl, collect, &copied);

    if (!copied) {
        cJSON_Delete(ctrl);
        return 0;
    }

    char *json = cJSON_PrintUnformatted(ctrl);
    cJSON_Delete(ctrl);
    if (!json) return -1;

    int rc = write_file_atomic(app->cfg.ctrl_path, json, strlen(json));
    if (rc != 0) {
        LOGW("save ctrl config failed: %s (%s)", app->cfg.ctrl_path, strerror(errno));
        cJSON_free(json);
        return -1;
    }

    cJSON_free(json);
    return 1;
}
