#define _POSIX_C_SOURCE 200809L
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "mqtt_gateway_app.h"

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

/*
 * File tailer thread:
 * - Watches data_dir for files (default: /userdata/modbus_data)
 * - Publishes completed files (not the newest/active file) to MQTT
 * - Records uploaded files into "mqtt_upload.log" to avoid re-upload
 *
 * Notes:
 * - Payload is sent as raw text/JSON/XML based on tail_format.
 * - For JSONL, each line is sent as one MQTT message.
 * - A file is considered complete when a newer file exists (or idle long enough if it's the only file).
 */

#ifndef TAILER_LINE_BUFFER_SHRINK_THRESHOLD
#define TAILER_LINE_BUFFER_SHRINK_THRESHOLD (128u * 1024u)
#endif

static const char *k_done_file_name = "mqtt_upload.log";
static const char *k_legacy_done_file_name = ".sent_files";

typedef struct {
    char **items;
    size_t count;
    size_t cap;
} str_list_t;

static void str_list_free(str_list_t *list) {
    if (!list) return;
    for (size_t i = 0; i < list->count; i++) {
        free(list->items[i]);
    }
    free(list->items);
    list->items = NULL;
    list->count = 0;
    list->cap = 0;
}

static bool str_list_contains(const str_list_t *list, const char *s) {
    if (!list || !s) return false;
    for (size_t i = 0; i < list->count; i++) {
        if (strcmp(list->items[i], s) == 0) return true;
    }
    return false;
}

static int str_list_add(str_list_t *list, const char *s) {
    if (!list || !s || !*s) return -1;
    if (str_list_contains(list, s)) return 0;
    if (list->count == list->cap) {
        size_t ncap = (list->cap == 0) ? 16 : list->cap * 2;
        char **tmp = (char **)realloc(list->items, ncap * sizeof(char *));
        if (!tmp) return -1;
        list->items = tmp;
        list->cap = ncap;
    }
    list->items[list->count] = strdup(s);
    if (!list->items[list->count]) return -1;
    list->count++;
    return 0;
}

static void trim_newline(char *s) {
    if (!s) return;
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) {
        s[n - 1] = '\0';
        n--;
    }
}

static void join_path(char *dst, size_t cap, const char *dir, const char *name) {
    size_t len = strlen(dir);
    if (len > 0 && dir[len - 1] == '/') {
        snprintf(dst, cap, "%s%s", dir, name);
    } else {
        snprintf(dst, cap, "%s/%s", dir, name);
    }
}

static bool has_suffix(const char *s, const char *suf) {
    size_t ls = strlen(s);
    size_t lf = strlen(suf);
    if (ls < lf) return false;
    return strcmp(s + (ls - lf), suf) == 0;
}

static bool is_stable_mtime(time_t now, time_t mtime, int idle_ms) {
    if (idle_ms <= 0) return true;
    if (now <= mtime) return false;
    return ((now - mtime) * 1000) >= idle_ms;
}

static void normalize_ext(char *dst, size_t cap, const char *ext) {
    if (!dst || cap == 0) return;
    dst[0] = '\0';
    if (!ext || !*ext) return;
    if (ext[0] == '.') {
        snprintf(dst, cap, "%s", ext);
    } else {
        snprintf(dst, cap, ".%s", ext);
    }
}

static bool matches_format(const char *name, tail_format_t fmt, const char *ext_override) {
    if (!name || !*name) return false;
    if (ext_override && *ext_override) return has_suffix(name, ext_override);

    if (fmt == TAIL_FORMAT_TEXT) {
        return has_suffix(name, ".txt") || has_suffix(name, ".log");
    }
    if (fmt == TAIL_FORMAT_JSON) {
        return has_suffix(name, ".jsonl") || has_suffix(name, ".json");
    }
    if (fmt == TAIL_FORMAT_XML) {
        return has_suffix(name, ".xml");
    }
    // AUTO: accept common extensions.
    return has_suffix(name, ".jsonl") || has_suffix(name, ".json") ||
           has_suffix(name, ".xml") || has_suffix(name, ".txt") ||
           has_suffix(name, ".log");
}

static int load_done_list(const char *path, str_list_t *done) {
    FILE *fp = fopen(path, "r");
    if (!fp) return 0;
    char line[256];
    while (fgets(line, sizeof(line), fp) != NULL) {
        trim_newline(line);
        if (line[0] == '\0') continue;
        if (str_list_add(done, line) != 0) {
            fclose(fp);
            return -1;
        }
    }
    fclose(fp);
    return 0;
}

static int append_done_list(const char *path, const char *name) {
    FILE *fp = fopen(path, "a");
    if (!fp) return -1;
    fprintf(fp, "%s\n", name);
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
    return 0;
}

typedef struct {
    char name[PATH_MAX];
    char path[PATH_MAX];
    FILE *fp;
    off_t offset;
    off_t last_size;
    time_t last_mtime;
    uint64_t last_change_ms;
    bool stable;
} tail_file_t;

static void tail_file_close(tail_file_t *tf) {
    if (tf->fp) fclose(tf->fp);
    memset(tf, 0, sizeof(*tf));
}

static int tail_file_open(tail_file_t *tf, const char *dir, const char *name) {
    memset(tf, 0, sizeof(*tf));
    snprintf(tf->name, sizeof(tf->name), "%s", name);
    join_path(tf->path, sizeof(tf->path), dir, name);
    tf->fp = fopen(tf->path, "rb");
    if (!tf->fp) return -1;
    tf->offset = 0;
    struct stat st;
    if (stat(tf->path, &st) == 0) {
        tf->last_size = st.st_size;
        tf->last_mtime = st.st_mtime;
        tf->last_change_ms = monotonic_ms();
    }
    return 0;
}

static int collect_files(const char *dir,
                         const char *ext,
                         tail_format_t fmt,
                         const str_list_t *done,
                         const char *done_file_name,
                         time_t now,
                         int idle_ms,
                         str_list_t *out) {
    DIR *d = opendir(dir);
    if (!d) return -1;
    struct dirent *de;
    str_list_t all = {0};
    size_t total_files = 0;
    time_t newest_mtime = 0;
    char newest_name[PATH_MAX];
    newest_name[0] = '\0';
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        if (done_file_name && strcmp(de->d_name, done_file_name) == 0) continue;
        if (!matches_format(de->d_name, fmt, ext)) continue;

        char path[PATH_MAX];
        join_path(path, sizeof(path), dir, de->d_name);
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) continue;
        if (str_list_add(&all, de->d_name) != 0) {
            closedir(d);
            str_list_free(&all);
            return -1;
        }
        total_files++;
        if (newest_name[0] == '\0' ||
            st.st_mtime > newest_mtime ||
            (st.st_mtime == newest_mtime && strcmp(de->d_name, newest_name) > 0)) {
            newest_mtime = st.st_mtime;
            snprintf(newest_name, sizeof(newest_name), "%s", de->d_name);
        }
    }
    closedir(d);
    if (total_files == 0) {
        str_list_free(&all);
        return 0;
    }

    for (size_t i = 0; i < all.count; i++) {
        const char *name = all.items[i];
        if (str_list_contains(done, name)) continue;
        if (total_files > 1 && newest_name[0] && strcmp(name, newest_name) == 0) continue;

        char path[PATH_MAX];
        join_path(path, sizeof(path), dir, name);
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) continue;
        if (total_files == 1 && !is_stable_mtime(now, st.st_mtime, idle_ms)) continue;

        if (str_list_add(out, name) != 0) {
            str_list_free(&all);
            return -1;
        }
    }
    str_list_free(&all);
    return 0;
}

static int cmp_str_ptr(const void *a, const void *b) {
    const char *sa = *(const char *const *)a;
    const char *sb = *(const char *const *)b;
    return strcmp(sb, sa);
}

static void release_large_line_buffer(char **line, size_t *line_cap) {
    if (!line || !line_cap || !*line) return;
    if (*line_cap <= TAILER_LINE_BUFFER_SHRINK_THRESHOLD) return;
    free(*line);
    *line = NULL;
    *line_cap = 0;
}

static void upsert_json_string(cJSON *root, const char *key, const char *value) {
    if (!root || !cJSON_IsObject(root) || !key || !*key || !value || !*value) return;
    cJSON_DeleteItemFromObjectCaseSensitive(root, key);
    cJSON_AddStringToObject(root, key, value);
}

static char *build_publish_payload(app_t *app, const char *line) {
    if (!app || !line || !*line) return NULL;

    cJSON *root = cJSON_Parse(line);
    if (!root) {
        return strdup(line);
    }
    if (!cJSON_IsObject(root)) {
        char *raw_payload = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        return raw_payload ? raw_payload : strdup(line);
    }

    cJSON *item_device_id = cJSON_GetObjectItemCaseSensitive(root, "device_id");
    if (cJSON_IsString(item_device_id) && item_device_id->valuestring && item_device_id->valuestring[0] &&
        strcmp(item_device_id->valuestring, app->cfg.device_id) != 0) {
        cJSON *existing_source = cJSON_GetObjectItemCaseSensitive(root, "source_device_id");
        if (!cJSON_IsString(existing_source) || !existing_source->valuestring || !existing_source->valuestring[0]) {
            cJSON_AddStringToObject(root, "source_device_id", item_device_id->valuestring);
        }
    }

    cJSON *item_cpuinfo = cJSON_GetObjectItemCaseSensitive(root, "cpuinfo");
    if (cJSON_IsString(item_cpuinfo) && item_cpuinfo->valuestring && item_cpuinfo->valuestring[0] &&
        strcmp(item_cpuinfo->valuestring, app->cfg.device_id) != 0) {
        cJSON *existing_source = cJSON_GetObjectItemCaseSensitive(root, "source_cpuinfo");
        if (!cJSON_IsString(existing_source) || !existing_source->valuestring || !existing_source->valuestring[0]) {
            cJSON_AddStringToObject(root, "source_cpuinfo", item_cpuinfo->valuestring);
        }
    }

    upsert_json_string(root, "cpuinfo", app->cfg.device_id);
    upsert_json_string(root, "device_id", app->cfg.device_id);
    upsert_json_string(root, "client_id", app->cfg.client_id);
    upsert_json_string(root, "gateway_id", app->cfg.device_id);
    upsert_json_string(root, "data_class", "modbus");
    upsert_json_string(root, "thread_kind", "modbus_collector");

    cJSON *source = cJSON_GetObjectItemCaseSensitive(root, "source");
    if (!cJSON_IsString(source) || !source->valuestring || !source->valuestring[0]) {
        cJSON_AddStringToObject(root, "source", "modbus_tailer");
    }

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) {
        return strdup(line);
    }
    return payload;
}

void *file_tailer_main(void *arg) {
    app_t *app = (app_t *)arg;

    // Runtime tailer settings are fixed after startup; take one snapshot here.
    app_cfg_t cfg = app->cfg;

    if (!cfg.tail_enable) {
        while (g_running) sleep_ms(500);
        return NULL;
    }

    if (!cfg.data_dir[0]) {
        LOGW("%s", "tailer disabled: data_dir is empty");
        while (g_running) sleep_ms(500);
        return NULL;
    }

    // Normalize extension override (if provided).
    char ext_buf[16];
    normalize_ext(ext_buf, sizeof(ext_buf), cfg.data_ext);

    char done_path[PATH_MAX];
    join_path(done_path, sizeof(done_path), cfg.data_dir, k_done_file_name);
    char legacy_done_path[PATH_MAX];
    join_path(legacy_done_path, sizeof(legacy_done_path), cfg.data_dir, k_legacy_done_file_name);

    str_list_t done = {0};
    (void)load_done_list(done_path, &done);
    (void)load_done_list(legacy_done_path, &done);

    tail_file_t cur;
    memset(&cur, 0, sizeof(cur));

    char *line = NULL;
    size_t line_cap = 0;
    uint64_t last_dir_warn = 0;
    size_t publish_count = 0;

    while (g_running) {
        if (!cur.fp) {
            str_list_t files = {0};
            time_t now_s = time(NULL);
            if (collect_files(cfg.data_dir, ext_buf, cfg.tail_format, &done,
                              k_done_file_name, now_s, cfg.tail_idle_ms, &files) != 0) {
                uint64_t now = monotonic_ms();
                if (now - last_dir_warn > 5000) {
                    LOGW("tailer cannot open dir: %s", cfg.data_dir);
                    last_dir_warn = now;
                }
                release_large_line_buffer(&line, &line_cap);
                str_list_free(&files);
                sleep_ms(cfg.tail_poll_ms);
                continue;
            }
            if (files.count == 0) {
                uint64_t now = monotonic_ms();
                if (now - last_dir_warn > 5000) {
                    LOGD("tailer idle: no stable files in %s", cfg.data_dir);
                    last_dir_warn = now;
                }
                release_large_line_buffer(&line, &line_cap);
                str_list_free(&files);
                sleep_ms(cfg.tail_poll_ms);
                continue;
            }
            qsort(files.items, files.count, sizeof(char *), cmp_str_ptr);
            if (tail_file_open(&cur, cfg.data_dir, files.items[0]) != 0) {
                LOGD("tailer open failed: %s", files.items[0]);
                release_large_line_buffer(&line, &line_cap);
                str_list_free(&files);
                sleep_ms(cfg.tail_poll_ms);
                continue;
            }
            cur.stable = true;
            publish_count = 0;
            LOGD("tailer opened: %s", cur.name);
            str_list_free(&files);
        }

        struct stat st;
        if (stat(cur.path, &st) != 0) {
            LOGW("tailer stat failed: %s", cur.path);
            tail_file_close(&cur);
            release_large_line_buffer(&line, &line_cap);
            sleep_ms(cfg.tail_poll_ms);
            continue;
        }

        // Detect file changes to reset idle timer.
        if (st.st_size != cur.last_size || st.st_mtime != cur.last_mtime) {
            cur.last_size = st.st_size;
            cur.last_mtime = st.st_mtime;
            cur.last_change_ms = monotonic_ms();
            cur.stable = false;
        }

        // Handle file truncation or rotation (should not happen for timestamped names).
        if (cur.offset > st.st_size) {
            LOGW("tailer file truncated: %s", cur.path);
            cur.offset = 0;
        }

        if (fseeko(cur.fp, cur.offset, SEEK_SET) != 0) {
            LOGW("tailer seek failed: %s", cur.path);
            tail_file_close(&cur);
            release_large_line_buffer(&line, &line_cap);
            sleep_ms(cfg.tail_poll_ms);
            continue;
        }

        bool publish_blocked = false;
        int last_pub_rc = MQTTCLIENT_SUCCESS;
        while (g_running) {
            off_t line_start = ftello(cur.fp);
            errno = 0;
            ssize_t n = getline(&line, &line_cap, cur.fp);
            if (n < 0) {
                clearerr(cur.fp);
                break;
            }
            if (n == 0) continue;

            // If the line has no newline, it is still being written; retry later.
            if (line[n - 1] != '\n') {
                fseeko(cur.fp, line_start, SEEK_SET);
                break;
            }

            size_t len = (size_t)n;
            while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
                line[len - 1] = '\0';
                len--;
            }
            if (len == 0) continue;

            char *publish_payload = build_publish_payload(app, line);
            if (!publish_payload) {
                fseeko(cur.fp, line_start, SEEK_SET);
                publish_blocked = true;
                last_pub_rc = MQTTCLIENT_FAILURE;
                break;
            }

            int rc = mqtt_publish_json(app, app->topic_data_up, 1, 0, publish_payload);
            free(publish_payload);
            if (rc != MQTTCLIENT_SUCCESS) {
                // MQTT not ready; rewind and retry later to avoid data loss.
                fseeko(cur.fp, line_start, SEEK_SET);
                publish_blocked = true;
                last_pub_rc = rc;
                break;
            }

            cur.offset = ftello(cur.fp);
            publish_count++;
        }

        if (publish_blocked) {
            LOGD("tailer publish blocked (rc=%d), will retry", last_pub_rc);
            release_large_line_buffer(&line, &line_cap);
            sleep_ms(cfg.tail_poll_ms);
            continue;
        }

        // If we are at EOF and idle long enough, mark file as sent.
        uint64_t now = monotonic_ms();
        if (cur.offset >= st.st_size &&
            (cur.stable || (now - cur.last_change_ms) >= (uint64_t)cfg.tail_idle_ms)) {
            if (!str_list_contains(&done, cur.name)) {
                if (append_done_list(done_path, cur.name) == 0) {
                    (void)str_list_add(&done, cur.name);
                    LOGD("tailer done: %s (lines=%zu)", cur.name, publish_count);
                } else {
                    LOGW("tailer done-list write failed: %s", cur.name);
                }
            }
            tail_file_close(&cur);
            release_large_line_buffer(&line, &line_cap);
        }

        sleep_ms(cfg.tail_poll_ms);
    }

    if (line) free(line);
    tail_file_close(&cur);
    str_list_free(&done);
    return NULL;
}
