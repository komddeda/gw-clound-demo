#include <curl/curl.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <strings.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "cJSON.h"
#include "../demo_config.h"

#ifndef LOG_LEVEL
#define LOG_LEVEL 3 // 0=quiet, 1=error, 2=warn, 3=info, 4=debug
#endif
#ifndef LOG_MAX_BYTES
#define LOG_MAX_BYTES (1u * 1024u * 1024u)
#endif
static FILE *g_log_fp = NULL;
static char g_log_path[PATH_MAX];
static void log_rotate_if_needed(void) {
	if (!g_log_fp || g_log_path[0] == '\0') return;
	int fd = fileno(g_log_fp);
	struct stat st;
	if (fstat(fd, &st) != 0) return;
	if ((size_t)st.st_size <= LOG_MAX_BYTES) return;
	fclose(g_log_fp);
	g_log_fp = NULL;
	(void)remove(g_log_path);
	g_log_fp = fopen(g_log_path, "a");
	if (g_log_fp) {
		setvbuf(g_log_fp, NULL, _IOLBF, 0);
	} else {
		fprintf(stderr, "[W] log reopen failed: %s (%s)\n", g_log_path, strerror(errno));
	}
}
static void log_write(const char *level, const char *fmt, ...) {
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
	if (g_log_fp) {
		fprintf(g_log_fp, "[%s] %s %s\n", level, ts, msg);
		fflush(g_log_fp);
		log_rotate_if_needed();
	}
}
#define LOGE(fmt, ...) do { if (LOG_LEVEL >= 1) log_write("E", fmt, ##__VA_ARGS__); } while (0)
#define LOGW(fmt, ...) do { if (LOG_LEVEL >= 2) log_write("W", fmt, ##__VA_ARGS__); } while (0)
#define LOGI(fmt, ...) do { if (LOG_LEVEL >= 3) log_write("I", fmt, ##__VA_ARGS__); } while (0)
#define LOGD(fmt, ...) do { if (LOG_LEVEL >= 4) log_write("D", fmt, ##__VA_ARGS__); } while (0)

#ifndef MAX_FILE_BYTES
#define MAX_FILE_BYTES (10u * 1024u * 1024u)
#endif
#ifndef MAX_TOTAL_BYTES
#define MAX_TOTAL_BYTES (4u * 1024u * 1024u)
#endif
#ifndef STABLE_AGE_S
#define STABLE_AGE_S 3
#endif

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
	const char *dir;
	const char *url;
	const char *device_id;
	const char *cpuinfo;
	const char *bad_dir;
	const char *log_path;
	int interval_s;
	int http_timeout_ms;
	int http_retries;
	int http_backoff_ms;
	int stable_age_s;
	size_t max_file_bytes;
	size_t max_total_bytes;
	bool http_insecure;
	bool test_mode;
	int test_interval_s;
	test_value_type_t test_value_type;
	test_pattern_t test_pattern;
	double test_step;
	char bad_dir_buf[PATH_MAX];
	char log_path_buf[PATH_MAX];
	char cpuinfo_buf[128];
	char dir_buf[PATH_MAX];
	char url_buf[512];
	char device_id_buf[128];
	char test_value[256];
} cfg_t;

typedef struct {
	char path[PATH_MAX];
	time_t mtime;
	size_t size;
} file_item_t;

typedef struct {
	double numeric_value;
	bool numeric_ready;
} test_value_state_t;

static void cfg_set_string(char *buf, size_t cap, const char **field, const char *value) {
	if (!buf || cap == 0 || !field || !value) return;
	snprintf(buf, cap, "%s", value);
	*field = buf;
}

static void cfg_defaults(cfg_t *cfg) {
	memset(cfg, 0, sizeof(*cfg));
	cfg_set_string(cfg->dir_buf, sizeof(cfg->dir_buf), &cfg->dir, "./data");
	cfg_set_string(cfg->url_buf, sizeof(cfg->url_buf), &cfg->url, "http://47.100.112.208:8080/ingest");
	cfg_set_string(cfg->device_id_buf, sizeof(cfg->device_id_buf), &cfg->device_id, "http_uploader");
	cfg->cpuinfo = NULL;
	cfg->bad_dir = NULL;
	cfg->log_path = NULL;
	cfg->interval_s = 5;
	cfg->http_timeout_ms = 5000;
	cfg->http_retries = 3;
	cfg->http_backoff_ms = 500;
	cfg->stable_age_s = STABLE_AGE_S;
	cfg->max_file_bytes = MAX_FILE_BYTES;
	cfg->max_total_bytes = MAX_TOTAL_BYTES;
	cfg->http_insecure = false;
	cfg->test_mode = false;
	cfg->test_interval_s = 1;
	cfg->test_value_type = TEST_VALUE_INT;
	cfg->test_pattern = TEST_PATTERN_FIXED;
	cfg->test_step = 1.0;
	snprintf(cfg->test_value, sizeof(cfg->test_value), "%s", "0");
}

static const char *k_done_file_name = ".sent_files";

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

typedef enum {
	UPLOAD_OK = 0,
	UPLOAD_SKIP,
	UPLOAD_RETRY,
	UPLOAD_BAD,
} upload_result_t;

static upload_result_t http_post_with_retries(const cfg_t *cfg, const char *payload, size_t len, long *out_code);

static volatile sig_atomic_t g_running = 1;
static void on_signal(int sig) {
	(void)sig;
	g_running = 0;
}

static void usage(const char *argv0) {
	fprintf(stderr,
			"Usage: %s [--config PATH] --dir PATH --url URL [--interval-s N] [--http-timeout-ms N] [--http-retries N]\n"
			"       [--device-id STR] [--max-file-kb N] [--max-total-kb N] [--stable-age-s N]\n"
			"       [--bad-dir PATH] [--log PATH] [--http-insecure] [--http-backoff-ms N]\n"
			"       [--test-mode] [--test-type int|float|bool|string|json]\n"
			"       [--test-pattern fixed|inc|dec] [--test-value VALUE] [--test-step N]\n"
			"       [--test-interval-s N]\n",
			argv0);
}

static test_value_type_t parse_test_value_type(const char *s) {
	if (!s || !*s) return TEST_VALUE_INT;
	if (!strcasecmp(s, "int") || !strcasecmp(s, "integer")) return TEST_VALUE_INT;
	if (!strcasecmp(s, "float") || !strcasecmp(s, "double")) return TEST_VALUE_FLOAT;
	if (!strcasecmp(s, "bool") || !strcasecmp(s, "boolean")) return TEST_VALUE_BOOL;
	if (!strcasecmp(s, "string") || !strcasecmp(s, "text")) return TEST_VALUE_STRING;
	if (!strcasecmp(s, "json")) return TEST_VALUE_JSON;
	return TEST_VALUE_INT;
}

static test_pattern_t parse_test_pattern(const char *s) {
	if (!s || !*s) return TEST_PATTERN_FIXED;
	if (!strcasecmp(s, "fixed")) return TEST_PATTERN_FIXED;
	if (!strcasecmp(s, "inc") || !strcasecmp(s, "increase")) return TEST_PATTERN_INC;
	if (!strcasecmp(s, "dec") || !strcasecmp(s, "decrease")) return TEST_PATTERN_DEC;
	return TEST_PATTERN_FIXED;
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

static bool is_http_common_section(const char *section) {
	return !section || !section[0] ||
		demo_config_name_eq(section, "http") ||
		demo_config_name_eq(section, "common");
}

static int http_apply_config_entry(const char *section,
								   const char *key,
								   const char *value,
								   void *user,
								   char *errbuf,
								   size_t errcap) {
	cfg_t *cfg = (cfg_t *)user;
	long lv = 0;
	double dv = 0.0;
	bool bv = false;

	if (!cfg || !key || !value) return -1;

	if (is_http_common_section(section)) {
		if (demo_config_name_eq(key, "dir")) {
			cfg_set_string(cfg->dir_buf, sizeof(cfg->dir_buf), &cfg->dir, value);
			return 0;
		}
		if (demo_config_name_eq(key, "url")) {
			cfg_set_string(cfg->url_buf, sizeof(cfg->url_buf), &cfg->url, value);
			return 0;
		}
		if (demo_config_name_eq(key, "device_id")) {
			cfg_set_string(cfg->device_id_buf, sizeof(cfg->device_id_buf), &cfg->device_id, value);
			return 0;
		}
		if (demo_config_name_eq(key, "interval_s")) {
			if (!demo_config_parse_long(value, 1, 86400, &lv)) goto bad_value;
			cfg->interval_s = (int)lv;
			return 0;
		}
		if (demo_config_name_eq(key, "http_timeout_ms") || demo_config_name_eq(key, "timeout_ms")) {
			if (!demo_config_parse_long(value, 100, 600000, &lv)) goto bad_value;
			cfg->http_timeout_ms = (int)lv;
			return 0;
		}
		if (demo_config_name_eq(key, "http_retries") || demo_config_name_eq(key, "retries")) {
			if (!demo_config_parse_long(value, 0, 100, &lv)) goto bad_value;
			cfg->http_retries = (int)lv;
			return 0;
		}
		if (demo_config_name_eq(key, "http_backoff_ms") || demo_config_name_eq(key, "backoff_ms")) {
			if (!demo_config_parse_long(value, 0, 600000, &lv)) goto bad_value;
			cfg->http_backoff_ms = (int)lv;
			return 0;
		}
		if (demo_config_name_eq(key, "max_file_kb")) {
			if (!demo_config_parse_long(value, 1, 102400, &lv)) goto bad_value;
			cfg->max_file_bytes = (size_t)lv * 1024u;
			return 0;
		}
		if (demo_config_name_eq(key, "max_total_kb")) {
			if (!demo_config_parse_long(value, 1, 102400, &lv)) goto bad_value;
			cfg->max_total_bytes = (size_t)lv * 1024u;
			return 0;
		}
		if (demo_config_name_eq(key, "stable_age_s")) {
			if (!demo_config_parse_long(value, 0, 86400, &lv)) goto bad_value;
			cfg->stable_age_s = (int)lv;
			return 0;
		}
		if (demo_config_name_eq(key, "bad_dir")) {
			snprintf(cfg->bad_dir_buf, sizeof(cfg->bad_dir_buf), "%s", value);
			cfg->bad_dir = cfg->bad_dir_buf;
			return 0;
		}
		if (demo_config_name_eq(key, "log") || demo_config_name_eq(key, "log_path")) {
			snprintf(cfg->log_path_buf, sizeof(cfg->log_path_buf), "%s", value);
			cfg->log_path = cfg->log_path_buf;
			return 0;
		}
		if (demo_config_name_eq(key, "http_insecure")) {
			if (!parse_cfg_bool01(value, &bv)) goto bad_value;
			cfg->http_insecure = bv;
			return 0;
		}
		if (demo_config_name_eq(key, "test_mode")) {
			if (!parse_cfg_bool01(value, &bv)) goto bad_value;
			cfg->test_mode = bv;
			return 0;
		}
	}

	if (demo_config_name_eq(section, "test")) {
		if (demo_config_name_eq(key, "enable") || demo_config_name_eq(key, "test_mode")) {
			if (!parse_cfg_bool01(value, &bv)) goto bad_value;
			cfg->test_mode = bv;
			return 0;
		}
		if (demo_config_name_eq(key, "interval_s") || demo_config_name_eq(key, "test_interval_s")) {
			if (!demo_config_parse_long(value, 1, 86400, &lv)) goto bad_value;
			cfg->test_interval_s = (int)lv;
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

static const char *http_find_config_path(int argc, char **argv) {
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

static bool http_is_config_arg(const char *arg) {
	return !strcmp(arg, "-c") || !strcmp(arg, "--config") || !strncmp(arg, "--config=", 9);
}

static bool has_suffix(const char *s, const char *suf) {
	size_t ls = strlen(s);
	size_t lf = strlen(suf);
	if (ls < lf) return false;
	return strcmp(s + (ls - lf), suf) == 0;
}

static bool has_suffix_ci(const char *s, const char *suf) {
	size_t ls = strlen(s);
	size_t lf = strlen(suf);
	if (ls < lf) return false;
	const char *tail = s + (ls - lf);
	for (size_t i = 0; i < lf; i++) {
		char a = tail[i];
		char b = suf[i];
		if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
		if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
		if (a != b) return false;
	}
	return true;
}

static bool read_cpuinfo_serial(char *out, size_t cap) {
	if (!out || cap == 0) return false;
	out[0] = '\0';
	FILE *fp = fopen("/proc/cpuinfo", "r");
	if (!fp) return false;
	char line[256];
	while (fgets(line, sizeof(line), fp) != NULL) {
		const char *p = line;
		while (*p == ' ' || *p == '\t') p++;
		if (strncmp(p, "Serial", 6) != 0) continue;
		const char *colon = strchr(p, ':');
		if (!colon) continue;
		colon++;
		while (*colon == ' ' || *colon == '\t') colon++;
		size_t n = strcspn(colon, "\r\n");
		if (n == 0) continue;
		if (n >= cap) n = cap - 1;
		memcpy(out, colon, n);
		out[n] = '\0';
		fclose(fp);
		return true;
	}
	fclose(fp);
	return false;
}

static void format_iso8601_utc(char *out, size_t cap) {
	if (!out || cap == 0) return;
	time_t now = time(NULL);
	struct tm tm_utc;
	gmtime_r(&now, &tm_utc);
	strftime(out, cap, "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
}

static long long round_to_long_long(double value) {
	return (long long)(value >= 0.0 ? value + 0.5 : value - 0.5);
}

static bool parse_bool_text(const char *text, bool default_value) {
	if (!text || !*text) return default_value;
	if (!strcasecmp(text, "1") || !strcasecmp(text, "true") ||
		!strcasecmp(text, "yes") || !strcasecmp(text, "on")) {
		return true;
	}
	if (!strcasecmp(text, "0") || !strcasecmp(text, "false") ||
		!strcasecmp(text, "no") || !strcasecmp(text, "off")) {
		return false;
	}
	return default_value;
}

static double normalize_test_step(double step) {
	if (step == 0.0) return 1.0;
	return step < 0.0 ? -step : step;
}

static int cmp_file_item(const void *a, const void *b) {
	const file_item_t *fa = (const file_item_t *)a;
	const file_item_t *fb = (const file_item_t *)b;
	if (fa->mtime < fb->mtime) return -1;
	if (fa->mtime > fb->mtime) return 1;
	return strcmp(fa->path, fb->path);
}

static void join_path(char *dst, size_t cap, const char *dir, const char *name) {
	size_t len = strlen(dir);
	if (len > 0 && dir[len - 1] == '/') {
		snprintf(dst, cap, "%s%s", dir, name);
	} else {
		snprintf(dst, cap, "%s/%s", dir, name);
	}
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

static int log_open(const char *path) {
	if (!path || !*path) return -1;
	char dirbuf[PATH_MAX];
	strncpy(dirbuf, path, sizeof(dirbuf) - 1);
	dirbuf[sizeof(dirbuf) - 1] = '\0';
	char *slash = strrchr(dirbuf, '/');
	if (slash) {
		*slash = '\0';
		if (dirbuf[0] != '\0') (void)mkdir_p(dirbuf);
	}
	FILE *fp = fopen(path, "a");
	if (!fp) return -1;
	setvbuf(fp, NULL, _IOLBF, 0);
	g_log_fp = fp;
	strncpy(g_log_path, path, sizeof(g_log_path) - 1);
	g_log_path[sizeof(g_log_path) - 1] = '\0';
	return 0;
}

static void log_close(void) {
	if (g_log_fp) {
		fclose(g_log_fp);
		g_log_fp = NULL;
	}
	g_log_path[0] = '\0';
}

static const char *path_basename(const char *path) {
	const char *p = strrchr(path, '/');
	return p ? p + 1 : path;
}

static int list_log_files(const char *dir, file_item_t **out_items, size_t *out_count, size_t *out_total_bytes) {
	DIR *d = opendir(dir);
	if (!d) return -1;
	size_t cap = 16;
	size_t count = 0;
	size_t total = 0;
	file_item_t *items = (file_item_t *)calloc(cap, sizeof(*items));
	if (!items) { closedir(d); return -1; }

	struct dirent *de;
	while ((de = readdir(d)) != NULL) {
		if (de->d_name[0] == '.') continue;
		if (!has_suffix_ci(de->d_name, ".jsonl") && !has_suffix_ci(de->d_name, ".json")) continue;
		char path[PATH_MAX];
		join_path(path, sizeof(path), dir, de->d_name);
		struct stat st;
		if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) continue;
		if (count == cap) {
			cap *= 2;
			file_item_t *tmp = (file_item_t *)realloc(items, cap * sizeof(*items));
			if (!tmp) { free(items); closedir(d); return -1; }
			items = tmp;
		}
		strncpy(items[count].path, path, sizeof(items[count].path) - 1);
		items[count].mtime = st.st_mtime;
		items[count].size = (size_t)st.st_size;
		total += (size_t)st.st_size;
		count++;
	}
	closedir(d);

	qsort(items, count, sizeof(*items), cmp_file_item);
	*out_items = items;
	*out_count = count;
	*out_total_bytes = total;
	return 0;
}

static int prune_old_files_if_needed(const cfg_t *cfg, const file_item_t *items, size_t count, size_t total_bytes) {
	if (!cfg || !items || count == 0) return 0;
	if (cfg->max_total_bytes == 0) return 0;
	if (total_bytes <= cfg->max_total_bytes) return 0;
	if (count <= 1) return 0;

	size_t keep_idx = count - 1; // list is sorted by mtime asc
	for (size_t i = 0; i < count; i++) {
		if (i == keep_idx) continue;
		LOGW("purge old file (total %zu > %zu): %s", total_bytes, cfg->max_total_bytes, items[i].path);
		(void)remove(items[i].path);
	}
	return 1;
}

static int ensure_bad_dir(cfg_t *cfg) {
	if (cfg->bad_dir && cfg->bad_dir[0] != '\0') return 0;
	snprintf(cfg->bad_dir_buf, sizeof(cfg->bad_dir_buf), "%s/bad_data", cfg->dir);
	cfg->bad_dir = cfg->bad_dir_buf;
	return mkdir_p(cfg->bad_dir);
}

static int move_to_bad(cfg_t *cfg, const char *path) {
	if (ensure_bad_dir(cfg) != 0) {
		LOGW("bad dir create failed: %s", cfg->bad_dir ? cfg->bad_dir : "(null)");
		return -1;
	}
	const char *base = path_basename(path);
	char dst[PATH_MAX];
	join_path(dst, sizeof(dst), cfg->bad_dir, base);
	if (rename(path, dst) == 0) return 0;
	if (errno == EEXIST) {
		time_t t = time(NULL);
		char tmp[PATH_MAX];
		snprintf(tmp, sizeof(tmp), "%s_%ld", dst, (long)t);
		if (rename(path, tmp) == 0) return 0;
	}
	return -1;
}

static bool is_file_stable(const cfg_t *cfg, const file_item_t *item, time_t now) {
	if (cfg->stable_age_s <= 0) return true;
	if (item->mtime == 0) return true;
	return (now - item->mtime) >= cfg->stable_age_s;
}

static upload_result_t upload_json_lines(const cfg_t *cfg, const char *file_buf, size_t len) {
	size_t start = 0;
	size_t sent = 0;
	for (size_t i = 0; i <= len; i++) {
		if (i == len || file_buf[i] == '\n') {
			size_t line_start = start;
			size_t line_len = i - start;

			while (line_len > 0 && (file_buf[line_start] == ' ' || file_buf[line_start] == '\t')) {
				line_start++;
				line_len--;
			}
			while (line_len > 0) {
				char c = file_buf[line_start + line_len - 1];
				if (c == '\r' || c == ' ' || c == '\t') {
					line_len--;
				} else {
					break;
				}
			}

			if (line_len > 0) {
				cJSON *item = cJSON_ParseWithLength(file_buf + line_start, line_len);
				if (!item) return UPLOAD_BAD;

				const char *payload = file_buf + line_start;
				size_t payload_len = line_len;
				char *owned_payload = NULL;
				if (cJSON_IsObject(item)) {
					bool modified = false;
					if (cfg->device_id && cfg->device_id[0] != '\0' &&
						!cJSON_GetObjectItemCaseSensitive(item, "device_id")) {
						if (!cJSON_AddStringToObject(item, "device_id", cfg->device_id)) {
							cJSON_Delete(item);
							return UPLOAD_RETRY;
						}
						modified = true;
					}
					if (cfg->cpuinfo && cfg->cpuinfo[0] != '\0' &&
						!cJSON_GetObjectItemCaseSensitive(item, "cpuinfo")) {
						if (!cJSON_AddStringToObject(item, "cpuinfo", cfg->cpuinfo)) {
							cJSON_Delete(item);
							return UPLOAD_RETRY;
						}
						modified = true;
					}
					if (modified) {
						owned_payload = cJSON_PrintUnformatted(item);
						if (!owned_payload) {
							cJSON_Delete(item);
							return UPLOAD_RETRY;
						}
						payload = owned_payload;
						payload_len = strlen(owned_payload);
					}
				}
				cJSON_Delete(item);

				long code = 0;
				upload_result_t rc = http_post_with_retries(cfg, payload, payload_len, &code);
				if (owned_payload) cJSON_free(owned_payload);
				if (rc != UPLOAD_OK) return rc;
				sent++;
			}
			start = i + 1;
		}
	}
	(void)sent;
	return UPLOAD_OK;
}

static int http_post_json(const cfg_t *cfg, const char *payload, size_t len, long *out_code) {
	CURL *curl = curl_easy_init();
	if (!curl) return -1;
	struct curl_slist *headers = NULL;
	headers = curl_slist_append(headers, "Content-Type: application/json");
	char errbuf[CURL_ERROR_SIZE];
	errbuf[0] = '\0';

	curl_easy_setopt(curl, CURLOPT_URL, cfg->url);
	curl_easy_setopt(curl, CURLOPT_POST, 1L);
	curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload);
	curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)len);
	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, (long)cfg->http_timeout_ms);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, (long)(cfg->http_timeout_ms / 2));
	curl_easy_setopt(curl, CURLOPT_USERAGENT, "http_uploader/1.0");
	curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
	curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, cfg->http_insecure ? 0L : 1L);
	curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, cfg->http_insecure ? 0L : 2L);
	curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);

	CURLcode rc = curl_easy_perform(curl);
	long code = 0;
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);

	curl_slist_free_all(headers);
	curl_easy_cleanup(curl);
	if (out_code) *out_code = code;
	if (rc != CURLE_OK) {
		LOGW("curl error: %s", errbuf[0] ? errbuf : curl_easy_strerror(rc));
	}
	return (rc == CURLE_OK) ? 0 : -1;
}

static upload_result_t http_post_with_retries(const cfg_t *cfg, const char *payload, size_t len, long *out_code) {
	int attempts = (cfg->http_retries < 0) ? 0 : cfg->http_retries;
	int backoff = cfg->http_backoff_ms;
	for (int i = 0; i <= attempts; i++) {
		long code = 0;
		int rc = http_post_json(cfg, payload, len, &code);
		if (out_code) *out_code = code;
		if (rc == 0 && code >= 200 && code < 300) return UPLOAD_OK;
		if (rc == 0 && code >= 400 && code < 500) {
			LOGW("http permanent error %ld", code);
			return UPLOAD_BAD;
		}
		LOGW("http upload failed (attempt %d/%d), code=%ld", i + 1, attempts + 1, code);
		if (i < attempts && backoff > 0) {
			usleep((useconds_t)backoff * 1000u);
			if (backoff < 10000) backoff *= 2;
		}
	}
	return UPLOAD_RETRY;
}

static int add_test_value(cJSON *root, const cfg_t *cfg, test_value_state_t *state) {
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
				cJSON_AddNumberToObject(root, "data", current);
			}
			if (cfg->test_pattern != TEST_PATTERN_FIXED) {
				double step = normalize_test_step(cfg->test_step);
				state->numeric_value = current + (cfg->test_pattern == TEST_PATTERN_DEC ? -step : step);
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

static int send_test_payload_once(const cfg_t *cfg, test_value_state_t *state) {
	cJSON *root = cJSON_CreateObject();
	if (!root) return -1;

	char ts[32];
	format_iso8601_utc(ts, sizeof(ts));
	cJSON_AddStringToObject(root, "ts", ts);
	cJSON_AddStringToObject(root, "cpuinfo", (cfg->cpuinfo && cfg->cpuinfo[0] != '\0') ? cfg->cpuinfo : cfg->device_id);
	if (add_test_value(root, cfg, state) != 0) {
		cJSON_Delete(root);
		return -1;
	}

	char *payload = cJSON_PrintUnformatted(root);
	cJSON_Delete(root);
	if (!payload) return -1;

	long code = 0;
	upload_result_t rc = http_post_with_retries(cfg, payload, strlen(payload), &code);
	if (rc == UPLOAD_OK) {
		LOGD("test payload uploaded");
	} else {
		LOGW("test payload upload failed, code=%ld payload=%s", code, payload);
	}
	cJSON_free(payload);
	return (rc == UPLOAD_OK) ? 0 : -1;
}

static upload_result_t upload_file(const cfg_t *cfg, const file_item_t *item) {
	if (item->size > cfg->max_file_bytes) {
		LOGW("file too large (%zu > %zu): %s", item->size, cfg->max_file_bytes, item->path);
		return UPLOAD_BAD;
	}

	LOGD("uploading file: %s (%zu bytes)", item->path, item->size);
	FILE *f = fopen(item->path, "rb");
	if (!f) {
		if (errno == EACCES || errno == ENOENT) return UPLOAD_SKIP;
		return UPLOAD_RETRY;
	}
	int fd = fileno(f);
	if (flock(fd, LOCK_SH | LOCK_NB) != 0) {
		fclose(f);
		LOGD("file locked, skip: %s", item->path);
		return UPLOAD_SKIP;
	}
	struct stat st;
	if (fstat(fd, &st) != 0 || !S_ISREG(st.st_mode)) {
		fclose(f);
		return UPLOAD_SKIP;
	}
	if ((size_t)st.st_size > cfg->max_file_bytes) {
		fclose(f);
		LOGW("file too large after stat (%zu > %zu): %s", (size_t)st.st_size, cfg->max_file_bytes, item->path);
		return UPLOAD_BAD;
	}

	size_t len = (size_t)st.st_size;
	char *file_buf = (char *)malloc(len + 1u);
	if (!file_buf) { fclose(f); return UPLOAD_RETRY; }
	if (len > 0) {
		size_t rd = fread(file_buf, 1, len, f);
		if (rd != len) { free(file_buf); fclose(f); return UPLOAD_RETRY; }
	}
	file_buf[len] = '\0';
	fclose(f);

	upload_result_t rc = upload_json_lines(cfg, file_buf, len);
	free(file_buf);
	return rc;
}

static int upload_once(cfg_t *cfg) {
	file_item_t *items = NULL;
	size_t count = 0;
	size_t total_bytes = 0;
	if (list_log_files(cfg->dir, &items, &count, &total_bytes) != 0) {
		return -1;
	}

	if (prune_old_files_if_needed(cfg, items, count, total_bytes)) {
		free(items);
		items = NULL;
		count = 0;
		total_bytes = 0;
		if (list_log_files(cfg->dir, &items, &count, &total_bytes) != 0) {
			return -1;
		}
	}

	LOGD("scan dir=%s files=%zu total_bytes=%zu", cfg->dir, count, total_bytes);
	char done_path[PATH_MAX];
	join_path(done_path, sizeof(done_path), cfg->dir, k_done_file_name);
	str_list_t done = {0};
	if (load_done_list(done_path, &done) != 0) {
		LOGW("done list read failed: %s", done_path);
	}

	size_t newest_idx = (count > 0) ? (count - 1) : 0;
	if (count == 0) {
		str_list_free(&done);
		free(items);
		return 0;
	}

	time_t now = time(NULL);
	for (size_t i = 0; i < count; i++) {
		if (items[i].path[0] == '\0') continue;
		if (i == newest_idx) {
			// Skip newest file to avoid uploading an actively appended file.
			LOGD("skip newest file: %s", items[i].path);
			continue;
		}
		if (items[i].size == 0) {
			LOGD("empty file, skip: %s", items[i].path);
			continue;
		}
		const char *base = path_basename(items[i].path);
		if (str_list_contains(&done, base)) {
			LOGD("already sent, skip: %s", items[i].path);
			continue;
		}
		if (!is_file_stable(cfg, &items[i], now)) {
			LOGD("file not stable yet: %s", items[i].path);
			continue;
		}

		upload_result_t rc = upload_file(cfg, &items[i]);
		if (rc == UPLOAD_OK) {
			LOGD("uploaded: %s", items[i].path);
			if (append_done_list(done_path, base) != 0) {
				LOGW("mark sent failed: %s", base);
			} else {
				(void)str_list_add(&done, base);
			}
		} else if (rc == UPLOAD_BAD) {
			LOGW("bad file: %s", items[i].path);
			if (move_to_bad(cfg, items[i].path) != 0) {
				LOGW("move to bad failed, deleting: %s", items[i].path);
				(void)remove(items[i].path);
			}
		} else if (rc == UPLOAD_RETRY) {
			LOGW("upload failed, will retry later: %s", items[i].path);
			break;
		} else {
			continue;
		}
	}

	str_list_free(&done);
	free(items);
	return 0;
}

int main(int argc, char **argv) {
	cfg_t cfg;
	const char *config_path = NULL;
	cfg_defaults(&cfg);

	config_path = http_find_config_path(argc, argv);
	if ((config_path && !*config_path) || ((argc > 1) &&
		((strcmp(argv[argc - 1], "-c") == 0) || (strcmp(argv[argc - 1], "--config") == 0)))) {
		usage(argv[0]);
		return 2;
	}
	if (config_path) {
		char errbuf[256];
		if (demo_config_load(config_path, http_apply_config_entry, &cfg, errbuf, sizeof(errbuf)) != 0) {
			fprintf(stderr, "[E] load config failed: %s\n", errbuf[0] ? errbuf : config_path);
			return 2;
		}
	}

	for (int i = 1; i < argc; i++) {
		if (http_is_config_arg(argv[i])) {
			if (!strcmp(argv[i], "-c") || !strcmp(argv[i], "--config")) i++;
			continue;
		}
		if (!strcmp(argv[i], "--dir") && i + 1 < argc) { cfg.dir = argv[++i]; continue; }
		if (!strcmp(argv[i], "--url") && i + 1 < argc) { cfg.url = argv[++i]; continue; }
		if (!strcmp(argv[i], "--device-id") && i + 1 < argc) { cfg.device_id = argv[++i]; continue; }
		if (!strcmp(argv[i], "--interval-s") && i + 1 < argc) { cfg.interval_s = atoi(argv[++i]); continue; }
		if (!strcmp(argv[i], "--http-timeout-ms") && i + 1 < argc) { cfg.http_timeout_ms = atoi(argv[++i]); continue; }
		if (!strcmp(argv[i], "--http-retries") && i + 1 < argc) { cfg.http_retries = atoi(argv[++i]); continue; }
		if (!strcmp(argv[i], "--http-backoff-ms") && i + 1 < argc) { cfg.http_backoff_ms = atoi(argv[++i]); continue; }
		if (!strcmp(argv[i], "--max-file-kb") && i + 1 < argc) { cfg.max_file_bytes = (size_t)atoi(argv[++i]) * 1024u; continue; }
		if (!strcmp(argv[i], "--max-total-kb") && i + 1 < argc) { cfg.max_total_bytes = (size_t)atoi(argv[++i]) * 1024u; continue; }
		if (!strcmp(argv[i], "--stable-age-s") && i + 1 < argc) { cfg.stable_age_s = atoi(argv[++i]); continue; }
		if (!strcmp(argv[i], "--bad-dir") && i + 1 < argc) { cfg.bad_dir = argv[++i]; continue; }
		if (!strcmp(argv[i], "--log") && i + 1 < argc) { cfg.log_path = argv[++i]; continue; }
		if (!strcmp(argv[i], "--http-insecure")) { cfg.http_insecure = true; continue; }
		if (!strcmp(argv[i], "--test-mode")) { cfg.test_mode = true; continue; }
		if (!strcmp(argv[i], "--test-type") && i + 1 < argc) { cfg.test_value_type = parse_test_value_type(argv[++i]); continue; }
		if (!strcmp(argv[i], "--test-pattern") && i + 1 < argc) { cfg.test_pattern = parse_test_pattern(argv[++i]); continue; }
		if (!strcmp(argv[i], "--test-value") && i + 1 < argc) { snprintf(cfg.test_value, sizeof(cfg.test_value), "%s", argv[++i]); continue; }
		if (!strcmp(argv[i], "--test-step") && i + 1 < argc) { cfg.test_step = strtod(argv[++i], NULL); continue; }
		if (!strcmp(argv[i], "--test-interval-s") && i + 1 < argc) { cfg.test_interval_s = atoi(argv[++i]); continue; }
		usage(argv[0]);
		return 2;
	}

	if ((!cfg.test_mode && !cfg.dir) || !cfg.url) {
		usage(argv[0]);
		return 2;
	}
	if (cfg.interval_s < 1) cfg.interval_s = 1;
	if (cfg.test_interval_s < 1) cfg.test_interval_s = 1;
	if (cfg.http_timeout_ms < 100) cfg.http_timeout_ms = 100;
	if (cfg.http_retries < 0) cfg.http_retries = 0;
	if (cfg.max_file_bytes == 0) cfg.max_file_bytes = MAX_FILE_BYTES;
	if (cfg.stable_age_s < 0) cfg.stable_age_s = 0;
	if (cfg.bad_dir == NULL || cfg.bad_dir[0] == '\0') {
		snprintf(cfg.bad_dir_buf, sizeof(cfg.bad_dir_buf), "%s/bad_data", cfg.dir);
		cfg.bad_dir = cfg.bad_dir_buf;
	}
	if (cfg.log_path == NULL || cfg.log_path[0] == '\0') {
		if (cfg.test_mode) {
			snprintf(cfg.log_path_buf, sizeof(cfg.log_path_buf), "%s", "./http_uploader_test.log");
		} else {
			join_path(cfg.log_path_buf, sizeof(cfg.log_path_buf), cfg.dir, "http_uploader.log");
		}
		cfg.log_path = cfg.log_path_buf;
	}
	if (log_open(cfg.log_path) != 0) {
		fprintf(stderr, "[W] failed to open log file: %s (%s)\n", cfg.log_path, strerror(errno));
	} else {
		LOGI("log file: %s", cfg.log_path);
	}

	if (read_cpuinfo_serial(cfg.cpuinfo_buf, sizeof(cfg.cpuinfo_buf))) {
		cfg.cpuinfo = cfg.cpuinfo_buf;
	} else {
		cfg.cpuinfo = NULL;
	}

	LOGI("start: dir=%s url=%s interval=%ds test_mode=%s test_interval=%ds max_file=%zu max_total=%zu stable_age=%ds bad_dir=%s",
		 cfg.dir, cfg.url, cfg.interval_s, cfg.test_mode ? "true" : "false", cfg.test_interval_s,
		 cfg.max_file_bytes, cfg.max_total_bytes, cfg.stable_age_s, cfg.bad_dir);
	if (cfg.cpuinfo && cfg.cpuinfo[0] != '\0') {
		LOGI("cpuinfo serial: %s", cfg.cpuinfo);
	}

	CURLcode cr = curl_global_init(CURL_GLOBAL_DEFAULT);
	if (cr != CURLE_OK) {
		LOGE("curl init failed: %s", curl_easy_strerror(cr));
		log_close();
		return 1;
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	test_value_state_t test_state = {0};
	while (g_running) {
		if (cfg.test_mode) {
			(void)send_test_payload_once(&cfg, &test_state);
		} else if (upload_once(&cfg) != 0) {
			LOGW("upload scan failed: %s", strerror(errno));
		}
		int wait_seconds = cfg.test_mode ? cfg.test_interval_s : cfg.interval_s;
		for (int i = 0; i < wait_seconds && g_running; i++) {
			sleep(1);
		}
	}

	curl_global_cleanup();
	log_close();
	return 0;
}
