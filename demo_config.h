#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

typedef int (*demo_config_handler_t)(
    const char *section,
    const char *key,
    const char *value,
    void *user,
    char *errbuf,
    size_t errcap
);

static char *demo_config_ltrim(char *s) {
    while (s && *s && isspace((unsigned char)*s)) {
        s++;
    }
    return s;
}

static void demo_config_rtrim(char *s) {
    if (!s) return;
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) {
        s[n - 1] = '\0';
        n--;
    }
}

static char *demo_config_trim(char *s) {
    char *trimmed = demo_config_ltrim(s);
    demo_config_rtrim(trimmed);
    return trimmed;
}

static void demo_config_unquote(char *s) {
    size_t n;
    if (!s) return;
    n = strlen(s);
    if (n < 2) return;
    if ((s[0] == '"' && s[n - 1] == '"') || (s[0] == '\'' && s[n - 1] == '\'')) {
        memmove(s, s + 1, n - 2);
        s[n - 2] = '\0';
    }
}

static int demo_config_norm_char(int c) {
    if (c == '-' || c == '.' || c == ' ') return '_';
    return tolower((unsigned char)c);
}

static bool demo_config_name_eq(const char *a, const char *b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (demo_config_norm_char(*a) != demo_config_norm_char(*b)) {
            return false;
        }
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static bool demo_config_parse_long(const char *text, long minv, long maxv, long *out) {
    char *end = NULL;
    long value;
    if (!text || !*text || !out) return false;
    errno = 0;
    value = strtol(text, &end, 0);
    if (errno != 0 || end == text || !end || *end != '\0') return false;
    if (value < minv || value > maxv) return false;
    *out = value;
    return true;
}

static bool demo_config_parse_ull(const char *text, unsigned long long *out) {
    char *end = NULL;
    unsigned long long value;
    if (!text || !*text || !out) return false;
    errno = 0;
    value = strtoull(text, &end, 0);
    if (errno != 0 || end == text || !end || *end != '\0') return false;
    *out = value;
    return true;
}

static bool demo_config_parse_double(const char *text, double *out) {
    char *end = NULL;
    double value;
    if (!text || !*text || !out) return false;
    errno = 0;
    value = strtod(text, &end);
    if (errno != 0 || end == text || !end || *end != '\0') return false;
    *out = value;
    return true;
}

static bool demo_config_parse_bool(const char *text, bool *out) {
    if (!text || !*text || !out) return false;
    if (strcasecmp(text, "1") == 0 ||
        strcasecmp(text, "true") == 0 ||
        strcasecmp(text, "yes") == 0 ||
        strcasecmp(text, "on") == 0) {
        *out = true;
        return true;
    }
    if (strcasecmp(text, "0") == 0 ||
        strcasecmp(text, "false") == 0 ||
        strcasecmp(text, "no") == 0 ||
        strcasecmp(text, "off") == 0) {
        *out = false;
        return true;
    }
    return false;
}

static int demo_config_load(const char *path,
                            demo_config_handler_t handler,
                            void *user,
                            char *errbuf,
                            size_t errcap) {
    FILE *fp;
    char line[2048];
    char section[128];
    int lineno;

    if (errbuf && errcap > 0) errbuf[0] = '\0';
    if (!path || !*path || !handler) {
        if (errbuf && errcap > 0) {
            snprintf(errbuf, errcap, "invalid config load request");
        }
        return -1;
    }

    fp = fopen(path, "r");
    if (!fp) {
        if (errbuf && errcap > 0) {
            snprintf(errbuf, errcap, "open %s failed: %s", path, strerror(errno));
        }
        return -1;
    }

    section[0] = '\0';
    lineno = 0;
    while (fgets(line, sizeof(line), fp) != NULL) {
        char *text;
        size_t n;
        lineno++;

        n = strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) {
            line[n - 1] = '\0';
            n--;
        }

        text = demo_config_trim(line);
        if (*text == '\0' || *text == '#' || *text == ';') {
            continue;
        }

        if (*text == '[') {
            char *end = strrchr(text, ']');
            char *name;
            if (!end || end == text + 1 || end[1] != '\0') {
                if (errbuf && errcap > 0) {
                    snprintf(errbuf, errcap, "%s:%d invalid section header", path, lineno);
                }
                fclose(fp);
                return -1;
            }
            *end = '\0';
            name = demo_config_trim(text + 1);
            if (*name == '\0') {
                if (errbuf && errcap > 0) {
                    snprintf(errbuf, errcap, "%s:%d empty section name", path, lineno);
                }
                fclose(fp);
                return -1;
            }
            snprintf(section, sizeof(section), "%s", name);
            continue;
        }

        {
            char *eq = strchr(text, '=');
            char *key;
            char *value;
            int rc;
            if (!eq) {
                if (errbuf && errcap > 0) {
                    snprintf(errbuf, errcap, "%s:%d expected key=value", path, lineno);
                }
                fclose(fp);
                return -1;
            }
            *eq = '\0';
            key = demo_config_trim(text);
            value = demo_config_trim(eq + 1);
            demo_config_unquote(value);
            if (*key == '\0') {
                if (errbuf && errcap > 0) {
                    snprintf(errbuf, errcap, "%s:%d empty key", path, lineno);
                }
                fclose(fp);
                return -1;
            }
            rc = handler(section, key, value, user, errbuf, errcap);
            if (rc != 0) {
                if (errbuf && errcap > 0 && errbuf[0] == '\0') {
                    snprintf(errbuf, errcap, "%s:%d invalid value for %s", path, lineno, key);
                }
                fclose(fp);
                return -1;
            }
        }
    }

    fclose(fp);
    return 0;
}

#endif
