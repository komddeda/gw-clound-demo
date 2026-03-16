#ifndef MODBUS_FRAMER_H
#define MODBUS_FRAMER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>

#include <modbus/modbus.h>

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------ logging ------------------------
#ifndef MB_LOG_LEVEL
#define MB_LOG_LEVEL 3 // 0=quiet, 1=error, 2=warn, 3=info, 4=debug
#endif

#define LOGE(fmt, ...) do { if (MB_LOG_LEVEL >= 1) fprintf(stderr, "[E] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOGW(fmt, ...) do { if (MB_LOG_LEVEL >= 2) fprintf(stderr, "[W] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOGI(fmt, ...) do { if (MB_LOG_LEVEL >= 3) fprintf(stderr, "[I] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOGD(fmt, ...) do { if (MB_LOG_LEVEL >= 4) fprintf(stderr, "[D] " fmt "\n", ##__VA_ARGS__); } while (0)

// ------------------------ spec-related limits ------------------------
#define MB_MAX_READ_BITS 2000u // max for FC01/FC02 per spec
#define MB_QUEUE_CAP 32
#define MB_MAX_WRITE_REGS 123u   // per Modbus spec for FC10
#define MB_MAX_WRITE_COILS 1968u // per spec for FC0F (bits)

#ifndef MB_MIN_T15_US
#define MB_MIN_T15_US 1000u
#endif
#ifndef MB_DEFAULT_BROADCAST_DELAY_US
#define MB_DEFAULT_BROADCAST_DELAY_US 10000u
#endif
#ifndef MB_DEFAULT_RECONNECT_DELAY_MS
#define MB_DEFAULT_RECONNECT_DELAY_MS 200u
#endif
#ifndef MB_DEFAULT_RECONNECT_MAX
#define MB_DEFAULT_RECONNECT_MAX 3u
#endif

// ------------------------ error codes ------------------------
typedef enum {
    MB_OK = 0,
    MB_ERR_PARAM     = -1,
    MB_ERR_IO        = -2,
    MB_ERR_TIMEOUT   = -3,
    MB_ERR_CRC       = -4,
    MB_ERR_PROTO     = -5,
    MB_ERR_EXCEPTION = -6,
    MB_ERR_SHUTDOWN  = -7,
    MB_ERR_PENDING   = -8,
} mb_status_t;

const char *mb_status_str(int st);

// ------------------------ data mapping helpers ------------------------
typedef enum {
    MB_ORDER_ABCD = 0,
    MB_ORDER_BADC,
    MB_ORDER_CDAB,
    MB_ORDER_DCBA,
} mb_word_order_t;

uint32_t mb_u32_from_regs(const uint16_t *regs, mb_word_order_t order);
int32_t mb_s32_from_regs(const uint16_t *regs, mb_word_order_t order);
float mb_f32_from_regs(const uint16_t *regs, mb_word_order_t order);
void mb_u32_to_regs(uint16_t *regs, uint32_t value, mb_word_order_t order);
void mb_f32_to_regs(uint16_t *regs, float value, mb_word_order_t order);

// ------------------------ serial line config ------------------------
typedef struct {
    const char *dev;
    int baud;
    char parity;     // 'N','E','O'
    int data_bits;   // 7/8
    int stop_bits;   // 1/2
    bool rs485_enable;
    bool debug;
    uint32_t t15_min_us;
    uint32_t broadcast_delay_us;
    uint32_t reconnect_delay_ms;
    uint8_t reconnect_max;

    // derived timing
    uint32_t t15_us; // inter-character timeout (t1.5)
    uint32_t t35_us; // inter-frame delay (t3.5)
} mb_line_cfg_t;

// ------------------------ request/response plumbing ------------------------
typedef struct {
    // Multi-register write payload.
    uint16_t n;
    uint16_t vals[MB_MAX_WRITE_REGS];
} mb_wr_regs_t;

typedef struct {
    // Multi-coil write payload (packed LSB-first).
    uint16_t n_bits;
    uint8_t bytes[(MB_MAX_WRITE_COILS + 7u) / 8u];
} mb_wr_coils_t;

typedef union {
    // Union keeps request size small for read/single-write operations.
    mb_wr_regs_t regs;
    mb_wr_coils_t coils;
} mb_wr_payload_t;

typedef struct mb_request {
    // request
    uint8_t  slave;      // 0..247 (0=broadcast)
    uint8_t  func;
    uint16_t addr;
    uint16_t qty;
    uint16_t wr_u16;     // FC06 value; FC05 boolean as 0/1

    // optional payload copies (for multi write)
    mb_wr_payload_t wr;

    // outputs
    uint16_t *out_regs;
    uint16_t  out_regs_cap;
    uint8_t  *out_bits;
    uint16_t  out_bits_cap_bytes;

    // timing
    int timeout_ms;
    int retries;

    // result
    int status;
    uint8_t exception_code;

    // completion
    pthread_mutex_t m;
    pthread_cond_t  cv;
    bool done;
} mb_request_t;

typedef struct {
    // Single-producer/consumer queue guarded by mutex+condvar.
    mb_request_t *q[MB_QUEUE_CAP];
    size_t head, tail, count;
    pthread_mutex_t m;
    pthread_cond_t  cv;
} mb_queue_t;

// ------------------------ context ------------------------
typedef struct {
    modbus_t *mb;
    mb_line_cfg_t line;
    mb_queue_t queue;

    pthread_t th;
    bool running;
    uint64_t last_bus_activity_us;

    // stats
    uint64_t stat_tx;
    uint64_t stat_rx;
    uint64_t stat_timeouts;
    uint64_t stat_crc_err;
    uint64_t stat_proto_err;
} mb_ctx_t;

// ------------------------ public API ------------------------
int mb_ctx_init(mb_ctx_t *ctx, mb_line_cfg_t line);
void mb_ctx_shutdown(mb_ctx_t *ctx);

int mb_submit_async(mb_ctx_t *ctx, mb_request_t *r);
int mb_request_wait(mb_request_t *r, int wait_ms);
void mb_request_cleanup(mb_request_t *r);
bool mb_request_is_done(mb_request_t *r, int *status, uint8_t *exc);

int mb_read_holding(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t qty,
                    uint16_t *out_regs, uint16_t out_cap,
                    int timeout_ms, int retries, uint8_t *exc);
int mb_read_input(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t qty,
                  uint16_t *out_regs, uint16_t out_cap,
                  int timeout_ms, int retries, uint8_t *exc);
int mb_write_single_reg(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t val,
                        int timeout_ms, int retries, uint8_t *exc);
int mb_write_multi_regs(mb_ctx_t *ctx, uint8_t slave, uint16_t addr,
                        const uint16_t *vals, uint16_t n,
                        int timeout_ms, int retries, uint8_t *exc);
int mb_read_coils(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t qty,
                  uint8_t *out_bits, uint16_t out_cap_bytes,
                  int timeout_ms, int retries, uint8_t *exc);
int mb_write_single_coil(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, bool on,
                         int timeout_ms, int retries, uint8_t *exc);
int mb_write_multi_coils(mb_ctx_t *ctx, uint8_t slave, uint16_t addr,
                         const uint8_t *packed_bits, uint16_t n_bits,
                         int timeout_ms, int retries, uint8_t *exc);
int mb_scan_slaves(mb_ctx_t *ctx, uint8_t start, uint8_t end, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
