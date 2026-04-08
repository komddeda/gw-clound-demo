/*
 * modbus_framer_core.c（中文注释增强版）
 *
 * 这是一个面向 Linux/POSIX 的 Modbus RTU 主站（Master）核心实现，目标是“工程可用”：
 *
 * 设计要点（为什么这么写）
 * 1) 单 I/O 线程串行化访问串口：
 *    - Modbus RTU 是“总线”语义（半双工/共享介质），同一时刻只能有一个请求在路上。
 *    - 如果多个线程同时直接读写串口，会打乱 RTU 帧边界（t3.5 静默间隔）并导致 CRC/帧错。
 *    - 因此：外部可以多线程提交请求，内部由一个 worker 线程按队列顺序发送/接收。
 *
 * 2) 请求队列 + 条件变量同步：
 *    - submitter 把 mb_request_t 放入队列；worker 处理完后通过 condvar 唤醒等待者。
 *    - 支持“同步调用”（提交后等待）与“异步调用”（提交后轮询/等待）两种模式。
 *
 * 3) 计时采用 CLOCK_MONOTONIC：
 *    - 只用于计算间隔/超时，避免系统时间被 NTP/手动调整导致的跳变问题。
 *
 * 4) 严格遵守 RTU 帧间隔：
 *    - RTU 规定帧之间至少需要 t3.5 的静默时间（以字符时间换算）。
 *    - 本实现用 last_bus_activity_us 记录上一次总线活动时间，发送前确保静默足够。
 *
 * 5) 工程化容错：
 *    - 通过 libmodbus 的 errno 映射，区分 TIMEOUT/CRC/PROTO/EXCEPTION/IO 等错误。
 *    - I/O 错误时尝试重连；支持重试次数 retries。
 *
 * 注意：
 * - 本文件是“核心库实现”，通常会被 CLI 工具或上层业务代码调用。
 * - 你如果要做更强的压测（长期读写/随机写/统计耗时），建议在上层再加一层调度逻辑。
 */

#define _GNU_SOURCE
#include "modbus_framer.h"

#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>

#ifndef MB_IO_THREAD_STACK_SIZE
#define MB_IO_THREAD_STACK_SIZE (256u * 1024u)
#endif

#ifdef __linux__
#include <linux/serial.h>
#endif

// ------------------------ Modbus 功能码（本文件用到的子集） ------------------------
// 说明：这里只列出本实现封装的常用功能码；其它功能码可按同样方式扩展。
enum {
	MB_FC01_READ_COILS            = 0x01,
	MB_FC02_READ_DISCRETE_INPUTS  = 0x02,
	MB_FC03_READ_HOLDING_REGS     = 0x03,
	MB_FC04_READ_INPUT_REGS       = 0x04,
	MB_FC05_WRITE_SINGLE_COIL     = 0x05,
	MB_FC06_WRITE_SINGLE_REG      = 0x06,
	MB_FC0F_WRITE_MULTI_COILS     = 0x0F,
	MB_FC10_WRITE_MULTI_REGS      = 0x10,
};

const char *mb_status_str(int st) {
	switch (st) {
		case MB_OK: return "OK";
		case MB_ERR_PARAM: return "PARAM";
		case MB_ERR_IO: return "IO";
		case MB_ERR_TIMEOUT: return "TIMEOUT";
		case MB_ERR_CRC: return "CRC";
		case MB_ERR_PROTO: return "PROTO";
		case MB_ERR_EXCEPTION: return "EXCEPTION";
		case MB_ERR_SHUTDOWN: return "SHUTDOWN";
		case MB_ERR_PENDING: return "PENDING";
		default: return "UNKNOWN";
	}
}

// ------------------------ 小工具函数：计时/休眠/字节序转换 ------------------------
// 这些函数看似“杂项”，但决定了超时统计、RTU 静默间隔计算、寄存器映射等基础行为。
// 计时函数使用 CLOCK_MONOTONIC：用于可靠的时间间隔计算（不受系统校时影响）。
static uint64_t now_us(void) {
	/*
	 * 使用 CLOCK_MONOTONIC 获取“单调递增”的时间戳（微秒）。
	 * - 这里必须避免 CLOCK_REALTIME：系统时间被校时会前跳/后跳，导致超时判断失真。
	 * - 返回单位是 us，便于直接与 t1.5/t3.5（微秒）做运算，避免浮点累积误差。
	 */
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void sleep_us(uint64_t us) {
	/*
	 * 微秒级睡眠（基于 nanosleep）。
	 * - nanosleep 可能被信号中断返回 EINTR：此时 ts 会被内核改写为“剩余时间”，循环即可。
	 * - 该函数用于：保证 RTU 帧间隔静默时间、重连退避、广播延时等。
	 */
	struct timespec ts;
	ts.tv_sec = (time_t)(us / 1000000ull);
	ts.tv_nsec = (long)((us % 1000000ull) * 1000ull);
	while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}
}

// 32 位数据在 Modbus 寄存器中的常见映射（字序/字节序）辅助函数。
uint32_t mb_u32_from_regs(const uint16_t *regs, mb_word_order_t order) {
	/*
	 * 将两个 16-bit 寄存器组合为 32-bit 值（常见于能量表/变频器/PLC 的 32 位量）。
	 *
	 * 背景：
	 * - Modbus 协议本身只定义“寄存器序列”，并不规定 32-bit/float 的字节序。
	 * - 工业设备常见 4 种排列：ABCD / BADC / CDAB / DCBA（有的叫字节交换/字交换）。
	 *
	 * 实现策略：
	 * - 先把 regs[0]/regs[1] 拆成 4 个字节 b0..b3（按“寄存器高字节在前”的常规理解）。
	 * - 再按照 order 重排 bytes[0..3]，最后拼回 32-bit。
	 */
	// 约定：regs[0] 为高 16 位，regs[1] 为低 16 位（常见 Modbus 设备约定）。
	uint8_t b0 = (uint8_t)(regs[0] >> 8);
	uint8_t b1 = (uint8_t)(regs[0] & 0xFF);
	uint8_t b2 = (uint8_t)(regs[1] >> 8);
	uint8_t b3 = (uint8_t)(regs[1] & 0xFF);
	uint8_t bytes[4];

	switch (order) {
		case MB_ORDER_ABCD: bytes[0] = b0; bytes[1] = b1; bytes[2] = b2; bytes[3] = b3; break;
		case MB_ORDER_BADC: bytes[0] = b1; bytes[1] = b0; bytes[2] = b3; bytes[3] = b2; break;
		case MB_ORDER_CDAB: bytes[0] = b2; bytes[1] = b3; bytes[2] = b0; bytes[3] = b1; break;
		case MB_ORDER_DCBA: bytes[0] = b3; bytes[1] = b2; bytes[2] = b1; bytes[3] = b0; break;
		default: bytes[0] = b0; bytes[1] = b1; bytes[2] = b2; bytes[3] = b3; break;
	}

	return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
		   ((uint32_t)bytes[2] << 8) | (uint32_t)bytes[3];
}

int32_t mb_s32_from_regs(const uint16_t *regs, mb_word_order_t order) {
	return (int32_t)mb_u32_from_regs(regs, order);
}

float mb_f32_from_regs(const uint16_t *regs, mb_word_order_t order) {
	// 通过 memcpy 安全地做类型转换，避免 strict-aliasing（严格别名）导致的未定义行为。
	uint32_t u = mb_u32_from_regs(regs, order);
	float f;
	memcpy(&f, &u, sizeof(f));
	return f;
}

void mb_u32_to_regs(uint16_t *regs, uint32_t value, mb_word_order_t order) {
	/*
	 * 32-bit -> 两个 16-bit 寄存器的反向映射。
	 * - 注意：这里的“order”含义与 mb_u32_from_regs 对称，保证 round-trip 一致。
	 * - 最终写入 regs[0], regs[1]，便于上层调用写多个寄存器（0x10）时直接引用。
	 */
	uint8_t bytes[4];
	bytes[0] = (uint8_t)((value >> 24) & 0xFF);
	bytes[1] = (uint8_t)((value >> 16) & 0xFF);
	bytes[2] = (uint8_t)((value >> 8) & 0xFF);
	bytes[3] = (uint8_t)(value & 0xFF);

	switch (order) {
		case MB_ORDER_ABCD:
			regs[0] = (uint16_t)((uint16_t)bytes[0] << 8) | bytes[1];
			regs[1] = (uint16_t)((uint16_t)bytes[2] << 8) | bytes[3];
			break;
		case MB_ORDER_BADC:
			regs[0] = (uint16_t)((uint16_t)bytes[1] << 8) | bytes[0];
			regs[1] = (uint16_t)((uint16_t)bytes[3] << 8) | bytes[2];
			break;
		case MB_ORDER_CDAB:
			regs[0] = (uint16_t)((uint16_t)bytes[2] << 8) | bytes[3];
			regs[1] = (uint16_t)((uint16_t)bytes[0] << 8) | bytes[1];
			break;
		case MB_ORDER_DCBA:
			regs[0] = (uint16_t)((uint16_t)bytes[3] << 8) | bytes[2];
			regs[1] = (uint16_t)((uint16_t)bytes[1] << 8) | bytes[0];
			break;
		default:
			regs[0] = (uint16_t)((uint16_t)bytes[0] << 8) | bytes[1];
			regs[1] = (uint16_t)((uint16_t)bytes[2] << 8) | bytes[3];
			break;
	}
}

void mb_f32_to_regs(uint16_t *regs, float value, mb_word_order_t order) {
	// 注意：具体设备的字节序/字序由调用者按实际协议选择 order。
	uint32_t u;
	memcpy(&u, &value, sizeof(u));
	mb_u32_to_regs(regs, u, order);
}

// ------------------------ 串口/RTU 时序参数计算（t1.5 / t3.5） ------------------------
// RTU 的“帧边界”依赖串口线路的静默时间。这里根据波特率计算并做工程化钳位。
static void calc_modbus_timers(mb_line_cfg_t *lc) {
	/*
	 * 计算 RTU 帧相关的关键时间参数：
	 * - t1.5：字符间隔超时（用于判断帧内“断字节/断帧”）
	 * - t3.5：帧间最小静默时间（用于判断两帧边界）
	 *
	 * 标准要点（Modbus over Serial Line）：
	 * - 低波特率（<=19200）：t1.5/t3.5 用“字符时间”换算更准确；
	 * - 高波特率（>19200）：推荐固定值 t1.5=750us, t3.5=1750us（降低 CPU 负担、提高兼容性）。
	 *
	 * 工程化钳位：
	 * - 非实时 Linux 下调度粒度/线程抢占可能导致实际间隔更大；
	 * - 但过小的 t1.5 反而容易误判（尤其 USB 转串口/驱动缓冲），因此这里还会做最小值限制。
	 */
	// 依据规范计算时序；同时对非实时内核做最小 t1.5 钳位，避免过小导致误判。
	// 规范：baud<=19200 时，t1.5/t3.5 按 1.5/3.5 个字符时间计算。
	// 规范建议：baud>19200 时使用固定值：t1.5=750us，t3.5=1750us。
	// （这里遵循该建议，以提升兼容性并降低 CPU 压力。）
	if (lc->baud > 19200) {
		lc->t15_us = 750;
		lc->t35_us = 1750;
	} else {
		int parity_bits = (lc->parity == 'N' || lc->parity == 'n') ? 0 : 1;
		int bits_per_char = 1 /*start*/ + lc->data_bits + parity_bits + lc->stop_bits;
		double char_time_us = ((double)bits_per_char * 1000000.0) / (double)lc->baud;
		lc->t15_us = (uint32_t)(1.5 * char_time_us + 0.5);
		lc->t35_us = (uint32_t)(3.5 * char_time_us + 0.5);
	}
	if (lc->t15_us < lc->t15_min_us) lc->t15_us = lc->t15_min_us;
}

static void mb_apply_line_defaults(mb_line_cfg_t *lc) {
	// 当调用者未填写可选字段（为 0）时，补齐工程默认值，避免运行时出现不合理参数。
	if (lc->t15_min_us == 0) lc->t15_min_us = MB_MIN_T15_US;
	if (lc->broadcast_delay_us == 0) lc->broadcast_delay_us = MB_DEFAULT_BROADCAST_DELAY_US;
	if (lc->reconnect_delay_ms == 0) lc->reconnect_delay_ms = MB_DEFAULT_RECONNECT_DELAY_MS;
	if (lc->reconnect_max == 0) lc->reconnect_max = MB_DEFAULT_RECONNECT_MAX;
}

// ------------------------ 请求队列与同步机制 ------------------------
// 多线程提交请求 -> 放入队列；单 worker 线程取出并串行执行；完成后唤醒等待者。
static void queue_init(mb_queue_t *q) {
	// 队列在“提交线程们”和“worker 线程”之间共享，必须用互斥锁+条件变量保护。
	memset(q, 0, sizeof(*q));
	pthread_mutex_init(&q->m, NULL);
	pthread_cond_init(&q->cv, NULL);
}

static void queue_destroy(mb_queue_t *q) {
	pthread_mutex_destroy(&q->m);
	pthread_cond_destroy(&q->cv);
}

static int queue_push(mb_queue_t *q, mb_request_t *r, bool *running) {
	/*
	 * 入队（生产者侧）：
	 * - 当队列满时，提交线程会阻塞等待；避免忙等造成 CPU 空转。
	 * - shutdown 时（*running=false）需要让阻塞的线程退出，所以等待循环条件包含 *running。
	 *
	 * 注意：
	 * - 这里使用 pthread_cond_broadcast 而不是 signal：
	 *   因为可能既有“等待队列非空的 worker”，也有“等待队列有空位的 submitter”，广播更稳妥。
	 */
	// 队列满则阻塞等待；若收到 shutdown（running=false）则立即退出并返回错误码。
	pthread_mutex_lock(&q->m);
	while (q->count == MB_QUEUE_CAP && *running) pthread_cond_wait(&q->cv, &q->m);
	if (!*running) {
		pthread_mutex_unlock(&q->m);
		return MB_ERR_SHUTDOWN;
	}
	q->q[q->tail] = r;
	q->tail = (q->tail + 1) % MB_QUEUE_CAP;
	q->count++;
	pthread_cond_broadcast(&q->cv);
	pthread_mutex_unlock(&q->m);
	return MB_OK;
}

static mb_request_t *queue_pop(mb_queue_t *q, bool *running) {
	/*
	 * 出队（消费者侧，worker 线程）：
	 * - 队列空时阻塞等待；
	 * - 如果已经 shutdown 且队列也空，则返回 NULL 让 worker 线程自然退出。
	 *
	 * 这样设计的好处：
	 * - shutdown 不需要硬杀线程；只要设置 running=false 并广播条件变量即可。
	 */
	// 队列空则阻塞等待；若已 shutdown 且仍为空，则返回 NULL 让 worker 退出。
	pthread_mutex_lock(&q->m);
	while (q->count == 0 && *running) pthread_cond_wait(&q->cv, &q->m);
	if (q->count == 0 && !*running) {
		pthread_mutex_unlock(&q->m);
		return NULL;
	}
	mb_request_t *r = q->q[q->head];
	q->head = (q->head + 1) % MB_QUEUE_CAP;
	q->count--;
	pthread_cond_broadcast(&q->cv);
	pthread_mutex_unlock(&q->m);
	return r;
}

// ------------------------ 上下文与前置声明 ------------------------
static void ensure_bus_idle(mb_ctx_t *ctx);
	/*
	 * 计算距离“上一次总线活动”是否已经超过 t3.5：
	 * - last_bus_activity_us 在每次请求完成（或至少发送/接收后）更新；
	 * - 若还未达到 t3.5，则 sleep 剩余时间。
	 *
	 * 这里用“微秒”计算避免浮点误差，且逻辑直观。
	 */

// ------------------------ libmodbus 适配层：错误映射/打开/重连 ------------------------
// libmodbus 的很多信息通过 errno 反馈，本层把它翻译成更直观的 MB_ERR_*。
static int map_libmodbus_error(mb_request_t *r) {
	/*
	 * 将 libmodbus 的 errno 映射为本工程统一的错误码（MB_ERR_*）。
	 *
	 * 为什么要做这层映射：
	 * - 上层业务通常关心的是“超时/CRC/协议错误/从站异常/IO 断开”，而不是各种 errno。
	 * - 另外 Modbus 异常（Exception）在 libmodbus 中会被编码到一个 errno 区间（MODBUS_ENOBASE）。
	 *
	 * 关键点：
	 * - CRC 错误：主站收到了响应，但校验不通过 -> 需要统计 + 可能重试；
	 * - TIMEOUT：无响应/帧不完整 -> 常见于从站不在线、地址错误、485 方向控制问题；
	 * - EXCEPTION：从站明确回了异常码（例如 0x02 非法地址），不是链路问题。
	 */
	// 将 libmodbus 的 errno 翻译为本工程的统一状态码（MB_ERR_*）。
	int e = errno;
#ifdef EMBBADCRC
	if (e == EMBBADCRC) return MB_ERR_CRC;
#endif
#ifdef EMBBADDATA
	if (e == EMBBADDATA) return MB_ERR_PROTO;
#endif
#ifdef EMBBADEXC
	if (e == EMBBADEXC) return MB_ERR_PROTO;
#endif
#ifdef MODBUS_ENOBASE
	if (e >= MODBUS_ENOBASE && e < MODBUS_ENOBASE + 0x100) {
		r->exception_code = (uint8_t)(e - MODBUS_ENOBASE);
		return MB_ERR_EXCEPTION;
	}
#endif
	if (e == ETIMEDOUT) return MB_ERR_TIMEOUT;
	if (e == EINVAL) return MB_ERR_PARAM;
	return MB_ERR_IO;
}

static void mb_close_modbus(mb_ctx_t *ctx) {
	// 安全关闭/释放封装（幂等）：多次调用也不会出错。
	if (ctx->mb) {
		modbus_close(ctx->mb);
		modbus_free(ctx->mb);
		ctx->mb = NULL;
	}
}

static int mb_open_modbus(mb_ctx_t *ctx) {
	/*
	 * 打开并连接 Modbus RTU：
	 * - modbus_new_rtu 创建上下文（设置串口参数：设备、波特率、校验、数据位、停止位）
	 * - modbus_connect 真正打开 fd 并配置 termios
	 *
	 */
	// 创建并连接新的 libmodbus RTU 上下文。
	ctx->mb = modbus_new_rtu(ctx->line.dev, ctx->line.baud, ctx->line.parity,
							 ctx->line.data_bits, ctx->line.stop_bits);
	if (!ctx->mb) {
		LOGE("modbus_new_rtu failed: %s", modbus_strerror(errno));
		return MB_ERR_IO;
	}

	if (ctx->line.debug) modbus_set_debug(ctx->mb, 1);

	if (modbus_connect(ctx->mb) == -1) {
		LOGE("modbus_connect failed: %s", modbus_strerror(errno));
		mb_close_modbus(ctx);
		return MB_ERR_IO;
	}

#ifdef __linux__
	if (ctx->line.rs485_enable) {
		int fd = modbus_get_socket(ctx->mb);
		if (fd >= 0) {
			struct serial_rs485 rs = {0};
			rs.flags |= SER_RS485_ENABLED;
			if (ioctl(fd, TIOCSRS485, &rs) != 0) {
				LOGW("TIOCSRS485 failed (driver may not support RS485 ioctl): %s", strerror(errno));
			} else {
				LOGI("%s", "RS485 mode enabled via TIOCSRS485");
			}
		} else {
			LOGW("%s", "modbus_get_socket failed for RS485 ioctl");
		}
	}
#endif

	return MB_OK;
}

static int mb_reconnect(mb_ctx_t *ctx) {
	/*
	 * 重连策略：
	 * - 典型场景：USB 串口抖动/插拔、485 收发器瞬断、驱动返回 EIO 等。
	 * - 做法：先 close/free，然后按次数重试 open/connect，并在两次之间 sleep 做退避。
	 *
	 * 注意：
	 * - 这里不做指数退避，采用固定间隔，避免引入过多复杂度；
	 * - 如果你现场链路非常不稳定，可以在这里改成指数退避/上限钳位。
	 */
	// 基本重连循环：worker 遇到 MB_ERR_IO 时调用，用于恢复串口/驱动层异常。
	mb_close_modbus(ctx);

	for (uint8_t attempt = 0; attempt < ctx->line.reconnect_max && ctx->running; attempt++) {
		if (mb_open_modbus(ctx) == MB_OK) return MB_OK;
		if (attempt + 1u < ctx->line.reconnect_max) {
			sleep_us((uint64_t)ctx->line.reconnect_delay_ms * 1000ull);
		}
	}

	return MB_ERR_IO;
}

static void apply_broadcast_delay(mb_ctx_t *ctx) {
	/*
	 * 广播（slave=0）的特殊处理：
	 * - Modbus 规范：广播写请求“从站不应答”，否则会总线冲突。
	 * - 但从站仍需要一定时间去执行写入（可能写 EEPROM/Flash）。
	 * - 因此主站在广播写后应该等待一小段时间，再发送下一帧，避免占满总线导致从站处理不过来。
	 */
	// 广播写后等待：给从站留出处理时间，再发下一帧，避免从站忙不过来。
	if (ctx->line.broadcast_delay_us > 0) sleep_us(ctx->line.broadcast_delay_us);
}

static void pack_bits(uint8_t *dst, uint16_t dst_bytes, const uint8_t *src_bits, uint16_t n_bits) {
	/*
	 * bit 打包：把 libmodbus 读出来的 bits[]（每个元素 0/1，占 1 字节）
	 * 转换为 Modbus 协议线上的“按位打包、LSB-first”的字节流。
	 *
	 * 举例：
	 * - coil0 对应字节 bit0（最低位），coil7 对应 bit7；
	 * - coil8 则落到下一个字节的 bit0。
	 */
	// 将 libmodbus 的 0/1 数组压成 Modbus 线上的按位打包字节（LSB-first）。
	memset(dst, 0, dst_bytes);
	for (uint16_t i = 0; i < n_bits; i++) {
		if (src_bits[i]) dst[i / 8u] |= (uint8_t)(1u << (i % 8u));
	}
}

static void unpack_bits(uint8_t *dst_bits, uint16_t n_bits, const uint8_t *src_bytes) {
	/*
	 * bit 解包：与 pack_bits 相反。
	 * - 上层写多个线圈（0x0F）通常传入“按位打包”的数据；
	 * - libmodbus 的 modbus_write_bits 需要 0/1 数组，因此这里展开成 bytes->bits[]。
	 */
	// 将按位打包字节展开为 libmodbus 需要的 0/1 数组。
	for (uint16_t i = 0; i < n_bits; i++) {
		dst_bits[i] = (uint8_t)((src_bytes[i / 8u] >> (i % 8u)) & 0x01u);
	}
}

static int set_request_timeouts(mb_ctx_t *ctx, mb_request_t *r, bool broadcast) {
	/*
	 * 为“本次请求”设置超时：
	 * 1) response timeout：
	 *    - 普通单播请求：等待响应的最大时间（由 r->timeout_ms 决定）
	 *    - 广播写：规范要求无响应，因此把响应超时设为 0（相当于不等待）
	 *
	 * 2) byte timeout（字符间隔超时）：
	 *    - 用于“帧内字节间隔”判定：超过 t1.5 认为本帧结束/断帧。
	 *    - 对抗现场常见问题：USB 转串口分包、驱动缓冲导致的间隙。
	 */
	// 广播写不应答：因此禁用响应超时等待（相当于不等响应）。
	uint32_t sec = 0;
	uint32_t usec = 0;
	if (!broadcast) {
		sec = (uint32_t)(r->timeout_ms / 1000);
		usec = (uint32_t)((r->timeout_ms % 1000) * 1000);
	}
	if (modbus_set_response_timeout(ctx->mb, sec, usec) == -1) return map_libmodbus_error(r);
	if (ctx->line.t15_us > 0) {
		if (modbus_set_byte_timeout(ctx->mb, 0, (uint32_t)ctx->line.t15_us) == -1) return map_libmodbus_error(r);
	}
	return MB_OK;
}

static int do_libmodbus_request(mb_ctx_t *ctx, mb_request_t *r) {
	/*
	 * worker 线程执行单个请求的核心函数：
	 * - 设置 slave 地址
	 * - 设置超时（响应/字节）
	 * - flush 串口输入缓冲（避免上一次残留字节污染下一帧）
	 * - 确保 t3.5 静默时间后再发送（RTU 帧边界要求）
	 * - 调用 libmodbus 对应的 read/write API
	 * - 校验返回的字节数/寄存器数，防止“部分返回”被误当成成功
	 *
	 * 广播（slave=0）特别注意：
	 * - 禁止广播读（没有意义且规范不允许）
	 * - 广播写时不等待响应：若 libmodbus 因无响应报 TIMEOUT，这里把它视为成功。
	 */
	// 把队列中的一个请求翻译并执行为对应的 libmodbus API 调用。
	bool broadcast = (r->slave == 0);

	if (!ctx->mb) return MB_ERR_IO;

	if (modbus_set_slave(ctx->mb, r->slave) == -1) return map_libmodbus_error(r);

	int rc = set_request_timeouts(ctx, r, broadcast);
	if (rc != MB_OK) return rc;

	if (broadcast) {
		if (r->func == MB_FC01_READ_COILS || r->func == MB_FC02_READ_DISCRETE_INPUTS ||
			r->func == MB_FC03_READ_HOLDING_REGS || r->func == MB_FC04_READ_INPUT_REGS) {
			return MB_ERR_PARAM;
		}
	}

		// 清空串口接收缓冲区：
		// - 现场经常出现“上一帧的尾巴/噪声字节”残留在缓冲里；
		// - 若不 flush，下一次请求的响应解析可能从残留字节开始，导致 CRC/帧头错误。
		
	modbus_flush(ctx->mb);
		// 发送前确保总线满足 RTU 的帧间静默（t3.5）：
		// - 这是 RTU 判帧边界的核心要求；
		// - 如果连续发送太快，从站可能把两帧粘在一起解析，直接导致协议错误。
		
	ensure_bus_idle(ctx);
	ctx->stat_tx++;

		// 根据功能码分派到具体读写：
		// - 每个 case 除了调用 libmodbus，还会做“返回数量一致性检查”；
		// - 这是为了避免“部分响应”被当成成功（在噪声/中断环境下很常见）。
		
	switch (r->func) {
		case MB_FC03_READ_HOLDING_REGS:
			// 读操作把数据填入调用者提供的 buffer；并检查返回数量，防止部分返回被误判为成功。
			if (!r->out_regs || r->out_regs_cap < r->qty) return MB_ERR_PARAM;
			rc = modbus_read_registers(ctx->mb, (int)r->addr, (int)r->qty, r->out_regs);
			if (rc == -1) return map_libmodbus_error(r);
			return (rc == (int)r->qty) ? MB_OK : MB_ERR_PROTO;
		case MB_FC04_READ_INPUT_REGS:
			if (!r->out_regs || r->out_regs_cap < r->qty) return MB_ERR_PARAM;
			rc = modbus_read_input_registers(ctx->mb, (int)r->addr, (int)r->qty, r->out_regs);
			if (rc == -1) return map_libmodbus_error(r);
			return (rc == (int)r->qty) ? MB_OK : MB_ERR_PROTO;
		case MB_FC01_READ_COILS: {
			// libmodbus 读到的每个 bit 占 1 字节；这里重新打包成 Modbus 协议要求的位流。
			if (!r->out_bits) return MB_ERR_PARAM;
			if (r->qty == 0 || r->qty > MB_MAX_READ_BITS) return MB_ERR_PARAM;
			uint16_t need_bytes = (uint16_t)((r->qty + 7u) / 8u);
			if (r->out_bits_cap_bytes < need_bytes) return MB_ERR_PARAM;
			uint8_t bits[MB_MAX_READ_BITS];
			rc = modbus_read_bits(ctx->mb, (int)r->addr, (int)r->qty, bits);
			if (rc == -1) return map_libmodbus_error(r);
			if (rc != (int)r->qty) return MB_ERR_PROTO;
			pack_bits(r->out_bits, need_bytes, bits, r->qty);
			return MB_OK;
		}
		case MB_FC02_READ_DISCRETE_INPUTS: {
			if (!r->out_bits) return MB_ERR_PARAM;
			if (r->qty == 0 || r->qty > MB_MAX_READ_BITS) return MB_ERR_PARAM;
			uint16_t need_bytes = (uint16_t)((r->qty + 7u) / 8u);
			if (r->out_bits_cap_bytes < need_bytes) return MB_ERR_PARAM;
			uint8_t bits[MB_MAX_READ_BITS];
			rc = modbus_read_input_bits(ctx->mb, (int)r->addr, (int)r->qty, bits);
			if (rc == -1) return map_libmodbus_error(r);
			if (rc != (int)r->qty) return MB_ERR_PROTO;
			pack_bits(r->out_bits, need_bytes, bits, r->qty);
			return MB_OK;
		}
		case MB_FC06_WRITE_SINGLE_REG:
			// 广播写是“发出即走”：由于从站不回响应，遇到 TIMEOUT 也视为成功。
			rc = modbus_write_register(ctx->mb, (int)r->addr, (int)r->wr_u16);
			if (rc == -1) {
				int st = map_libmodbus_error(r);
				if (broadcast && st == MB_ERR_TIMEOUT) {
					apply_broadcast_delay(ctx);
					return MB_OK;
				}
				return st;
			}
			if (broadcast) apply_broadcast_delay(ctx);
			return (!broadcast && rc != 1) ? MB_ERR_PROTO : MB_OK;
		case MB_FC05_WRITE_SINGLE_COIL:
			rc = modbus_write_bit(ctx->mb, (int)r->addr, r->wr_u16 ? 1 : 0);
			if (rc == -1) {
				int st = map_libmodbus_error(r);
				if (broadcast && st == MB_ERR_TIMEOUT) {
					apply_broadcast_delay(ctx);
					return MB_OK;
				}
				return st;
			}
			if (broadcast) apply_broadcast_delay(ctx);
			return (!broadcast && rc != 1) ? MB_ERR_PROTO : MB_OK;
		case MB_FC10_WRITE_MULTI_REGS:
			// 多寄存器/多线圈写把数据放在 union 里，避免 request 结构体过大。
			if (r->wr.regs.n == 0 || r->wr.regs.n > MB_MAX_WRITE_REGS) return MB_ERR_PARAM;
			rc = modbus_write_registers(ctx->mb, (int)r->addr, (int)r->wr.regs.n, r->wr.regs.vals);
			if (rc == -1) {
				int st = map_libmodbus_error(r);
				if (broadcast && st == MB_ERR_TIMEOUT) {
					apply_broadcast_delay(ctx);
					return MB_OK;
				}
				return st;
			}
			if (broadcast) apply_broadcast_delay(ctx);
			return (!broadcast && rc != (int)r->wr.regs.n) ? MB_ERR_PROTO : MB_OK;
		case MB_FC0F_WRITE_MULTI_COILS: {
			if (r->wr.coils.n_bits == 0 || r->wr.coils.n_bits > MB_MAX_WRITE_COILS) return MB_ERR_PARAM;
			uint8_t bits[MB_MAX_WRITE_COILS];
			unpack_bits(bits, r->wr.coils.n_bits, r->wr.coils.bytes);
			rc = modbus_write_bits(ctx->mb, (int)r->addr, (int)r->wr.coils.n_bits, bits);
			if (rc == -1) {
				int st = map_libmodbus_error(r);
				if (broadcast && st == MB_ERR_TIMEOUT) {
					apply_broadcast_delay(ctx);
					return MB_OK;
				}
				return st;
			}
			if (broadcast) apply_broadcast_delay(ctx);
			return (!broadcast && rc != (int)r->wr.coils.n_bits) ? MB_ERR_PROTO : MB_OK;
		}
		default:
			return MB_ERR_PARAM;
	}
}

// ------------------------ I/O 工作线程：总线串行化 + 重试/重连策略 ------------------------
// 该线程是“唯一”读写串口的地方，保证 RTU 帧不会被多线程并发破坏。
static void request_complete(mb_request_t *r, int status) {
	/*
	 * 完成通知：
	 * - worker 线程写入 r->status 并置 r->done=true，然后 signal 等待者。
	 * - 这里必须持有 r->m：防止等待线程在检查 done/读取 status 时发生竞态。
	 *
	 * 说明：
	 * - condvar 允许“伪唤醒”，等待侧要用 while(!done) 循环检查条件。
	 */
	// 通知等待线程：该请求已完成。
	pthread_mutex_lock(&r->m);
	r->status = status;
	r->done = true;
	pthread_cond_signal(&r->cv);
	pthread_mutex_unlock(&r->m);
}

static void ensure_bus_idle(mb_ctx_t *ctx) {
	/*
	 * 计算距离“上一次总线活动”是否已经超过 t3.5：
	 * - last_bus_activity_us 在每次请求完成（或至少发送/接收后）更新；
	 * - 若还未达到 t3.5，则 sleep 剩余时间。
	 *
	 * 这里用“微秒”计算避免浮点误差，且逻辑直观。
	 */
	// RTU 要求两帧之间至少有 t3.5 的静默间隔（用于判定帧边界）。
	uint64_t now = now_us();
	uint64_t need = ctx->last_bus_activity_us + (uint64_t)ctx->line.t35_us;
	if (need > now) sleep_us(need - now);
}

static void *io_thread_main(void *arg) {
	/*
	 * I/O 工作线程主循环：
	 * - 从队列取请求（queue_pop）；如果 shutdown 且队列空则退出线程。
	 * - 每个请求可按 retries 进行重试：
	 *   * TIMEOUT/CRC/PROTO 通常可重试（现场抖动、干扰导致）
	 *   * PARAM 属于编程错误（地址/长度不合法）不应重试
	 *   * IO 表示串口/驱动层错误，先尝试重连再继续
	 *
	 * 统计项（ctx->stat_*）用于长期运行观测：发包/收包/超时/CRC/协议错误等。
	 */
	mb_ctx_t *ctx = (mb_ctx_t *)arg;

	// 单 worker 线程对 RTU 总线进行串行化：保证同一时刻只有一个请求在路上。
	while (ctx->running) {
		mb_request_t *r = queue_pop(&ctx->queue, &ctx->running);
		if (!r) break;

		int final_rc = MB_ERR_SHUTDOWN;

		for (int attempt = 0; attempt <= r->retries; attempt++) {
			if (!ctx->mb) {
				if (mb_reconnect(ctx) != MB_OK) { final_rc = MB_ERR_IO; break; }
			}

			int rc = do_libmodbus_request(ctx, r);
			ctx->last_bus_activity_us = now_us();

			if (rc == MB_ERR_TIMEOUT) ctx->stat_timeouts++;
			if (rc == MB_ERR_CRC) ctx->stat_crc_err++;
			if (rc == MB_ERR_PROTO) ctx->stat_proto_err++;
			if (r->slave != 0 && (rc == MB_OK || rc == MB_ERR_EXCEPTION)) ctx->stat_rx++;

			if (rc == MB_ERR_IO) {
				LOGW("%s", "I/O error, attempting reconnect");
				if (mb_reconnect(ctx) == MB_OK) {
					final_rc = rc;
					continue;
				}
			}

			final_rc = rc;
			if (rc == MB_ERR_PARAM) break;
			if (rc == MB_OK || rc == MB_ERR_EXCEPTION) break;
		}

		request_complete(r, final_rc);
	}

	return NULL;
}

// ------------------------ 对外 API：提交请求/等待/常用读写封装 ------------------------
static int mb_create_io_thread(mb_ctx_t *ctx) {
	pthread_attr_t attr;
	bool attr_inited = false;
	pthread_attr_t *attr_ptr = NULL;
	const size_t page_size = 4096u;
	size_t stack_size = MB_IO_THREAD_STACK_SIZE;
	int rc = pthread_attr_init(&attr);
	if (rc == 0) {
		attr_inited = true;
		if (stack_size < PTHREAD_STACK_MIN) stack_size = PTHREAD_STACK_MIN;
		stack_size = ((stack_size + page_size - 1u) / page_size) * page_size;
		rc = pthread_attr_setstacksize(&attr, stack_size);
		if (rc == 0) {
			attr_ptr = &attr;
		} else {
			LOGW("io thread stack size fallback to default: %s", strerror(rc));
		}
	} else {
		LOGW("io thread attr init failed: %s", strerror(rc));
	}

	rc = pthread_create(&ctx->th, attr_ptr, io_thread_main, ctx);
	if (attr_inited) pthread_attr_destroy(&attr);
	return rc;
}

int mb_ctx_init(mb_ctx_t *ctx, mb_line_cfg_t line) {
	/*
	 * 初始化主站上下文：
	 * - 复制线路参数 line，并补齐默认值（例如 t15_min_us、broadcast_delay_us）
	 * - 计算 t1.5/t3.5
	 * - 打开串口（libmodbus connect）
	 * - 初始化队列、启动 I/O 线程
	 *
	 * 失败处理：
	 * - 任何一步失败都会释放已申请资源，保证调用者可以安全重试。
	 */
	memset(ctx, 0, sizeof(*ctx));
	ctx->mb = NULL;
	ctx->line = line;
	mb_apply_line_defaults(&ctx->line);
	calc_modbus_timers(&ctx->line);

	int rc = mb_open_modbus(ctx);
	if (rc != MB_OK) return rc;

	queue_init(&ctx->queue);
	ctx->running = true;
	ctx->last_bus_activity_us = now_us();

	rc = mb_create_io_thread(ctx);
	if (rc != 0) {
		LOGE("%s", "pthread_create failed");
		mb_close_modbus(ctx);
		ctx->running = false;
		queue_destroy(&ctx->queue);
		return MB_ERR_IO;
	}

	return MB_OK;
}

void mb_ctx_shutdown(mb_ctx_t *ctx) {
	/*
	 * 关闭上下文（优雅退出）：
	 * - 设置 running=false，并广播队列 condvar，唤醒所有可能阻塞的线程；
	 * - join worker 线程，确保不再访问串口/队列；
	 * - 关闭 libmodbus fd 并释放资源。
	 *
	 * 这是工业场景常见需求：程序需要可控退出/重启，而不是直接 kill -9。
	 */
	if (!ctx) return;
	pthread_mutex_lock(&ctx->queue.m);
	ctx->running = false;
	pthread_cond_broadcast(&ctx->queue.cv);
	pthread_mutex_unlock(&ctx->queue.m);

	if (ctx->th) pthread_join(ctx->th, NULL);
	mb_close_modbus(ctx);
	queue_destroy(&ctx->queue);
}

static void mb_request_init(mb_request_t *r) {
	// 入队前初始化每个请求的同步对象（mutex/condvar）与状态字段。
	pthread_mutex_init(&r->m, NULL);
	pthread_cond_init(&r->cv, NULL);
	r->done = false;
	r->status = MB_ERR_SHUTDOWN;
	r->exception_code = 0;
}

void mb_request_cleanup(mb_request_t *r) {
	// 调用者在等待完成后（或入队失败）必须清理同步对象，避免资源泄漏。
	pthread_cond_destroy(&r->cv);
	pthread_mutex_destroy(&r->m);
}

int mb_submit_async(mb_ctx_t *ctx, mb_request_t *r) {
	// 异步提交：只入队不等待；request 的生命周期由调用者负责管理。
	mb_request_init(r);
	int rc = queue_push(&ctx->queue, r, &ctx->running);
	if (rc != MB_OK) mb_request_cleanup(r);
	return rc;
}

int mb_request_wait(mb_request_t *r, int wait_ms) {
	/*
	 * 等待单个请求完成：
	 * - wait_ms < 0：无限等待
	 * - wait_ms >=0：使用 pthread_cond_timedwait 做超时等待
	 *
	 * 注意：这里使用 CLOCK_REALTIME 生成绝对时间点（pthread_cond_timedwait 接口要求）。
	 * - 如果系统时间被调整，可能影响 timedwait 的准确性；
	 * - 但这是 pthread_cond_timedwait 的典型用法，且等待的是“用户态超时”，影响可接受。
	 * - 若你需要完全抗校时，可改为 pthread_condattr_setclock 使用 CLOCK_MONOTONIC。
	 */
	// 等待请求完成：wait_ms<0 表示无限等待；否则超时返回 PENDING/IO 等状态。
	int rc = MB_OK;

	pthread_mutex_lock(&r->m);
	if (!r->done) {
		if (wait_ms < 0) {
			while (!r->done) pthread_cond_wait(&r->cv, &r->m);
		} else {
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_sec += wait_ms / 1000;
			ts.tv_nsec += (long)(wait_ms % 1000) * 1000000L;
			if (ts.tv_nsec >= 1000000000L) {
				ts.tv_sec += 1;
				ts.tv_nsec -= 1000000000L;
			}
			while (!r->done) {
				int err = pthread_cond_timedwait(&r->cv, &r->m, &ts);
				if (err == ETIMEDOUT) break;
				if (err != 0 && err != EINTR) { rc = MB_ERR_IO; break; }
			}
		}
	}

	if (rc == MB_OK) rc = r->done ? r->status : MB_ERR_PENDING;
	pthread_mutex_unlock(&r->m);
	return rc;
}

bool mb_request_is_done(mb_request_t *r, int *status, uint8_t *exc) {
	// 非阻塞查询：用于异步模式下轮询 done/status/exception。
	bool done;
	pthread_mutex_lock(&r->m);
	done = r->done;
	if (done) {
		if (status) *status = r->status;
		if (exc) *exc = r->exception_code;
	}
	pthread_mutex_unlock(&r->m);
	return done;
}

static int mb_submit_and_wait(mb_ctx_t *ctx, mb_request_t *r) {
	/*
	 * 同步封装：提交 + 等待 + 清理
	 * - 适用于最常见的“发一个 Modbus 请求，拿到结果再继续”的业务流程。
	 * - 内部会调用 mb_submit_async -> mb_request_wait -> mb_request_cleanup。
	 */
	// 同步调用：入队并阻塞等待直到完成。
	int rc = mb_submit_async(ctx, r);
	if (rc != MB_OK) return rc;
	rc = mb_request_wait(r, -1);
	mb_request_cleanup(r);
	return rc;
}

// ---- 常用高层操作封装（读/写寄存器与线圈） ----
int mb_read_holding(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t qty,
	/*
	 * 读保持寄存器（0x03）的便捷封装：
	 * - 负责把参数填到 mb_request_t；
	 * - out_regs/out_cap 用于承接结果并做容量保护；
	 * - timeout_ms/retries 让上层能按现场链路质量灵活调参；
	 * - exc 返回从站异常码（如果返回 MB_ERR_EXCEPTION）。
	 */
						   uint16_t *out_regs, uint16_t out_cap,
						   int timeout_ms, int retries, uint8_t *exc) {
	// 同步封装：填充 request 结构体后提交并等待，减少上层重复代码。
	mb_request_t r;
	memset(&r, 0, sizeof(r));
	r.slave = slave;
	r.func = MB_FC03_READ_HOLDING_REGS;
	r.addr = addr;
	r.qty = qty;
	r.out_regs = out_regs;
	r.out_regs_cap = out_cap;
	r.timeout_ms = timeout_ms;
	r.retries = retries;

	int rc = mb_submit_and_wait(ctx, &r);
	if (rc == MB_ERR_EXCEPTION && exc) *exc = r.exception_code;
	return rc;
}

int mb_read_input(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t qty,
						 uint16_t *out_regs, uint16_t out_cap,
						 int timeout_ms, int retries, uint8_t *exc) {
	mb_request_t r;
	memset(&r, 0, sizeof(r));
	r.slave = slave;
	r.func = MB_FC04_READ_INPUT_REGS;
	r.addr = addr;
	r.qty = qty;
	r.out_regs = out_regs;
	r.out_regs_cap = out_cap;
	r.timeout_ms = timeout_ms;
	r.retries = retries;

	int rc = mb_submit_and_wait(ctx, &r);
	if (rc == MB_ERR_EXCEPTION && exc) *exc = r.exception_code;
	return rc;
}

int mb_write_single_reg(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t val,
							   int timeout_ms, int retries, uint8_t *exc) {
	mb_request_t r;
	memset(&r, 0, sizeof(r));
	r.slave = slave;
	r.func = MB_FC06_WRITE_SINGLE_REG;
	r.addr = addr;
	r.wr_u16 = val;
	r.timeout_ms = timeout_ms;
	r.retries = retries;

	int rc = mb_submit_and_wait(ctx, &r);
	if (rc == MB_ERR_EXCEPTION && exc) *exc = r.exception_code;
	return rc;
}

int mb_write_multi_regs(mb_ctx_t *ctx, uint8_t slave, uint16_t addr,
							   const uint16_t *vals, uint16_t n,
							   int timeout_ms, int retries, uint8_t *exc) {
	// 多寄存器写：把 vals[] 拷贝进 request 的 union，交由 worker 线程发送。
	if (!vals || n == 0 || n > MB_MAX_WRITE_REGS) return MB_ERR_PARAM;

	mb_request_t r;
	memset(&r, 0, sizeof(r));
	r.slave = slave;
	r.func = MB_FC10_WRITE_MULTI_REGS;
	r.addr = addr;
	r.wr.regs.n = n;
	for (uint16_t i = 0; i < n; i++) r.wr.regs.vals[i] = vals[i];
	r.timeout_ms = timeout_ms;
	r.retries = retries;

	int rc = mb_submit_and_wait(ctx, &r);
	if (rc == MB_ERR_EXCEPTION && exc) *exc = r.exception_code;
	return rc;
}

int mb_read_coils(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t qty,
						 uint8_t *out_bits, uint16_t out_cap_bytes,
						 int timeout_ms, int retries, uint8_t *exc) {
	// Coil read returns packed bits (LSB-first), matching Modbus wire format.
	mb_request_t r;
	memset(&r, 0, sizeof(r));
	r.slave = slave;
	r.func = MB_FC01_READ_COILS;
	r.addr = addr;
	r.qty = qty;
	r.out_bits = out_bits;
	r.out_bits_cap_bytes = out_cap_bytes;
	r.timeout_ms = timeout_ms;
	r.retries = retries;

	int rc = mb_submit_and_wait(ctx, &r);
	if (rc == MB_ERR_EXCEPTION && exc) *exc = r.exception_code;
	return rc;
}

int mb_write_single_coil(mb_ctx_t *ctx, uint8_t slave, uint16_t addr, bool on,
								int timeout_ms, int retries, uint8_t *exc) {
	mb_request_t r;
	memset(&r, 0, sizeof(r));
	r.slave = slave;
	r.func = MB_FC05_WRITE_SINGLE_COIL;
	r.addr = addr;
	r.wr_u16 = on ? 1 : 0;
	r.timeout_ms = timeout_ms;
	r.retries = retries;

	int rc = mb_submit_and_wait(ctx, &r);
	if (rc == MB_ERR_EXCEPTION && exc) *exc = r.exception_code;
	return rc;
}

int mb_write_multi_coils(mb_ctx_t *ctx, uint8_t slave, uint16_t addr,
							   const uint8_t *packed_bits, uint16_t n_bits,
							   int timeout_ms, int retries, uint8_t *exc) {
	// 多线圈写：调用者传入按位打包后的数据（符合 Modbus 线格式）。
	if (!packed_bits || n_bits == 0 || n_bits > MB_MAX_WRITE_COILS) return MB_ERR_PARAM;

	mb_request_t r;
	memset(&r, 0, sizeof(r));
	r.slave = slave;
	r.func = MB_FC0F_WRITE_MULTI_COILS;
	r.addr = addr;
	r.wr.coils.n_bits = n_bits;
	uint16_t bc = (uint16_t)((n_bits + 7u) / 8u);
	memcpy(r.wr.coils.bytes, packed_bits, bc);
	r.timeout_ms = timeout_ms;
	r.retries = retries;

	int rc = mb_submit_and_wait(ctx, &r);
	if (rc == MB_ERR_EXCEPTION && exc) *exc = r.exception_code;
	return rc;
}

int mb_scan_slaves(mb_ctx_t *ctx, uint8_t start, uint8_t end, int timeout_ms) {
	/*
	 * 扫描从站地址（粗略探测在线设备）：
	 * - 从 start..end 依次尝试对每个地址做一个最小读（读 1 个 holding 寄存器）；
	 * - 只要收到“正常响应”或“从站异常响应”，都说明该地址有设备存在；
	 * - 如果遇到 MB_ERR_IO（串口层错误），立即中止扫描并返回错误，避免误判。
	 *
	 * 工业现场建议：
	 * - 扫描时 timeout 不宜太大，否则全网扫描耗时会爆炸；
	 * - 通常 50~200ms 足够（取决于波特率、总线长度、设备响应时间）。
	 */
	// 简易扫描：对每个地址做一次最小读操作，用于探测在线从站。
	if (start < 1) start = 1;
	if (end > 247) end = 247;
	if (start > end) return MB_ERR_PARAM;
	if (timeout_ms < 1) timeout_ms = 50;

	int found = 0;
	for (uint16_t id = start; id <= end; id++) {
		uint16_t reg = 0;
		uint8_t exc = 0;
		int rc = mb_read_holding(ctx, (uint8_t)id, 0, 1, &reg, 1, timeout_ms, 0, &exc);
		if (rc == MB_OK || rc == MB_ERR_EXCEPTION) {
			printf("found slave %u\n", (unsigned)id);
			found++;
		} else if (rc == MB_ERR_IO) {
			LOGE("scan stopped at slave %u: %s", (unsigned)id, mb_status_str(rc));
			return rc;
		}
	}

	LOGI("scan done: %d device(s) found", found);
	return MB_OK;
}


