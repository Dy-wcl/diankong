#ifndef TOOL_PROCESS_PROCESS_H_
#define TOOL_PROCESS_PROCESS_H_

#include <stdint.h>

/* ===== 自定义的小工具 ===== */
#ifdef __cplusplus
extern "C" {
#endif

/* ===== 私有宏定义 ===== */

/* ===== 私有宏定义 ===== */

// 角度转弧度
#define RAD_PER_DEG (0.017453292519943295f)

// 弧度转角度
#define DEG_PER_RAD (57.29577951308232f)

// PI定义
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 2PI定义
#ifndef M_2PI
#define M_2PI 6.28318530717958647693f
#endif

// PI/2定义
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

/**
 * @brief 数值限制宏
 * 使用语句表达式避免参数被多次求值的问题
 */
#define CONSTRAIN(x, min, max)                                                 \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    __typeof__(min) _min = (min);                                              \
    __typeof__(max) _max = (max);                                              \
    _x < _min ? _min : (_x > _max ? _max : _x);                                \
  })

/**
 * @brief 数值限制宏（指针版，直接修改数值）
 */
#define CONSTRAIN_PTR(ptr, min, max)                                           \
  do {                                                                         \
    __typeof__(*(ptr)) _val = *(ptr);                                          \
    *(ptr) = _val < (min) ? (min) : (_val > (max) ? (max) : _val);             \
  } while (0)

/**
 * @brief 绝对值宏
 * 使用语句表达式避免参数被多次求值的问题
 */
#define ABS(x)                                                                 \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    _x < 0 ? -_x : _x;                                                         \
  })

/**
 * @brief 绝对值宏（指针版）
 */
#define ABS_PTR(ptr)                                                           \
  do {                                                                         \
    __typeof__(*(ptr)) _val = *(ptr);                                          \
    *(ptr) = _val < 0 ? -_val : _val;                                          \
  } while (0)

/**
 * @brief 返回两个值中的最大值
 * 使用语句表达式避免参数被多次求值的问题
 */
#define MAX(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a > _b ? _a : _b;                                                         \
  })

/**
 * @brief 返回两个值中的最小值
 * 使用语句表达式避免参数被多次求值的问题
 */
#define MIN(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a < _b ? _a : _b;                                                         \
  })

/**
 * @brief 三数最小值
 */
#define MIN3(a, b, c) MIN(MIN((a), (b)), (c))

/**
 * @brief 三数最大值
 */
#define MAX3(a, b, c) MAX(MAX((a), (b)), (c))

/**
 * @brief 限制范围在0-1
 */
#define CONSTRAIN_01(x) CONSTRAIN((x), 0.0f, 1.0f)

/**
 * @brief 限制范围在-1到1
 */
#define CONSTRAIN_PM1(x) CONSTRAIN((x), -1.0f, 1.0f)

/**
 * @brief 限制范围在-PI到PI
 */
#define CONSTRAIN_PI(x) CONSTRAIN((x), -M_PI, M_PI)

/**
 * @brief 死区宏，返回0如果绝对值小于阈值
 * 使用语句表达式避免参数被多次求值的问题
 */
#define DEADZONE(x, threshold)                                                 \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    __typeof__(threshold) _th = (threshold);                                   \
    ABS(_x) < _th ? 0.0f : _x;                                                 \
  })

/**
 * @brief 死区宏（指针版）
 */
#define DEADZONE_PTR(ptr, threshold)                                           \
  do {                                                                         \
    __typeof__(*(ptr)) _val = *(ptr);                                          \
    __typeof__(threshold) _th = (threshold);                                   \
    *(ptr) = ABS(_val) < _th ? 0.0f : _val;                                    \
  } while (0)

/**
 * @brief 符号函数，返回-1、0或1
 * 使用语句表达式避免参数被多次求值的问题
 */
#define SIGN(x)                                                                \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    _x > 0 ? 1 : (_x < 0 ? -1 : 0);                                            \
  })

/**
 * @brief 线性插值
 * 使用语句表达式避免参数被多次求值的问题
 */
#define LERP(a, b, t)                                                          \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    __typeof__(t) _t = (t);                                                    \
    _a + (_b - _a) * _t;                                                       \
  })

/**
 * @brief 线性插值（限制t在0-1）
 */
#define LERP_CLAMP(a, b, t) ((a) + ((b) - (a)) * CONSTRAIN_01(t))

// 数组元素数量
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// 向上取整整除
#define DIV_CEIL(a, b) (((a) + (b) - 1) / (b))

// 字节数转为字数（32位）
#define BYTES_TO_WORDS(bytes) DIV_CEIL((bytes), 4)

/**
 * @brief 四舍五入
 * 使用语句表达式避免参数被多次求值的问题
 */
#define ROUND(x)                                                               \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    (int)(_x + 0.5f);                                                          \
  })

/**
 * @brief 交换两个值（需temp变量）
 */
#define SWAP(a, b, temp)                                                       \
  do {                                                                         \
    (temp) = (a);                                                              \
    (a) = (b);                                                                 \
    (b) = (temp);                                                              \
  } while (0)

// // 设置位
// #define SET_BIT(reg, bit) ((reg) |= (1U << (bit)))

// // 清除位
// #define CLEAR_BIT(reg, bit) ((reg) &= ~(1U << (bit)))

// // 翻转位
// #define TOGGLE_BIT(reg, bit) ((reg) ^= (1U << (bit)))

// // 读取位
// #define READ_BIT(reg, bit) (((reg) >> (bit)) & 1U)

// 设置多个位
#define SET_BITS(reg, bits) ((reg) |= (bits))

// 清除多个位
#define CLEAR_BITS(reg, bits) ((reg) &= ~(bits))

// 检查位是否设置
#define BIT_IS_SET(reg, bit) ((reg) & (1U << (bit)))

// 检查位是否清除
#define BIT_IS_CLEARED(reg, bit) (!BIT_IS_SET((reg), (bit)))

// 判断范围是否合法（左闭右开）
#define IS_IN_RANGE(x, min, max) ((x) >= (min) && (x) < (max))

// 判断范围是否合法（双闭区间）
#define IS_IN_RANGE_INCLUSIVE(x, min, max) ((x) >= (min) && (x) <= (max))

/**
 * @brief 映射一个值从一个范围到另一个范围
 * 使用语句表达式避免参数被多次求值的问题
 */
#define MAP(x, in_min, in_max, out_min, out_max)                               \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    __typeof__(in_min) _in_min = (in_min);                                     \
    __typeof__(in_max) _in_max = (in_max);                                     \
    __typeof__(out_min) _out_min = (out_min);                                  \
    __typeof__(out_max) _out_max = (out_max);                                  \
    ((_x - _in_min) * (_out_max - _out_min) / (_in_max - _in_min) + _out_min); \
  })

/**
 * @brief 映射并限制到目标范围
 */
#define MAP_CLAMP(x, in_min, in_max, out_min, out_max)                         \
  CONSTRAIN(MAP((x), (in_min), (in_max), (out_min), (out_max)), (out_min),     \
            (out_max))

/**
 * @brief 度数平方
 * 使用语句表达式避免参数被多次求值的问题
 */
#define SQUARE(x)                                                              \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    _x *_x;                                                                    \
  })

/**
 * @brief 度数立方
 * 使用语句表达式避免参数被多次求值的问题
 */
#define CUBE(x)                                                                \
  ({                                                                           \
    __typeof__(x) _x = (x);                                                    \
    _x *_x *_x;                                                                \
  })

/**
 * @brief 检查符号是否相同
 * 使用语句表达式避免参数被多次求值的问题
 */
#define SAME_SIGN(a, b)                                                        \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    ((_a > 0 && _b > 0) || (_a < 0 && _b < 0) || (_a == 0 && _b == 0));        \
  })

/**
 * @brief 检查符号是否不同
 */
#define DIFFERENT_SIGN(a, b) (!SAME_SIGN((a), (b)))

/**
 * @brief 饱和加法（避免溢出）
 */
#define SATURATE_ADD(a, b, max) MIN((a) + (b), (max))

/**
 * @brief 饱和减法
 */
#define SATURATE_SUB(a, b, min) MAX((a) - (b), (min))

// 判断是否为偶数
#define IS_EVEN(x) (((x) & 1U) == 0)

// 判断是否为奇数
#define IS_ODD(x) (((x) & 1U) != 0)

// 对齐到下一个N的倍数
#define ALIGN_UP(x, n) (((x) + ((n) - 1)) & ~((n) - 1))

// 对齐到下一个N的倍数（必须为2的幂）
#define ALIGN_UP_POW2(x, n) (((x) + ((n) - 1)) & ~((n) - 1))

// 判断是否为2的幂
#define IS_POWER_OF_TWO(x) (((x) & ((x) - 1)) == 0 && (x) != 0)

// 获取最近的2的幂
#define NEAREST_POWER_OF_TWO(x)                                                \
  (1U << (32 - __builtin_clz(x) - (IS_POWER_OF_TWO(x) ? 1 : 0)))

// 延时宏（单位：循环次数，仅供调试）
#define DELAY_LOOPS(loops)                                                     \
  do {                                                                         \
    volatile uint32_t _i = (loops);                                            \
    while (_i--)                                                               \
      __NOP();                                                                 \
  } while (0)

// 压入栈（假设有指针和大小）
#define PUSH_STACK(ptr, val, size, idx)                                        \
  do {                                                                         \
    if ((idx) < (size))                                                        \
      (ptr)[(idx)++] = (val);                                                  \
  } while (0)

// 弹出栈
#define POP_STACK(ptr, size, idx) ((idx) > 0 ? (ptr)[--(idx)] : 0)

// 获取结构体成员偏移量
#define OFFSET_OF(type, member) ((size_t)&(((type *)0)->member))

// // 获取包含某成员的结构体指针
// #define CONTAINER_OF(ptr, type, member) 
//   ((type *)((char *)(ptr) - OFFSET_OF(type, member)))

// 检查指针是否对齐
#define IS_ALIGNED(ptr, alignment) (((uintptr_t)(ptr) & ((alignment) - 1)) == 0)

// 获取高8位
#define HIGH_BYTE(x) (((x) >> 8) & 0xFF)

// 获取低8位
#define LOW_BYTE(x) ((x) & 0xFF)

// 合并高低8位
#define MAKE_WORD(h, l) (((uint16_t)(h) << 8) | (uint8_t)(l))

// 获取高16位
#define HIGH_WORD(x) (((x) >> 16) & 0xFFFF)

// 获取低16位
#define LOW_WORD(x) ((x) & 0xFFFF)

// 合并高低16位
#define MAKE_DWORD(h, l) (((uint32_t)(h) << 16) | (uint16_t)(l))

/**
 * @brief 判断是否在容忍范围内
 * 使用语句表达式避免参数被多次求值的问题
 */
#define IS_CLOSE(a, b, tolerance)                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    __typeof__(tolerance) _tol = (tolerance);                                  \
    ABS(_a - _b) < _tol;                                                       \
  })

/**
 * @brief 判断是否相等（容忍浮点误差）
 */
#define IS_EQUAL_FLOAT(a, b) IS_CLOSE((a), (b), 0.0001f)

#ifdef __cplusplus
}
#endif

#endif /* TOOL_PROCESS_PROCESS_H_ */
