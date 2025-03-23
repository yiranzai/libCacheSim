//
//  Clock (时钟) 缓存替换算法实现
//  与FIFO-Reinsertion或Second Chance相同，是一种FIFO变体
//  它在驱逐时会将某些对象重新插入回缓存
//
//  Clock算法特点：
//  - 维护一个环形队列，使用一个指针（时钟指针）指向最老的对象
//  - 每个对象有一个引用位(reference bit)，初始为0
//  - 当对象被访问时，引用位设置为1
//  - 当需要淘汰对象时，检查时钟指针指向的对象：
//    * 如果引用位为0，则淘汰该对象
//    * 如果引用位为1，则将引用位重置为0，指针移动到下一个对象，重复此过程
//  - 时间复杂度：查找O(1)，更新O(1)，淘汰最坏情况O(n)
//  - 优点：比LRU实现简单，性能接近LRU
//  - 缺点：不考虑访问频率，只考虑最近是否被访问
//
//  本实现支持多位计数器(n-bit-counter)，可以记录更多的访问历史
//
//  Clock.c
//  libCacheSim
//
//  Created by Juncheng on 12/4/18.
//  Copyright © 2018 Juncheng. All rights reserved.
//

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../include/libCacheSim/evictionAlgo.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define USE_BELADY
#undef USE_BELADY

// 默认参数：初始频率为0，计数器位数为1
static const char *DEFAULT_PARAMS = "init-freq=0,n-bit-counter=1";

// ***********************************************************************
// ****                                                               ****
// ****                   函数声明部分                                 ****
// ****                                                               ****
// ***********************************************************************

static void Clock_parse_params(cache_t *cache, const char *cache_specific_params);
static void Clock_free(cache_t *cache);
static bool Clock_get(cache_t *cache, const request_t *req);
static cache_obj_t *Clock_find(cache_t *cache, const request_t *req, const bool update_cache);
static cache_obj_t *Clock_insert(cache_t *cache, const request_t *req);
static cache_obj_t *Clock_to_evict(cache_t *cache, const request_t *req);
static void Clock_evict(cache_t *cache, const request_t *req);
static bool Clock_remove(cache_t *cache, const obj_id_t obj_id);

// ***********************************************************************
// ****                                                               ****
// ****                   面向用户的函数接口                           ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 初始化一个Clock缓存
 *
 * @param ccache_params 通用缓存参数
 * @param cache_specific_params Clock特定参数字符串
 */
cache_t *Clock_init(const common_cache_params_t ccache_params, const char *cache_specific_params) {
  cache_t *cache = cache_struct_init("Clock", ccache_params, cache_specific_params);
  cache->cache_init = Clock_init;
  cache->cache_free = Clock_free;
  cache->get = Clock_get;
  cache->find = Clock_find;
  cache->insert = Clock_insert;
  cache->evict = Clock_evict;
  cache->remove = Clock_remove;
  cache->can_insert = cache_can_insert_default;
  cache->get_n_obj = cache_get_n_obj_default;
  cache->get_occupied_byte = cache_get_occupied_byte_default;
  cache->to_evict = Clock_to_evict;
  cache->obj_md_size = 0;

#ifdef USE_BELADY
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "Clock_Belady");
#endif

  cache->eviction_params = malloc(sizeof(Clock_params_t));
  memset(cache->eviction_params, 0, sizeof(Clock_params_t));
  Clock_params_t *params = (Clock_params_t *)cache->eviction_params;
  params->q_head = NULL;
  params->q_tail = NULL;
  params->n_bit_counter = 1;  // 默认使用1位计数器
  params->max_freq = 1;       // 最大频率值，由n_bit_counter决定

  // 解析默认参数
  Clock_parse_params(cache, DEFAULT_PARAMS);
  if (cache_specific_params != NULL) {
    Clock_parse_params(cache, cache_specific_params);
  }

  // 如果不是默认的1位计数器，则在缓存名称中体现
  if (params->n_bit_counter != 1) {
    snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "Clock-%d-%d", params->n_bit_counter, params->init_freq);
  }

  return cache;
}

/**
 * 释放此缓存使用的资源
 *
 * @param cache 要释放的缓存
 */
static void Clock_free(cache_t *cache) {
  free(cache->eviction_params);
  cache_struct_free(cache);
}

/**
 * @brief 这个函数是面向用户的API
 * 它执行以下逻辑：
 *
 * ```
 * 如果对象在缓存中:
 *    更新元数据
 *    返回true
 * 否则:
 *    如果缓存没有足够空间:
 *        驱逐直到有空间插入
 *    插入对象
 *    返回false
 * ```
 *
 * @param cache 缓存
 * @param req 请求
 * @return 如果缓存命中返回true，否则返回false
 */
static bool Clock_get(cache_t *cache, const request_t *req) { return cache_get_base(cache, req); }

// ***********************************************************************
// ****                                                               ****
// ****       面向开发者的API（由缓存开发者使用）                       ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 检查对象是否在缓存中
 *
 * @param cache 缓存
 * @param req 请求
 * @param update_cache 是否更新缓存,
 *  如果为true，对象会被提升
 *  如果对象已过期，会从缓存中移除
 * @return 命中返回true，未命中返回false
 */
static cache_obj_t *Clock_find(cache_t *cache, const request_t *req, const bool update_cache) {
  Clock_params_t *params = (Clock_params_t *)cache->eviction_params;
  cache_obj_t *obj = cache_find_base(cache, req, update_cache);
  if (obj != NULL && update_cache) {
    // 如果对象被访问且需要更新缓存，增加其频率计数（但不超过最大值）
    if (obj->clock.freq < params->max_freq) {
      obj->clock.freq += 1;
    }
#ifdef USE_BELADY
    obj->next_access_vtime = req->next_access_vtime;
#endif
  }

  return obj;
}

/**
 * @brief 将对象插入缓存，
 * 更新哈希表和缓存元数据
 * 此函数假设缓存有足够空间
 * 驱逐不是此函数的一部分
 *
 * @param cache 缓存
 * @param req 请求
 * @return 插入的对象
 */
static cache_obj_t *Clock_insert(cache_t *cache, const request_t *req) {
  Clock_params_t *params = (Clock_params_t *)cache->eviction_params;

  cache_obj_t *obj = cache_insert_base(cache, req);
  prepend_obj_to_head(&params->q_head, &params->q_tail, obj);

  // 设置初始频率
  obj->clock.freq = params->init_freq;
#ifdef USE_BELADY
  obj->next_access_vtime = req->next_access_vtime;
#endif

  return obj;
}

/**
 * @brief 找到要驱逐的对象
 * 此函数不会实际驱逐对象或更新元数据
 * 并非所有驱逐算法都支持此函数
 * 因为驱逐逻辑可能无法与查找驱逐候选对象分离
 * 如果无法支持此函数，请使用assert(false)
 *
 * @param cache 缓存
 * @return 要驱逐的对象
 */
static cache_obj_t *Clock_to_evict(cache_t *cache, const request_t *req) {
  Clock_params_t *params = (Clock_params_t *)cache->eviction_params;

  int n_round = 0;
  cache_obj_t *obj_to_evict = params->q_tail;
#ifdef USE_BELADY
  // 如果使用Belady，选择下一次访问时间最远的对象
  while (obj_to_evict->next_access_vtime != INT64_MAX) {
#else
  // 标准Clock算法：遍历队列，找到freq值小于当前轮次的对象
  while (obj_to_evict->clock.freq - n_round >= 1) {
#endif
    obj_to_evict = obj_to_evict->queue.prev;
    if (obj_to_evict == NULL) {
      obj_to_evict = params->q_tail;
      n_round += 1;  // 完成一轮遍历，增加轮次计数
    }
  }

  return obj_to_evict;
}

/**
 * @brief 从缓存中驱逐一个对象
 * 在返回前需要调用cache_evict_base
 * 它会更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存
 * @param req 未使用
 */
static void Clock_evict(cache_t *cache, const request_t *req) {
  Clock_params_t *params = (Clock_params_t *)cache->eviction_params;

  cache_obj_t *obj_to_evict = params->q_tail;
  // Clock算法核心：如果对象的freq>=1，减少其频率并移到队列头部
  while (obj_to_evict->clock.freq >= 1) {
    obj_to_evict->clock.freq -= 1;
    params->n_obj_rewritten += 1;
    params->n_byte_rewritten += obj_to_evict->obj_size;
    move_obj_to_head(&params->q_head, &params->q_tail, obj_to_evict);
    obj_to_evict = params->q_tail;
  }

  // 找到freq为0的对象，将其从队列中移除并驱逐
  remove_obj_from_list(&params->q_head, &params->q_tail, obj_to_evict);
  cache_evict_base(cache, obj_to_evict, true);
}

/**
 * @brief 从缓存中移除给定对象
 * 注意驱逐不应调用此函数，而应调用
 * `cache_evict_base`，因为我们在驱逐期间跟踪额外的元数据
 *
 * 此函数与驱逐不同
 * 因为它用于用户触发的
 * 移除，而驱逐是由缓存用来为新对象腾出空间
 *
 * 在返回前需要调用cache_remove_obj_base
 * 它会更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存
 * @param obj 要移除的对象
 */
static void Clock_remove_obj(cache_t *cache, cache_obj_t *obj) {
  Clock_params_t *params = (Clock_params_t *)cache->eviction_params;

  DEBUG_ASSERT(obj != NULL);
  remove_obj_from_list(&params->q_head, &params->q_tail, obj);
  cache_remove_obj_base(cache, obj, true);
}

/**
 * @brief 从缓存中移除一个对象
 * 这与cache_evict不同，因为它用于用户触发的
 * 移除，而驱逐是由缓存用来为新对象腾出空间
 *
 * 在返回前需要调用cache_remove_obj_base
 * 它会更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存
 * @param obj_id 对象ID
 * @return 如果对象被移除返回true，如果对象不在缓存中返回false
 */
static bool Clock_remove(cache_t *cache, const obj_id_t obj_id) {
  cache_obj_t *obj = hashtable_find_obj_id(cache->hashtable, obj_id);
  if (obj == NULL) {
    return false;
  }

  Clock_remove_obj(cache, obj);

  return true;
}

// ***********************************************************************
// ****                                                               ****
// ****                  参数设置函数                                  ****
// ****                                                               ****
// ***********************************************************************
/**
 * @brief 获取当前缓存参数的字符串表示
 * 
 * @param cache 缓存
 * @param params 参数结构体
 * @return 参数字符串
 */
static const char *Clock_current_params(cache_t *cache, Clock_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "n-bit-counter=%d\n", params->n_bit_counter);

  return params_str;
}

/**
 * @brief 解析缓存特定参数
 * 
 * @param cache 缓存
 * @param cache_specific_params 参数字符串
 */
static void Clock_parse_params(cache_t *cache, const char *cache_specific_params) {
  Clock_params_t *params = (Clock_params_t *)cache->eviction_params;
  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;
  char *end;

  while (params_str != NULL && params_str[0] != '\0') {
    /* 不同参数由逗号分隔，
     * 键和值由等号分隔 */
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    // 跳过空格
    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    if (strcasecmp(key, "n-bit-counter") == 0) {
      // 设置计数器位数
      params->n_bit_counter = (int)strtol(value, &end, 0);
      params->max_freq = (1 << params->n_bit_counter) - 1;  // 计算最大频率值
      if (strlen(end) > 2) {
        ERROR("参数解析错误，在数字后发现字符串 \"%s\"\n", end);
      }
    } else if (strcasecmp(key, "init-freq") == 0) {
      // 设置初始频率
      params->init_freq = (int)strtol(value, &end, 0);
      if (strlen(end) > 2) {
        ERROR("参数解析错误，在数字后发现字符串 \"%s\"\n", end);
      }
    } else if (strcasecmp(key, "print") == 0) {
      // 打印当前参数并退出
      printf("当前参数: %s\n", Clock_current_params(cache, params));
      exit(0);
    } else {
      ERROR("%s 没有参数 %s，示例参数 %s\n", cache->cache_name, key,
            Clock_current_params(cache, params));
      exit(1);
    }
  }
  free(old_params_str);
}

#ifdef __cplusplus
}
#endif
