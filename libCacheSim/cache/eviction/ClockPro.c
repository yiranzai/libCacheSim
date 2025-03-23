//
// ClockPro replacement algorithm
// https://www.usenix.org/legacy/event/usenix05/tech/general/full_papers/jiang/jiang.pdf
//
// Inspirations are taken from
// https://blog.yufeng.info/wp-content/uploads/2010/08/8-Clock-Pro.pdf
//
// compared with https://bitbucket.org/SamiLehtinen/pyclockpro/src/master/ using --ignore-obj-size
// using cloudPhysicsIO as traces
//
//    Size	      This Implementation	PyClockPro
//   ======	    =======================	==========
//    4897	            0.8363	          0.7420
//    9794	            0.7662	          0.7076
//    14692	            0.6435	          0.6214
//    19589	            0.5670	          0.5848
//    24487	            0.5092	          0.5654
//    29384	            0.4955	          0.5653
//    34281	            0.4726	          0.5646
//    39179	            0.4574	          0.5049
//    44076	            0.4384	          0.4302
//    48974	            0.4301	          0.4301
//
// one thing to note is the difference in the clock hand movement (this implementation vs PyClockPro)
// this implementation checks the object pointed by the hand first before moving the hand (as per the material in blog.yufeng.info)
// PyClockPro implementation moves the hand first before checking the object pointed by the hand
//
// libCacheSim
//
// Created by Marthen on 2/12/25.
// Copyright © 2025 Marthen. All rights reserved.
//

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../include/libCacheSim/evictionAlgo.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define USE_BELADY
#undef USE_BELADY

// ClockPro算法参数结构体
typedef struct ClockPro_params {
  cache_obj_t *hand_hot;   // 指向热数据区域的时钟指针
  cache_obj_t *hand_cold;  // 指向冷数据区域的时钟指针
  cache_obj_t *hand_test;  // 指向测试区域的时钟指针

  int64_t mem_cold_max;    // 冷数据区域的最大大小
  int64_t mem_cold;        // 当前冷数据区域的大小
  int64_t mem_test;        // 当前测试区域的大小
  int64_t mem_hot;         // 当前热数据区域的大小

  hashtable_t *ht_test;    // 测试区域的哈希表

  bool init_ref;           // 初始引用标志
} ClockPro_params_t;

// 默认参数设置
static const char *DEFAULT_PARAMS = "init-ref=0,init-ratio-cold=1";

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************

static void ClockPro_parse_params(cache_t *cache, const char *cache_specific_params);
static void ClockPro_free(cache_t *cache);
static bool ClockPro_get(cache_t *cache, const request_t *req);
static cache_obj_t *ClockPro_find(cache_t *cache, const request_t *req, bool update_cache);
static cache_obj_t *ClockPro_insert(cache_t *cache, const request_t *req);
static void ClockPro_evict(cache_t *cache, const request_t *req);
static bool ClockPro_remove(cache_t *cache, obj_id_t obj_id);
static bool ClockPro_can_insert(cache_t *cache, const request_t *req);
static void ClockPro_promote(cache_t *cache, cache_obj_t *obj);
static void ClockPro_run_test(cache_t *cache);
static void ClockPro_run_cold(cache_t *cache);
static void ClockPro_run_hot(cache_t *cache);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ***********************************************************************

/**
* @brief 初始化ClockPro缓存
*
* @param ccache_params 通用缓存参数
* @param cache_specific_params ClockPro特定参数字符串
* @return 初始化的缓存对象
*/
cache_t *ClockPro_init(const common_cache_params_t ccache_params, const char *cache_specific_params) {
  cache_t *cache = cache_struct_init("ClockPro", ccache_params, cache_specific_params);
  cache->cache_init = ClockPro_init;
  cache->cache_free = ClockPro_free;
  cache->get = ClockPro_get;
  cache->find = ClockPro_find;
  cache->insert = ClockPro_insert;
  cache->evict = ClockPro_evict;
  cache->remove = ClockPro_remove;
  cache->can_insert = ClockPro_can_insert;
  cache->get_n_obj = cache_get_n_obj_default;
  cache->get_occupied_byte = cache_get_occupied_byte_default;
  cache->obj_md_size = 0;

  cache->eviction_params = my_malloc_n(ClockPro_params_t, 1);
  ClockPro_params_t *params = (ClockPro_params_t *)(cache->eviction_params);

  // 初始化参数
  params->hand_hot = NULL;
  params->hand_cold = NULL;
  params->hand_test = NULL;
  params->mem_cold = 0;
  params->mem_test = 0;
  params->mem_hot = 0;
  params->mem_cold_max = cache->cache_size; // 默认为缓存大小
  params->ht_test = create_hashtable(HASH_POWER_DEFAULT);

  // 解析参数
  ClockPro_parse_params(cache, DEFAULT_PARAMS);
  if (cache_specific_params != NULL) {
    ClockPro_parse_params(cache, cache_specific_params);
  }

  return cache;
};

/**
 * 释放缓存使用的资源
 *
 * @param cache 缓存对象
 */
static void ClockPro_free(cache_t *cache) {
  ClockPro_params_t *params = (ClockPro_params_t *)(cache->eviction_params);
  free_hashtable(params->ht_test);
  my_free(sizeof(ClockPro_params_t), params);
  cache_struct_free(cache);
}

/**
 * @brief 用户面向的API函数
 * 执行以下逻辑:
 *
 * ```
 * if obj in cache:
 *    update_metadata
 *    return true
 * else:
 *    if cache does not have enough space:
 *        evict until it has space to insert
 *    insert the object
 *    return false
 * ```
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果对象在缓存中返回true，否则返回false
 */
static bool ClockPro_get(cache_t *cache, const request_t *req) {
  return cache_get_base(cache, req);
}

// ***********************************************************************
// ****                                                               ****
// ****       developer facing APIs (used by cache developer)         ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 检查对象是否在缓存中
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @param update_cache 是否更新缓存
 * 如果为true，对象会被提升或标记为已引用
 * 如果对象已过期，会从缓存中移除
 * @return 命中返回对象指针，未命中返回NULL
 */
static cache_obj_t *ClockPro_find(cache_t *cache, const request_t *req, const bool update_cache) {
  cache_obj_t *obj = cache_find_base(cache, req, update_cache);

  if (obj != NULL && update_cache) {
    if (!obj->clockpro.referenced) {
      obj->clockpro.referenced = true;  // 标记对象为已引用
    }
  }

  return obj;
}

/**
 * @brief 将对象插入缓存
 * 更新哈希表和缓存元数据
 * 此函数假设缓存有足够空间
 * 驱逐不是此函数的一部分
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 插入的对象
 */
static cache_obj_t *ClockPro_insert(cache_t *cache, const request_t *req) {
  ClockPro_params_t *params = (ClockPro_params_t *)cache->eviction_params;

  // 检查是否为测试对象的请求
  cache_obj_t *test_obj = hashtable_find_obj_id(params->ht_test, req->obj_id);
  if (test_obj != NULL) {
    ClockPro_promote(cache, test_obj);  // 提升测试对象
    return test_obj;
  }

  // 插入新对象
  cache_obj_t *obj = cache_insert_base(cache, req);
  obj->clockpro.referenced = params->init_ref;
  obj->clockpro.status = CLOCKPRO_COLD;  // 新对象初始为冷对象

  // 处理链表插入
  if (params->hand_hot == NULL) { // 首次插入
    prepend_obj_to_head(&params->hand_hot, &params->hand_hot, obj);
    params->hand_hot->queue.next = params->hand_hot;
    params->hand_hot->queue.prev = params->hand_hot;
    params->hand_cold = params->hand_hot;
    params->hand_test = params->hand_hot;
  } else {
    cache_obj_t *hand_hot_prev = params->hand_hot->queue.prev;
    prepend_obj_to_head(&params->hand_hot, &hand_hot_prev, obj);
    obj->queue.prev = hand_hot_prev;
    obj->queue.prev->queue.next = obj;
    params->hand_hot = obj->queue.next;
  }

  params->mem_cold += obj->obj_size;  // 更新冷区大小

  return obj;
}

/**
 * @brief 从缓存中驱逐对象
 * 在返回前需要调用cache_evict_base
 * 以更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param req 未使用
 */
static void ClockPro_evict(cache_t *cache, const request_t *req) {
  ClockPro_run_cold(cache);  // 从冷区驱逐对象
}

/**
 * @brief 从缓存中移除指定对象
 * 注意驱逐不应调用此函数，而应调用`cache_evict_base`
 * 因为我们在驱逐期间跟踪额外的元数据
 *
 * 此函数与驱逐不同，因为它用于用户触发的移除
 * 而驱逐是由缓存用来为新对象腾出空间
 *
 * 在返回前需要调用cache_remove_obj_base
 * 以更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param obj 要移除的对象
 */
static void ClockPro_remove_obj(cache_t *cache, cache_obj_t *obj) {
  ClockPro_params_t *params = (ClockPro_params_t *)cache->eviction_params;

  DEBUG_ASSERT(obj != NULL);
  cache_obj_t *hand_hot_prev = params->hand_hot->queue.prev;

  // 根据对象状态更新相应区域大小
  if (obj->clockpro.status == CLOCKPRO_TEST) {
    params->mem_test -= obj->obj_size;
  } else if (obj->clockpro.status == CLOCKPRO_COLD) {
    params->mem_cold -= obj->obj_size;
  } else if (obj->clockpro.status == CLOCKPRO_HOT) {
    params->mem_hot -= obj->obj_size;
  }

  // 更新时钟指针，如果指向被移除的对象
  if (params->hand_test == obj) {
    params->hand_test = obj->queue.next;
  }
  if (params->hand_cold == obj) {
    params->hand_cold = obj->queue.next;
  }
  if (params->hand_hot == obj) {
    params->hand_hot = obj->queue.next;
  }

  // 从链表中移除对象
  remove_obj_from_list(&params->hand_hot, &hand_hot_prev, obj);
  cache_remove_obj_base(cache, obj, true);
}

/**
 * @brief 从缓存中移除对象
 * 这与cache_evict不同，因为它用于用户触发的移除
 * 而驱逐是由缓存用来为新对象腾出空间
 *
 * 在返回前需要调用cache_remove_obj_base
 * 以更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param obj_id 要移除的对象ID
 * @return 如果对象被移除返回true，如果对象不在缓存中返回false
 */
static bool ClockPro_remove(cache_t *cache, const obj_id_t obj_id) {
  cache_obj_t *obj = hashtable_find_obj_id(cache->hashtable, obj_id);
  if (obj == NULL) {
    return false;
  }

  ClockPro_remove_obj(cache, obj);

  return true;
}

/**
 * @brief 检查是否可以插入对象
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果可以插入返回true，否则返回false
 */
static bool ClockPro_can_insert(cache_t *cache, const request_t *req) {
  ClockPro_params_t *params = (ClockPro_params_t *)cache->eviction_params;
  return cache_can_insert_default(cache, req) && (params->mem_cold + req->obj_size <= params->mem_cold_max);
}

/**
 * @brief 处理测试区域的时钟扫描
 * 
 * @param cache 缓存对象
 */
static void ClockPro_run_test(cache_t *cache) {
  ClockPro_params_t *params = (ClockPro_params_t *)cache->eviction_params;
  cache_obj_t *obj = params->hand_test;

  // 如果不是测试对象，移动指针并返回
  if (obj->clockpro.status != CLOCKPRO_TEST) {
    params->hand_test = obj->queue.next;
    return;
  }

  // 更新测试区大小
  params->mem_test -= obj->obj_size;

  // 更新冷区最大大小
  if (params->mem_cold_max > obj->obj_size) {
    params->mem_cold_max -= obj->obj_size;
  } else {
    params->mem_cold_max = 0;
  }

  // 更新时钟指针，如果指向被移除的对象
  if (params->hand_hot == obj) {
    params->hand_hot = obj->queue.next;
  }
  if (params->hand_cold == obj) {
    params->hand_cold = obj->queue.next;
  }

  // 从链表中移除对象
  cache_obj_t *hand_test_prev = params->hand_test->queue.prev;
  remove_obj_from_list(&params->hand_test, &hand_test_prev, obj);
  hashtable_delete(params->ht_test, obj);

  // 确保冷区大小不超过最大值
  while (params->mem_cold > params->mem_cold_max) {
    ClockPro_run_cold(cache);
  }
}

/**
 * @brief 处理冷区的时钟扫描
 * 
 * @param cache 缓存对象
 */
static void ClockPro_run_cold(cache_t *cache) {
  ClockPro_params_t *params = (ClockPro_params_t *)cache->eviction_params;
  cache_obj_t *obj = params->hand_cold;

  // 如果不是冷对象，移动指针并返回
  if (obj->clockpro.status != CLOCKPRO_COLD) {
    params->hand_cold = obj->queue.next;
    return;
  }

  // 如果对象被引用，提升为热对象
  if (obj->clockpro.referenced) {
    ClockPro_promote(cache, obj);
    return;
  }

  // 更新冷区大小
  params->mem_cold -= obj->obj_size;

  // 确保测试区有足够空间
  while (params->mem_test + obj->obj_size > cache->cache_size) {
    ClockPro_run_test(cache);
  }

  // 将冷对象降级为测试对象
  request_t req;
  copy_cache_obj_to_request(&req, obj);
  cache_obj_t *demoted_obj = hashtable_insert(params->ht_test, &req);
  demoted_obj->clockpro.referenced = params->init_ref;
  demoted_obj->clockpro.status = CLOCKPRO_TEST;

  params->mem_test += obj->obj_size;

  // 更新链表指针
  demoted_obj->queue.next = params->hand_cold->queue.next;
  demoted_obj->queue.prev = params->hand_cold->queue.prev;

  params->hand_cold->queue.next->queue.prev = demoted_obj;
  params->hand_cold->queue.prev->queue.next = demoted_obj;

  // 更新时钟指针，如果指向被移除的对象
  if (params->hand_hot == obj) {
    params->hand_hot = demoted_obj;
  }
  if (params->hand_test == obj) {
    params->hand_test = demoted_obj;
  }

  // 从缓存中驱逐对象
  cache_evict_base(cache, obj, true);
  params->hand_cold = demoted_obj->queue.next;
}

/**
 * @brief 处理热区的时钟扫描
 * 
 * @param cache 缓存对象
 */
static void ClockPro_run_hot(cache_t *cache) {
  ClockPro_params_t *params = (ClockPro_params_t *)cache->eviction_params;
  cache_obj_t *obj = params->hand_hot;

  // 如果不是热对象，移动指针并返回
  if (obj->clockpro.status != CLOCKPRO_HOT) {
    params->hand_hot = obj->queue.next;
    return;
  }

  // 如果对象被引用，重置引用标志并移动指针
  if (obj->clockpro.referenced) {
    obj->clockpro.referenced = false;
    params->hand_hot = obj->queue.next;
    return;
  }

  // 确保冷区不超过最大大小
  while (params->mem_cold + obj->obj_size > params->mem_cold_max) {
    ClockPro_run_cold(cache);
  }

  // 将热对象降级为冷对象
  obj->clockpro.status = CLOCKPRO_COLD;
  obj->clockpro.referenced = params->init_ref;

  // 更新时钟指针，如果指向被降级的对象
  if (params->hand_cold == obj) {
    params->hand_cold = obj->queue.next;
  }
  if (params->hand_test == obj) {
    params->hand_test = obj->queue.next;
  }

  // 将对象移到链表尾部
  cache_obj_t *hand_hot_next = params->hand_hot->queue.next;
  move_obj_to_tail(&hand_hot_next, &params->hand_hot, obj);
  params->hand_hot = obj->queue.next;

  // 更新区域大小
  params->mem_hot -= obj->obj_size;
  params->mem_cold += obj->obj_size;
}

/**
 * @brief 将对象提升为热对象
 * 
 * @param cache 缓存对象
 * @param obj 要提升的对象
 */
static void ClockPro_promote(cache_t *cache, cache_obj_t *obj) {
  ClockPro_params_t *params = (ClockPro_params_t *)cache->eviction_params;

  // 如果是测试对象，更新冷区最大大小
  if (obj->clockpro.status == CLOCKPRO_TEST) {
    if (params->mem_cold_max + (int64_t)obj->obj_size > cache->cache_size) {
      params->mem_cold_max = cache->cache_size;
    } else {
      params->mem_cold_max += (int64_t)obj->obj_size;
    }
  }

  // 确保热区有足够空间
  while ((params->mem_hot + obj->obj_size) > (cache->cache_size - params->mem_cold_max)) {
    ClockPro_run_hot(cache);
  }

  // 更新时钟指针，如果指向被提升的对象
  if (params->hand_cold == obj) {
    params->hand_cold = obj->queue.next;
  }
  if (params->hand_test == obj) {
    params->hand_test = obj->queue.next;
  }

  // 保存旧状态并更新为热对象
  clockpro_status_e old_status = obj->clockpro.status;
  obj->clockpro.status = CLOCKPRO_HOT;
  obj->clockpro.referenced = params->init_ref;
  
  // 将对象移到链表尾部
  cache_obj_t *hand_hot_next = params->hand_hot->queue.next;
  move_obj_to_tail(&hand_hot_next, &params->hand_hot, obj);
  obj->queue.next  = hand_hot_next;
  hand_hot_next->queue.prev = obj;

  params->hand_hot = obj->queue.next;

  // 根据旧状态更新区域大小
  if (old_status == CLOCKPRO_COLD) {
    params->mem_cold -= obj->obj_size;
  } else if (old_status == CLOCKPRO_TEST) {
    params->mem_test -= obj->obj_size;
  }

  params->mem_hot += obj->obj_size;
}

// ***********************************************************************
// ****                                                               ****
// ****                  parameter set up functions                   ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 获取当前参数字符串
 * 
 * @param cache 缓存对象
 * @param params 参数结构体
 * @return 参数字符串
 */
static const char *ClockPro_current_params(cache_t *cache, ClockPro_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "init-ref=%d\n", params->init_ref);
  return params_str;
}

/**
 * @brief 解析参数字符串
 * 
 * @param cache 缓存对象
 * @param cache_specific_params 参数字符串
 */
static void ClockPro_parse_params(cache_t *cache, const char *cache_specific_params) {
  ClockPro_params_t *params = (ClockPro_params_t *)(cache->eviction_params);
  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;
  char *end;

  while (params_str != NULL && params_str[0] != '\0') {
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    if (strcasecmp(key, "init-ref") == 0) {
      params->init_ref = strtol(value, &end, 10);
    } else if (strcasecmp(key, "init-ratio-cold") == 0) {
      const double ratio = strtod(value, &end);
      params->mem_cold_max = (int64_t)((double)cache->cache_size * ratio);
    } else if (strcasecmp(key, "print") == 0) {
      printf("current parameters: %s\n", ClockPro_current_params(cache, params));
      exit(0);
    } else {
      ERROR("%s does not have parameter %s\n", cache->cache_name, key);
    }
  }
  free(old_params_str);
}

#ifdef __cplusplus
}
#endif
