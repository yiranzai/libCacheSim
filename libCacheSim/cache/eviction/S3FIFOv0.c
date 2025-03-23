//  This is the original S3FIFO implementation used in
//  "FIFO queues are all you need for cache eviction" from SOSP
//  10% small FIFO + 90% main FIFO (2-bit Clock) + ghost
//  insert to small FIFO if not in the ghost, else insert to the main FIFO
//  evict from small FIFO:
//      if object in the small is accessed,
//          reinsert to main FIFO,
//      else
//          evict and insert to the ghost
//  evict from main FIFO:
//      if object in the main is accessed,
//          reinsert to main FIFO,
//      else
//          evict
//
//  S3FIFO (Scan-resistant Simple Segmented FIFO) 是一种缓存替换算法，
//  它结合了分段FIFO和时钟算法的特点，以提高缓存命中率。
//  该算法由三个部分组成：
//  1. small FIFO (默认占总缓存大小的10%)：新对象首先进入这里
//  2. main FIFO (剩余缓存空间)：使用2位时钟算法，保存热点对象
//  3. ghost FIFO (默认为缓存大小的90%)：记录最近被驱逐的对象，不存储实际数据
//
//  工作流程：
//  - 如果请求的对象不在ghost中，插入到small FIFO
//  - 如果请求的对象在ghost中，插入到main FIFO
//  - 从small FIFO驱逐时：
//    * 如果对象被访问过(freq>=threshold)，移动到main FIFO
//    * 否则驱逐并添加到ghost
//  - 从main FIFO驱逐时：
//    * 如果对象被访问过(freq>=1)，降低频率计数并重新插入
//    * 否则直接驱逐
//
//  S3FIFOv0.c
//  libCacheSim
//
//  Created by Juncheng on 12/4/22.
//  Copyright © 2018 Juncheng. All rights reserved.
//

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../include/libCacheSim/evictionAlgo.h"

#ifdef __cplusplus
extern "C" {
#endif

// S3FIFO算法的参数结构体
typedef struct {
  cache_t *small_fifo;        // 小FIFO队列，用于新对象的初始存储
  cache_t *ghost_fifo;        // ghost FIFO队列，记录最近驱逐的对象
  cache_t *main_fifo;         // 主FIFO队列，使用时钟算法保存热点对象
  bool hit_on_ghost;          // 标记当前请求是否命中ghost

  int64_t n_obj_admit_to_small;  // 记录插入small FIFO的对象数量
  int64_t n_obj_admit_to_main;   // 记录插入main FIFO的对象数量
  int64_t n_obj_move_to_main;    // 记录从small移动到main的对象数量
  int64_t n_byte_admit_to_small; // 记录插入small FIFO的字节数
  int64_t n_byte_admit_to_main;  // 记录插入main FIFO的字节数
  int64_t n_byte_move_to_main;   // 记录从small移动到main的字节数

  int move_to_main_threshold;    // 对象从small移动到main的访问频率阈值
  double small_size_ratio;       // small FIFO占总缓存的比例
  double ghost_size_ratio;       // ghost FIFO的大小比例

  request_t *req_local;          // 本地请求对象，用于内部操作
} S3FIFOv0_params_t;

// 默认参数配置
static const char *DEFAULT_CACHE_PARAMS = "small-size-ratio=0.10,ghost-size-ratio=0.90,move-to-main-threshold=2";

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************
cache_t *S3FIFOv0_init(const common_cache_params_t ccache_params, const char *cache_specific_params);
static void S3FIFOv0_free(cache_t *cache);
static bool S3FIFOv0_get(cache_t *cache, const request_t *req);

static cache_obj_t *S3FIFOv0_find(cache_t *cache, const request_t *req, const bool update_cache);
static cache_obj_t *S3FIFOv0_insert(cache_t *cache, const request_t *req);
static cache_obj_t *S3FIFOv0_to_evict(cache_t *cache, const request_t *req);
static void S3FIFOv0_evict(cache_t *cache, const request_t *req);
static bool S3FIFOv0_remove(cache_t *cache, const obj_id_t obj_id);
static inline int64_t S3FIFOv0_get_occupied_byte(const cache_t *cache);
static inline int64_t S3FIFOv0_get_n_obj(const cache_t *cache);
static inline bool S3FIFOv0_can_insert(cache_t *cache, const request_t *req);
static void S3FIFOv0_parse_params(cache_t *cache, const char *cache_specific_params);

static void S3FIFOv0_evict_small(cache_t *cache, const request_t *req);
static void S3FIFOv0_evict_main(cache_t *cache, const request_t *req);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 初始化S3FIFO缓存
 * 
 * @param ccache_params 通用缓存参数
 * @param cache_specific_params S3FIFO特定参数
 * @return cache_t* 初始化后的缓存对象
 */
cache_t *S3FIFOv0_init(const common_cache_params_t ccache_params, const char *cache_specific_params) {
  cache_t *cache = cache_struct_init("S3FIFOv0", ccache_params, cache_specific_params);
  cache->cache_init = S3FIFOv0_init;
  cache->cache_free = S3FIFOv0_free;
  cache->get = S3FIFOv0_get;
  cache->find = S3FIFOv0_find;
  cache->insert = S3FIFOv0_insert;
  cache->evict = S3FIFOv0_evict;
  cache->remove = S3FIFOv0_remove;
  cache->to_evict = S3FIFOv0_to_evict;
  cache->get_n_obj = S3FIFOv0_get_n_obj;
  cache->get_occupied_byte = S3FIFOv0_get_occupied_byte;
  cache->can_insert = S3FIFOv0_can_insert;

  cache->obj_md_size = 0;

  // 分配并初始化参数结构体
  cache->eviction_params = malloc(sizeof(S3FIFOv0_params_t));
  memset(cache->eviction_params, 0, sizeof(S3FIFOv0_params_t));
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  params->req_local = new_request();
  params->hit_on_ghost = false;

  // 解析参数
  S3FIFOv0_parse_params(cache, DEFAULT_CACHE_PARAMS);
  if (cache_specific_params != NULL) {
    S3FIFOv0_parse_params(cache, cache_specific_params);
  }

  // 根据比例计算各个FIFO的大小
  int64_t fifo_cache_size = (int64_t)ccache_params.cache_size * params->small_size_ratio;
  int64_t main_fifo_size = ccache_params.cache_size - fifo_cache_size;
  int64_t ghostfifo__cachee_siz = (int64_t)(ccache_params.cache_size * params->ghost_size_ratio);

  // 初始化small FIFO
  common_cache_params_t ccache_params_local = ccache_params;
  ccache_params_local.cache_size = fifo_cache_size;
  params->small_fifo = FIFO_init(ccache_params_local, NULL);

  // 初始化ghost FIFO（如果大小大于0）
  if (ghostfifo__cachee_siz > 0) {
    ccache_params_local.cache_size = ghostfifo__cachee_siz;
    params->ghost_fifo = FIFO_init(ccache_params_local, NULL);
    snprintf(params->ghost_fifo->cache_name, CACHE_NAME_ARRAY_LEN, "FIFO-ghost");
  } else {
    params->ghost_fifo = NULL;
  }

  // 初始化main FIFO
  ccache_params_local.cache_size = main_fifo_size;
  params->main_fifo = FIFO_init(ccache_params_local, NULL);

#if defined(TRACK_EVICTION_V_AGE)
  if (params->ghost_fifo != NULL) {
    params->ghost_fifo->track_eviction_age = false;
  }
  params->small_fifo->track_eviction_age = false;
  params->main_fifo->track_eviction_age = false;
#endif

  // 设置缓存名称
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "S3FIFOv0-%.4lf-%d", params->small_size_ratio,
           params->move_to_main_threshold);

  return cache;
}

/**
 * 释放缓存使用的资源
 *
 * @param cache 要释放的缓存对象
 */
static void S3FIFOv0_free(cache_t *cache) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  free_request(params->req_local);
  params->small_fifo->cache_free(params->small_fifo);
  if (params->ghost_fifo != NULL) {
    params->ghost_fifo->cache_free(params->ghost_fifo);
  }
  params->main_fifo->cache_free(params->main_fifo);
  free(cache->eviction_params);
  cache_struct_free(cache);
}

/**
 * @brief 用户面向的API，处理缓存请求
 * 
 * 执行以下逻辑：
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
 * @return true 如果缓存命中，false 如果缓存未命中
 */
static bool S3FIFOv0_get(cache_t *cache, const request_t *req) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  DEBUG_ASSERT(params->small_fifo->get_occupied_byte(params->small_fifo) +
                   params->main_fifo->get_occupied_byte(params->main_fifo) <=
               cache->cache_size);

  bool cache_hit = cache_get_base(cache, req);

  return cache_hit;
}

// ***********************************************************************
// ****                                                               ****
// ****       developer facing APIs (used by cache developer)         ****
// ****                                                               ****
// ***********************************************************************
/**
 * @brief 在缓存中查找对象
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @param update_cache 是否更新缓存
 *  如果为true，对象会被提升
 *  如果对象已过期，会从缓存中移除
 * @return 找到的对象，如果未找到则返回NULL
 */
static cache_obj_t *S3FIFOv0_find(cache_t *cache, const request_t *req, const bool update_cache) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;

  cache_t *small = params->small_fifo;
  cache_t *main = params->main_fifo;

  // 如果不需要更新缓存，只检查small和main缓存
  if (!update_cache) {
    cache_obj_t *obj = small->find(small, req, false);
    if (obj != NULL) {
      return obj;
    }
    obj = main->find(main, req, false);
    if (obj != NULL) {
      return obj;
    }
    return NULL;
  }

  /* 从这里开始update_cache为true */
  params->hit_on_ghost = false;
  // 先在small FIFO中查找
  cache_obj_t *obj = small->find(small, req, true);
  if (obj != NULL) {
    obj->S3FIFO.freq += 1;  // 增加访问频率
    return obj;
  }

  // 检查ghost FIFO
  if (params->ghost_fifo != NULL && params->ghost_fifo->remove(params->ghost_fifo, req->obj_id)) {
    // 如果对象在ghost_fifo中，remove将返回true
    params->hit_on_ghost = true;
  }

  // 在main FIFO中查找
  obj = main->find(main, req, true);
  if (obj != NULL) {
    obj->S3FIFO.freq += 1;  // 增加访问频率
  }

  return obj;
}

/**
 * @brief 将对象插入缓存
 * 更新哈希表和缓存元数据
 * 此函数假设缓存有足够的空间
 * 在调用此函数前应该执行驱逐操作
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 插入的对象
 */
static cache_obj_t *S3FIFOv0_insert(cache_t *cache, const request_t *req) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  cache_obj_t *obj = NULL;

  if (params->hit_on_ghost) {
    /* 如果命中ghost，插入到main FIFO */
    params->hit_on_ghost = false;
    params->n_obj_admit_to_main += 1;
    params->n_byte_admit_to_main += req->obj_size;
    obj = params->main_fifo->insert(params->main_fifo, req);
  } else {
    /* 否则插入到small FIFO */
    if (req->obj_size >= params->small_fifo->cache_size) {
      return NULL;  // 对象太大，无法插入small FIFO
    }
    params->n_obj_admit_to_small += 1;
    params->n_byte_admit_to_small += req->obj_size;
    obj = params->small_fifo->insert(params->small_fifo, req);
  }

#if defined(TRACK_EVICTION_V_AGE)
  obj->create_time = CURR_TIME(cache, req);
#endif

#if defined(TRACK_DEMOTION)
  obj->create_time = cache->n_req;
#endif

  obj->S3FIFO.freq = 0;  // 初始化访问频率

  return obj;
}

/**
 * @brief 找到要驱逐的对象
 * 此函数不会实际驱逐对象或更新元数据
 * 不是所有驱逐算法都支持此函数
 * 因为驱逐逻辑可能无法与查找驱逐候选对象分离
 * 如果无法支持此函数，使用assert(false)
 *
 * @param cache 缓存对象
 * @return 要驱逐的对象
 */
static cache_obj_t *S3FIFOv0_to_evict(cache_t *cache, const request_t *req) {
  assert(false);
  return NULL;
}

/**
 * @brief 从small FIFO驱逐对象
 * 
 * 如果对象访问频率达到阈值，移动到main FIFO
 * 否则驱逐并添加到ghost FIFO
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 */
static void S3FIFOv0_evict_small(cache_t *cache, const request_t *req) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  cache_t *small = params->small_fifo;
  cache_t *ghost = params->ghost_fifo;
  cache_t *main = params->main_fifo;

  bool has_evicted = false;
  while (!has_evicted && small->get_occupied_byte(small) > 0) {
    // 从small FIFO中选择要驱逐的对象
    cache_obj_t *obj_to_evict = small->to_evict(small, req);
    DEBUG_ASSERT(obj_to_evict != NULL);
    // 在对象被驱逐前需要复制它
    copy_cache_obj_to_request(params->req_local, obj_to_evict);

    // 检查访问频率是否达到阈值
    if (obj_to_evict->S3FIFO.freq >= params->move_to_main_threshold) {
#if defined(TRACK_DEMOTION)
      printf("%ld keep %ld %ld\n", cache->n_req, obj_to_evict->create_time, obj_to_evict->misc.next_access_vtime);
#endif
      // 访问频率达到阈值，移动到main FIFO
      params->n_obj_move_to_main += 1;
      params->n_byte_move_to_main += obj_to_evict->obj_size;

      cache_obj_t *new_obj = main->insert(main, params->req_local);
#if defined(TRACK_EVICTION_V_AGE)
      new_obj->create_time = obj_to_evict->create_time;
    } else {
      record_eviction_age(cache, obj_to_evict, CURR_TIME(cache, req) - obj_to_evict->create_time);
#else
    } else {
#endif

#if defined(TRACK_DEMOTION)
      printf("%ld demote %ld %ld\n", cache->n_req, obj_to_evict->create_time, obj_to_evict->misc.next_access_vtime);
#endif

      // 访问频率未达到阈值，插入到ghost
      if (ghost != NULL) {
        ghost->get(ghost, params->req_local);
      }
      has_evicted = true;
    }

    // 从small FIFO中移除，但不更新统计信息
    bool removed = small->remove(small, params->req_local->obj_id);
    DEBUG_ASSERT(removed);
  }
}

/**
 * @brief 从main FIFO驱逐对象
 * 
 * 使用2位时钟算法：
 * - 如果对象访问频率>=1，降低频率并重新插入
 * - 否则直接驱逐
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 */
static void S3FIFOv0_evict_main(cache_t *cache, const request_t *req) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  cache_t *main = params->main_fifo;

  // 从main缓存中驱逐
  bool has_evicted = false;
  while (!has_evicted && main->get_occupied_byte(main) > 0) {
    cache_obj_t *obj_to_evict = main->to_evict(main, req);
    DEBUG_ASSERT(obj_to_evict != NULL);
    int freq = obj_to_evict->S3FIFO.freq;
#if defined(TRACK_EVICTION_V_AGE)
    int64_t create_time = obj_to_evict->create_time;
#endif
    copy_cache_obj_to_request(params->req_local, obj_to_evict);
    if (freq >= 1) {
      // 需要先驱逐，因为要插入的对象有相同的obj_id
      main->remove(main, obj_to_evict->obj_id);
      obj_to_evict = NULL;

      // 重新插入对象，并降低频率（2位时钟算法）
      cache_obj_t *new_obj = main->insert(main, params->req_local);
      new_obj->S3FIFO.freq = MIN(freq, 3) - 1;

#if defined(TRACK_EVICTION_V_AGE)
      new_obj->create_time = create_time;
#endif
    } else {
#if defined(TRACK_EVICTION_V_AGE)
      record_eviction_age(cache, obj_to_evict, CURR_TIME(cache, req) - obj_to_evict->create_time);
#endif

      // 频率为0，直接驱逐
      bool removed = main->remove(main, obj_to_evict->obj_id);
      DEBUG_ASSERT(removed);

      has_evicted = true;
    }
  }
}

/**
 * @brief 从缓存中驱逐对象
 * 需要在返回前调用cache_evict_base
 * 更新一些元数据，如n_obj、已占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param req 请求对象（不使用）
 */
static void S3FIFOv0_evict(cache_t *cache, const request_t *req) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;

  cache_t *fifo = params->small_fifo;
  cache_t *main = params->main_fifo;

  // 决定从哪个FIFO驱逐
  if (main->get_occupied_byte(main) > main->cache_size || fifo->get_occupied_byte(fifo) == 0) {
    return S3FIFOv0_evict_main(cache, req);
  }
  return S3FIFOv0_evict_small(cache, req);
}

/**
 * @brief 从缓存中移除对象
 * 这与cache_evict不同，因为它用于用户触发的移除
 * 而驱逐是由缓存用来为新对象腾出空间
 *
 * 需要在返回前调用cache_remove_obj_base
 * 更新一些元数据，如n_obj、已占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param obj_id 要移除的对象ID
 * @return true 如果对象被移除，false 如果对象不在缓存中
 */
static bool S3FIFOv0_remove(cache_t *cache, const obj_id_t obj_id) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  bool removed = false;
  // 尝试从所有FIFO中移除对象
  removed = removed || params->small_fifo->remove(params->small_fifo, obj_id);
  removed = removed || (params->ghost_fifo && params->ghost_fifo->remove(params->ghost_fifo, obj_id));
  removed = removed || params->main_fifo->remove(params->main_fifo, obj_id);

  return removed;
}

/**
 * @brief 获取缓存已占用的字节数
 * 
 * @param cache 缓存对象
 * @return int64_t 已占用的字节数
 */
static inline int64_t S3FIFOv0_get_occupied_byte(const cache_t *cache) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  return params->small_fifo->get_occupied_byte(params->small_fifo) +
         params->main_fifo->get_occupied_byte(params->main_fifo);
}

/**
 * @brief 获取缓存中的对象数量
 * 
 * @param cache 缓存对象
 * @return int64_t 对象数量
 */
static inline int64_t S3FIFOv0_get_n_obj(const cache_t *cache) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;
  return params->small_fifo->get_n_obj(params->small_fifo) + params->main_fifo->get_n_obj(params->main_fifo);
}

/**
 * @brief 检查是否可以插入对象
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 * @return true 如果可以插入
 */
static inline bool S3FIFOv0_can_insert(cache_t *cache, const request_t *req) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)cache->eviction_params;

  return req->obj_size <= params->small_fifo->cache_size && cache_can_insert_default(cache, req);
}

// ***********************************************************************
// ****                                                               ****
// ****                parameter set up functions                     ****
// ****                                                               ****
// ***********************************************************************
/**
 * @brief 获取当前参数的字符串表示
 * 
 * @param params 参数结构体
 * @return const char* 参数字符串
 */
static const char *S3FIFOv0_current_params(S3FIFOv0_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "small-size-ratio=%.4lf,main-cache=%s\n", params->small_size_ratio,
           params->main_fifo->cache_name);
  return params_str;
}

/**
 * @brief 解析缓存特定参数
 * 
 * @param cache 缓存对象
 * @param cache_specific_params 特定参数字符串
 */
static void S3FIFOv0_parse_params(cache_t *cache, const char *cache_specific_params) {
  S3FIFOv0_params_t *params = (S3FIFOv0_params_t *)(cache->eviction_params);

  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;

  while (params_str != NULL && params_str[0] != '\0') {
    /* 不同参数用逗号分隔，
     * 键和值用等号分隔 */
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    // 跳过空格
    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    if (strcasecmp(key, "fifo-size-ratio") == 0 || strcasecmp(key, "small-size-ratio") == 0) {
      params->small_size_ratio = strtod(value, NULL);
    } else if (strcasecmp(key, "ghost-size-ratio") == 0) {
      params->ghost_size_ratio = strtod(value, NULL);
    } else if (strcasecmp(key, "move-to-main-threshold") == 0) {
      params->move_to_main_threshold = atoi(value);
    } else if (strcasecmp(key, "print") == 0) {
      printf("parameters: %s\n", S3FIFOv0_current_params(params));
      exit(0);
    } else {
      ERROR("%s does not have parameter %s\n", cache->cache_name, key);
      exit(1);
    }
  }

  free(old_params_str);
}

#ifdef __cplusplus
}
#endif
