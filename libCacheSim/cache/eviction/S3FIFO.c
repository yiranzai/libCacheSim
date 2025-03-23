//
//  This version (S3FIFO.c) differs from the original S3-FIFO (S3FIFOv0.c) in that when the small queue is full, but the
//  cache is not full, the original S3-FIFO will insert into the small queue, but this version will insert into the main
//  queue. This version is in general better than the original S3-FIFO because
//    1. the objects inserted after the cache is full are evicted more quickly
//    2. the objects inserted between the small queue is full and the cache is full are kept slightly longer
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
//  - 当small FIFO已满但整个缓存未满时，新对象将插入main FIFO（这是与S3FIFOv0的主要区别）
//  - 从small FIFO驱逐时：
//    * 如果对象被访问过(freq>=threshold)，移动到main FIFO
//    * 否则驱逐并添加到ghost
//  - 从main FIFO驱逐时：
//    * 如果对象被访问过(freq>=1)，降低频率计数并重新插入
//    * 否则直接驱逐
//
//  S3FIFO算法的主要优势：
//  - 抵抗扫描：通过small FIFO过滤一次性访问的对象
//  - 高效利用缓存空间：将热点对象保留在main FIFO中
//  - ghost缓存记忆功能：通过ghost FIFO识别最近被驱逐但又被重新访问的对象
//  - 简单高效：实现简单，计算开销小，适合大规模缓存系统
//
//  参数说明：
//  - small-size-ratio: small FIFO占总缓存大小的比例，默认0.10
//  - ghost-size-ratio: ghost FIFO的大小比例，默认0.90
//  - move-to-main-threshold: 对象从small移动到main的访问频率阈值，默认2
//
//  S3FIFO.c
//  libCacheSim
//
//  Created by Juncheng on 12/4/24.
//  Copyright © 2018 Juncheng. All rights reserved.
//

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../include/libCacheSim/evictionAlgo.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  cache_t *small_fifo;       // 小队列，用于新对象的初始存储
  cache_t *ghost_fifo;       // 幽灵队列，记录最近被驱逐的对象
  cache_t *main_fifo;        // 主队列，存储热点对象
  bool hit_on_ghost;         // 标记当前请求是否命中幽灵队列

  int move_to_main_threshold;  // 从small移动到main的访问频率阈值
  double small_size_ratio;     // small FIFO占总缓存大小的比例
  double ghost_size_ratio;     // ghost FIFO的大小比例

  bool has_evicted;          // 标记缓存是否已经开始驱逐对象
  request_t *req_local;      // 本地请求对象，用于内部操作
} S3FIFO_params_t;

static const char *DEFAULT_CACHE_PARAMS = "small-size-ratio=0.10,ghost-size-ratio=0.90,move-to-main-threshold=2";

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************
cache_t *S3FIFO_init(const common_cache_params_t ccache_params, const char *cache_specific_params);
static void S3FIFO_free(cache_t *cache);
static bool S3FIFO_get(cache_t *cache, const request_t *req);

static cache_obj_t *S3FIFO_find(cache_t *cache, const request_t *req, const bool update_cache);
static cache_obj_t *S3FIFO_insert(cache_t *cache, const request_t *req);
static cache_obj_t *S3FIFO_to_evict(cache_t *cache, const request_t *req);
static void S3FIFO_evict(cache_t *cache, const request_t *req);
static bool S3FIFO_remove(cache_t *cache, const obj_id_t obj_id);
static inline int64_t S3FIFO_get_occupied_byte(const cache_t *cache);
static inline int64_t S3FIFO_get_n_obj(const cache_t *cache);
static inline bool S3FIFO_can_insert(cache_t *cache, const request_t *req);
static void S3FIFO_parse_params(cache_t *cache, const char *cache_specific_params);

static void S3FIFO_evict_small(cache_t *cache, const request_t *req);
static void S3FIFO_evict_main(cache_t *cache, const request_t *req);

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
 * @return 初始化的缓存对象
 */
cache_t *S3FIFO_init(const common_cache_params_t ccache_params, const char *cache_specific_params) {
  cache_t *cache = cache_struct_init("S3FIFO", ccache_params, cache_specific_params);
  cache->cache_init = S3FIFO_init;
  cache->cache_free = S3FIFO_free;
  cache->get = S3FIFO_get;
  cache->find = S3FIFO_find;
  cache->insert = S3FIFO_insert;
  cache->evict = S3FIFO_evict;
  cache->remove = S3FIFO_remove;
  cache->to_evict = S3FIFO_to_evict;
  cache->get_n_obj = S3FIFO_get_n_obj;
  cache->get_occupied_byte = S3FIFO_get_occupied_byte;
  cache->can_insert = S3FIFO_can_insert;

  cache->obj_md_size = 0;

  cache->eviction_params = malloc(sizeof(S3FIFO_params_t));
  memset(cache->eviction_params, 0, sizeof(S3FIFO_params_t));
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  params->req_local = new_request();
  params->hit_on_ghost = false;

  S3FIFO_parse_params(cache, DEFAULT_CACHE_PARAMS);
  if (cache_specific_params != NULL) {
    S3FIFO_parse_params(cache, cache_specific_params);
  }

  // 计算各个队列的大小
  int64_t small_fifo_size = (int64_t)ccache_params.cache_size * params->small_size_ratio;
  int64_t main_fifo_size = ccache_params.cache_size - small_fifo_size;
  int64_t ghost_fifo_size = (int64_t)(ccache_params.cache_size * params->ghost_size_ratio);

  // 初始化small FIFO
  common_cache_params_t ccache_params_local = ccache_params;
  ccache_params_local.cache_size = small_fifo_size;
  params->small_fifo = FIFO_init(ccache_params_local, NULL);
  params->has_evicted = false;

  // 初始化ghost FIFO（如果启用）
  if (ghost_fifo_size > 0) {
    ccache_params_local.cache_size = ghost_fifo_size;
    params->ghost_fifo = FIFO_init(ccache_params_local, NULL);
    snprintf(params->ghost_fifo->cache_name, CACHE_NAME_ARRAY_LEN, "FIFO-ghost");
  } else {
    params->ghost_fifo = NULL;
  }

  // 初始化main FIFO
  ccache_params_local.cache_size = main_fifo_size;
  params->main_fifo = FIFO_init(ccache_params_local, NULL);

  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "S3FIFO-%.4lf-%d", params->small_size_ratio,
           params->move_to_main_threshold);

  return cache;
}

/**
 * @brief 释放缓存使用的资源
 * 释放S3FIFO缓存使用的所有内存资源，包括small、main和ghost队列
 *
 * @param cache 要释放的缓存对象
 */
static void S3FIFO_free(cache_t *cache) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
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
 * @brief 用户面向的API函数，处理缓存请求
 * 执行以下逻辑：
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
 * @return 如果缓存命中返回true，缓存未命中返回false
 */
static bool S3FIFO_get(cache_t *cache, const request_t *req) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
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
static cache_obj_t *S3FIFO_find(cache_t *cache, const request_t *req, const bool update_cache) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;

  // 如果update_cache为false，只检查small和main缓存
  if (!update_cache) {
    cache_obj_t *obj = params->small_fifo->find(params->small_fifo, req, false);
    if (obj != NULL) {
      return obj;
    }
    obj = params->main_fifo->find(params->main_fifo, req, false);
    if (obj != NULL) {
      return obj;
    }
    return NULL;
  }

  /* update cache is true from now */
  params->hit_on_ghost = false;
  // 先在small FIFO中查找
  cache_obj_t *obj = params->small_fifo->find(params->small_fifo, req, true);
  if (obj != NULL) {
    obj->S3FIFO.freq += 1;  // 增加访问频率
    return obj;
  }

  // 检查ghost FIFO
  if (params->ghost_fifo != NULL && params->ghost_fifo->remove(params->ghost_fifo, req->obj_id)) {
    // 如果对象在ghost_fifo中，remove会返回true
    params->hit_on_ghost = true;
  }

  // 最后在main FIFO中查找
  obj = params->main_fifo->find(params->main_fifo, req, true);
  if (obj != NULL) {
    obj->S3FIFO.freq += 1;  // 增加访问频率
  }

  return obj;
}

/**
 * @brief 将对象插入缓存
 * 更新哈希表和缓存元数据
 * 此函数假设缓存有足够的空间
 * 在调用此函数前应执行驱逐操作
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 插入的对象
 */
static cache_obj_t *S3FIFO_insert(cache_t *cache, const request_t *req) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  cache_obj_t *obj = NULL;

  cache_t *small = params->small_fifo;
  cache_t *main = params->main_fifo;

  if (params->hit_on_ghost) {
    /* 如果命中ghost，插入到main FIFO */
    params->hit_on_ghost = false;
    obj = main->insert(main, req);
  } else {
    /* 否则插入到small FIFO */
    if (req->obj_size >= small->cache_size) {
      return NULL;  // 对象太大，无法插入small FIFO
    }

    // 如果small已满但缓存未开始驱逐，插入到main
    if (!params->has_evicted && small->get_occupied_byte(small) >= small->cache_size) {
      obj = main->insert(main, req);
    } else {
      obj = small->insert(small, req);
    }
  }

  obj->S3FIFO.freq = 0;  // 初始化访问频率

  return obj;
}

/**
 * @brief 查找要驱逐的对象
 * 此函数不实际驱逐对象或更新元数据
 * 不是所有驱逐算法都支持此函数
 * 因为驱逐逻辑可能无法与查找驱逐候选对象分离
 * 
 * @param cache 缓存对象
 * @return 要驱逐的对象
 */
static cache_obj_t *S3FIFO_to_evict(cache_t *cache, const request_t *req) {
  assert(false);  // S3FIFO不支持此函数
  return NULL;
}

/**
 * @brief 从small FIFO驱逐对象
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 */
/**
 * @brief 从small FIFO驱逐对象
 * 如果对象访问频率达到阈值，会将其移动到main FIFO
 * 否则将其驱逐并添加到ghost FIFO
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 无返回值
 */
static void S3FIFO_evict_small(cache_t *cache, const request_t *req) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  cache_t *small = params->small_fifo;
  cache_t *ghost = params->ghost_fifo;
  cache_t *main = params->main_fifo;

  bool has_evicted = false;
  while (!has_evicted && small->get_occupied_byte(small) > 0) {
    // 获取要驱逐的对象
    cache_obj_t *obj_to_evict = small->to_evict(small, req);
    DEBUG_ASSERT(obj_to_evict != NULL);
    // 在对象被驱逐前复制它
    copy_cache_obj_to_request(params->req_local, obj_to_evict);

    // 如果对象访问频率达到阈值，移动到main FIFO
    if (obj_to_evict->S3FIFO.freq >= params->move_to_main_threshold) {
      cache_obj_t *new_obj = main->insert(main, params->req_local);
    } else {
      // 否则插入到ghost FIFO
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
 * @param cache 缓存对象
 * @param req 请求对象
 */
/**
 * @brief 从main FIFO驱逐对象
 * 实现2位时钟算法：如果对象被访问过(freq>=1)，降低频率并重新插入
 * 否则直接驱逐
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 无返回值
 */
static void S3FIFO_evict_main(cache_t *cache, const request_t *req) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  cache_t *main = params->main_fifo;

  bool has_evicted = false;
  while (!has_evicted && main->get_occupied_byte(main) > 0) {
    // 获取要驱逐的对象
    cache_obj_t *obj_to_evict = main->to_evict(main, req);
    DEBUG_ASSERT(obj_to_evict != NULL);
    int freq = obj_to_evict->S3FIFO.freq;
    copy_cache_obj_to_request(params->req_local, obj_to_evict);
    
    // 实现时钟算法：如果对象被访问过，降低频率并重新插入
    if (freq >= 1) {
      // 需要先驱逐，因为要插入的对象有相同的obj_id
      main->remove(main, obj_to_evict->obj_id);
      obj_to_evict = NULL;

      cache_obj_t *new_obj = main->insert(main, params->req_local);
      // 2位时钟计数器实现
      new_obj->S3FIFO.freq = MIN(freq, 3) - 1;

    } else {
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
 * @param req 请求对象（未使用）
 */
static void S3FIFO_evict(cache_t *cache, const request_t *req) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  params->has_evicted = true;

  cache_t *small = params->small_fifo;
  cache_t *main = params->main_fifo;

  // 决定从哪个队列驱逐：
  // 1. 如果main超出大小限制，从main驱逐
  // 2. 如果small为空，从main驱逐
  // 3. 否则从small驱逐
  if (main->get_occupied_byte(main) > main->cache_size || small->get_occupied_byte(small) == 0) {
    S3FIFO_evict_main(cache, req);
  } else {
    S3FIFO_evict_small(cache, req);
  }
  return S3FIFO_evict_small(cache, req);
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
 * @param obj_id 对象ID
 * @return 如果对象被移除返回true，如果对象不在缓存中返回false
 */
static bool S3FIFO_remove(cache_t *cache, const obj_id_t obj_id) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  bool removed = false;
  // 尝试从所有队列中移除对象
  removed = removed || params->small_fifo->remove(params->small_fifo, obj_id);
  removed = removed || (params->ghost_fifo && params->ghost_fifo->remove(params->ghost_fifo, obj_id));
  removed = removed || params->main_fifo->remove(params->main_fifo, obj_id);

  return removed;
}

/**
 * @brief 获取缓存已占用的字节数
 * 
 * @param cache 缓存对象
 * @return 已占用的字节数
 */
static inline int64_t S3FIFO_get_occupied_byte(const cache_t *cache) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  return params->small_fifo->get_occupied_byte(params->small_fifo) +
         params->main_fifo->get_occupied_byte(params->main_fifo);
}

/**
 * @brief 获取缓存中的对象数量
 * 
 * @param cache 缓存对象
 * @return 对象数量
 */
static inline int64_t S3FIFO_get_n_obj(const cache_t *cache) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;
  return params->small_fifo->get_n_obj(params->small_fifo) + params->main_fifo->get_n_obj(params->main_fifo);
}

/**
 * @brief 检查是否可以插入对象
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果可以插入返回true
 */
static inline bool S3FIFO_can_insert(cache_t *cache, const request_t *req) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)cache->eviction_params;

  return req->obj_size <= params->small_fifo->cache_size && cache_can_insert_default(cache, req);
}

// ***********************************************************************
// ****                                                               ****
// ****                parameter set up functions                     ****
// ****                                                               ****
// ***********************************************************************
/**
 * @brief 获取当前参数的字符串表示
 * 将S3FIFO算法的当前参数配置转换为字符串形式
 * 用于调试、日志记录或打印当前配置信息
 * 
 * @param params S3FIFO参数结构体指针
 * @return 格式化后的参数字符串，格式为"small-size-ratio=X,ghost-size-ratio=Y,move-to-main-threshold=Z"
 */
static const char *S3FIFO_current_params(S3FIFO_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "small-size-ratio=%.4lf,ghost-size-ratio=%.4lf,move-to-main-threshold=%d\n",
           params->small_size_ratio, params->ghost_size_ratio, params->move_to_main_threshold);
  return params_str;
}

/**
 * @brief 解析S3FIFO特定参数
 * 解析用户提供的参数字符串，设置S3FIFO算法的配置参数
 * 支持的参数包括：
 * - small-size-ratio/fifo-size-ratio: small FIFO占总缓存大小的比例
 * - ghost-size-ratio: ghost FIFO的大小比例
 * - move-to-main-threshold: 对象从small移动到main的访问频率阈值
 * 
 * @param cache 缓存对象
 * @param cache_specific_params 特定参数字符串，格式为"key1=value1,key2=value2,..."
 */
static void S3FIFO_parse_params(cache_t *cache, const char *cache_specific_params) {
  S3FIFO_params_t *params = (S3FIFO_params_t *)(cache->eviction_params);

  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;
  char *end;

  while (params_str != NULL && params_str[0] != '\0') {
    /* different parameters are separated by comma,
     * key and value are separated by = */
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    // skip the white space
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
      printf("parameters: %s\n", S3FIFO_current_params(params));
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