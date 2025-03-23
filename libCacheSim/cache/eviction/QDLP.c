//
//  Quick demotion + lazy promotion v1
//
//  20% FIFO + ARC
//  insert to ARC when evicting from FIFO
//
//
//  QDLP.c
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

// QDLP算法参数结构体，包含FIFO缓存、FIFO幽灵缓存和主缓存
typedef struct {
  cache_t *fifo;           // FIFO缓存，用于快速淘汰
  cache_t *fifo_ghost;     // FIFO幽灵缓存，记录从FIFO淘汰的对象
  cache_t *main_cache;     // 主缓存，使用更复杂的替换策略
  bool hit_on_ghost;       // 标记是否命中幽灵缓存

  int64_t n_obj_admit_to_fifo;    // 记录添加到FIFO的对象数量
  int64_t n_obj_admit_to_main;    // 记录直接添加到主缓存的对象数量
  int64_t n_obj_move_to_main;     // 记录从FIFO移动到主缓存的对象数量
  int64_t n_byte_admit_to_fifo;   // 记录添加到FIFO的字节数
  int64_t n_byte_admit_to_main;   // 记录直接添加到主缓存的字节数
  int64_t n_byte_move_to_main;    // 记录从FIFO移动到主缓存的字节数

  int move_to_main_threshold;     // 对象从FIFO移动到主缓存的频率阈值
  double fifo_size_ratio;         // FIFO缓存占总缓存的比例
  double ghost_size_ratio;        // 幽灵缓存大小比例
  char main_cache_type[32];       // 主缓存类型名称

  request_t *req_local;           // 本地请求对象，用于内部操作
} QDLP_params_t;

// 默认缓存参数：FIFO占10%，幽灵缓存占90%，主缓存使用Clock2算法，移动阈值为1
static const char *DEFAULT_CACHE_PARAMS =
    "fifo-size-ratio=0.10,ghost-size-ratio=0.9,main-cache=Clock2,move-to-main-"
    "threshold=1";

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************
cache_t *QDLP_init(const common_cache_params_t ccache_params,
                     const char *cache_specific_params);
static void QDLP_free(cache_t *cache);
static bool QDLP_get(cache_t *cache, const request_t *req);

static cache_obj_t *QDLP_find(cache_t *cache, const request_t *req,
                                const bool update_cache);
static cache_obj_t *QDLP_insert(cache_t *cache, const request_t *req);
static cache_obj_t *QDLP_to_evict(cache_t *cache, const request_t *req);
static void QDLP_evict(cache_t *cache, const request_t *req);
static bool QDLP_remove(cache_t *cache, const obj_id_t obj_id);
static inline int64_t QDLP_get_occupied_byte(const cache_t *cache);
static inline int64_t QDLP_get_n_obj(const cache_t *cache);
static inline bool QDLP_can_insert(cache_t *cache, const request_t *req);
static void QDLP_parse_params(cache_t *cache,
                                const char *cache_specific_params);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ***********************************************************************

/**
 * 初始化QDLP缓存
 * 
 * @param ccache_params 通用缓存参数
 * @param cache_specific_params QDLP特定参数
 * @return 初始化好的缓存对象
 */
cache_t *QDLP_init(const common_cache_params_t ccache_params,
                     const char *cache_specific_params) {
  // 初始化缓存结构
  cache_t *cache =
      cache_struct_init("QDLP", ccache_params, cache_specific_params);
  // 设置缓存函数指针
  cache->cache_init = QDLP_init;
  cache->cache_free = QDLP_free;
  cache->get = QDLP_get;
  cache->find = QDLP_find;
  cache->insert = QDLP_insert;
  cache->evict = QDLP_evict;
  cache->remove = QDLP_remove;
  cache->to_evict = QDLP_to_evict;
  cache->get_n_obj = QDLP_get_n_obj;
  cache->get_occupied_byte = QDLP_get_occupied_byte;
  cache->can_insert = QDLP_can_insert;

  cache->obj_md_size = 0;

  // 分配并初始化QDLP参数
  cache->eviction_params = malloc(sizeof(QDLP_params_t));
  memset(cache->eviction_params, 0, sizeof(QDLP_params_t));
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;
  params->req_local = new_request();  // 创建本地请求对象
  params->hit_on_ghost = false;

  // 解析默认参数
  QDLP_parse_params(cache, DEFAULT_CACHE_PARAMS);
  // 如果提供了特定参数，则解析它们
  if (cache_specific_params != NULL) {
    QDLP_parse_params(cache, cache_specific_params);
  }

  // 计算各个缓存的大小
  int64_t fifo_cache_size =
      (int64_t)ccache_params.cache_size * params->fifo_size_ratio;
  int64_t main_cache_size = ccache_params.cache_size - fifo_cache_size;
  int64_t fifo_ghost_cache_size =
      (int64_t)(ccache_params.cache_size * params->ghost_size_ratio);

  // 初始化FIFO缓存
  common_cache_params_t ccache_params_local = ccache_params;
  ccache_params_local.cache_size = fifo_cache_size;
  params->fifo = FIFO_init(ccache_params_local, NULL);

  // 如果幽灵缓存大小大于0，则初始化幽灵缓存
  if (fifo_ghost_cache_size > 0) {
    ccache_params_local.cache_size = fifo_ghost_cache_size;
    params->fifo_ghost = FIFO_init(ccache_params_local, NULL);
    snprintf(params->fifo_ghost->cache_name, CACHE_NAME_ARRAY_LEN,
             "FIFO-ghost");
  } else {
    params->fifo_ghost = NULL;
  }

  // 初始化主缓存，根据指定的类型
  ccache_params_local.cache_size = main_cache_size;
  // 根据main_cache_type选择不同的缓存算法
  if (strcasecmp(params->main_cache_type, "ARC") == 0) {
    params->main_cache = ARC_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LHD") == 0) {
    params->main_cache = LHD_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "clock") == 0) {
    params->main_cache = Clock_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "sieve") == 0) {
    params->main_cache = Sieve_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "clock2") == 0) {
    params->main_cache = Clock_init(ccache_params_local, "n-bit-counter=2");
  } else if (strcasecmp(params->main_cache_type, "clock3") == 0) {
    params->main_cache = Clock_init(ccache_params_local, "n-bit-counter=3");
  } else if (strcasecmp(params->main_cache_type, "LRU") == 0) {
    params->main_cache = LRU_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LeCaR") == 0) {
    params->main_cache = LeCaR_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "Cacheus") == 0) {
    params->main_cache = Cacheus_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "twoQ") == 0) {
    params->main_cache = TwoQ_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "FIFO") == 0) {
    params->main_cache = FIFO_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "SLRU") == 0) {
    params->main_cache = SLRU_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LIRS") == 0) {
    params->main_cache = LIRS_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "Hyperbolic") == 0) {
    params->main_cache = Hyperbolic_init(ccache_params_local, NULL);
  } else {
    ERROR("QDLP does not support %s \n", params->main_cache_type);
  }

#if defined(TRACK_EVICTION_V_AGE)
  // 如果定义了跟踪淘汰年龄，则禁用子缓存的跟踪
  if (params->fifo_ghost != NULL) {
    params->fifo_ghost->track_eviction_age = false;
  }
  params->fifo->track_eviction_age = false;
  params->main_cache->track_eviction_age = false;
#endif

  // 设置缓存名称，包含关键参数信息
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "QDLP-%.4lf-%.4lf-%s-%d",
           params->fifo_size_ratio, params->ghost_size_ratio,
           params->main_cache_type, params->move_to_main_threshold);

  return cache;
}

/**
 * 释放缓存使用的资源
 *
 * @param cache 要释放的缓存
 */
static void QDLP_free(cache_t *cache) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;
  // 释放本地请求对象
  free_request(params->req_local);
  // 释放FIFO缓存
  params->fifo->cache_free(params->fifo);
  // 如果有幽灵缓存，释放它
  if (params->fifo_ghost != NULL) {
    params->fifo_ghost->cache_free(params->fifo_ghost);
  }
  // 释放主缓存
  params->main_cache->cache_free(params->main_cache);
  // 释放参数结构体
  free(cache->eviction_params);
  // 释放缓存结构体
  cache_struct_free(cache);
}

/**
 * @brief 用户面向的API函数，处理缓存请求
 * 执行以下逻辑：
 *
 * ```
 * 如果对象在缓存中:
 *    更新元数据
 *    返回true
 * 否则:
 *    如果缓存没有足够空间:
 *        淘汰对象直到有足够空间插入
 *    插入对象
 *    返回false
 * ```
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果缓存命中返回true，否则返回false
 */
static bool QDLP_get(cache_t *cache, const request_t *req) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;
  // 确保FIFO和主缓存的总大小不超过缓存总大小
  DEBUG_ASSERT(params->fifo->get_occupied_byte(params->fifo) +
                   params->main_cache->get_occupied_byte(params->main_cache) <=
               cache->cache_size);

  // 调用基础缓存获取函数
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
static cache_obj_t *QDLP_find(cache_t *cache, const request_t *req,
                                const bool update_cache) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;

  // 如果不需要更新缓存，只检查FIFO和主缓存
  if (!update_cache) {
    // 先在FIFO缓存中查找
    cache_obj_t *obj = params->fifo->find(params->fifo, req, false);
    if (obj != NULL) {
      return obj;
    }
    // 再在主缓存中查找
    obj = params->main_cache->find(params->main_cache, req, false);
    if (obj != NULL) {
      return obj;
    }
    return NULL;
  }

  /* 从这里开始，update_cache为true */
  params->hit_on_ghost = false;
  // 先在FIFO缓存中查找
  cache_obj_t *obj = params->fifo->find(params->fifo, req, true);
  if (obj != NULL) {
    return obj;
  }

  // 检查是否在幽灵缓存中
  if (params->fifo_ghost != NULL &&
      params->fifo_ghost->remove(params->fifo_ghost, req->obj_id)) {
    // 如果对象在幽灵缓存中，remove会返回true
    params->hit_on_ghost = true;
  }

  // 在主缓存中查找
  obj = params->main_cache->find(params->main_cache, req, true);

  return obj;
}

/**
 * @brief 将对象插入缓存
 * 更新哈希表和缓存元数据
 * 此函数假设缓存有足够空间
 * 在调用此函数前应该执行淘汰
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 插入的对象
 */
static cache_obj_t *QDLP_insert(cache_t *cache, const request_t *req) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;
  cache_obj_t *obj = NULL;

  if (params->hit_on_ghost) {
    /* 如果命中幽灵缓存，插入到主缓存 */
    params->hit_on_ghost = false;
    // 更新统计信息
    params->n_obj_admit_to_main += 1;
    params->n_byte_admit_to_main += req->obj_size;
    // 将对象插入主缓存
    params->main_cache->get(params->main_cache, req);
    obj = params->main_cache->find(params->main_cache, req, false);
  } else {
    /* 否则插入到FIFO缓存 */
    // 如果对象大小超过FIFO缓存大小，无法插入
    if (req->obj_size >= params->fifo->cache_size) {
      return NULL;
    }
    // 更新统计信息
    params->n_obj_admit_to_fifo += 1;
    params->n_byte_admit_to_fifo += req->obj_size;
    // 将对象插入FIFO缓存
    obj = params->fifo->insert(params->fifo, req);
  }

#if defined(TRACK_EVICTION_V_AGE)
  // 如果跟踪淘汰年龄，记录创建时间
  obj->create_time = CURR_TIME(cache, req);
#endif

  // 确保对象的频率为0
  assert(obj->misc.freq == 0);

  return obj;
}

/**
 * @brief 找出要淘汰的对象
 * 此函数不会实际淘汰对象或更新元数据
 * 不是所有淘汰算法都支持此函数
 * 因为淘汰逻辑可能无法与查找淘汰候选对象分离
 * 如果无法支持此函数，使用assert(false)
 *
 * @param cache 缓存对象
 * @return 要淘汰的对象
 */
static cache_obj_t *QDLP_to_evict(cache_t *cache, const request_t *req) {
  // QDLP不支持此函数
  assert(false);
  return NULL;
}

/**
 * @brief 从缓存中淘汰对象
 * 在返回前需要调用cache_evict_base
 * 它会更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param req 请求对象（不使用）
 */
static void QDLP_evict(cache_t *cache, const request_t *req) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;

  cache_t *fifo = params->fifo;
  cache_t *ghost = params->fifo_ghost;
  cache_t *main = params->main_cache;

  // 如果FIFO缓存为空，从主缓存淘汰
  if (fifo->get_occupied_byte(fifo) == 0) {
#if defined(TRACK_EVICTION_V_AGE)
    // 如果跟踪淘汰年龄，记录淘汰对象的年龄
    cache_obj_t *obj = main->to_evict(main, req);
    record_eviction_age(cache, obj, CURR_TIME(cache, req) - obj->create_time);
#endif

    // 确保主缓存大小不超过总缓存大小
    assert(main->get_occupied_byte(main) <= cache->cache_size);
    // 从主缓存淘汰
    main->evict(main, req);

    return;
  }

  // 从FIFO缓存淘汰
  cache_obj_t *obj = fifo->to_evict(fifo, req);
  assert(obj != NULL);
  // 在对象被淘汰前需要复制它
  copy_cache_obj_to_request(params->req_local, obj);

  // 如果对象的频率达到阈值，移动到主缓存
  if (obj->misc.freq >= params->move_to_main_threshold) {
    // 更新统计信息
    params->n_obj_move_to_main += 1;
    params->n_byte_move_to_main += obj->obj_size;

    // 将对象插入主缓存
    params->main_cache->get(params->main_cache, params->req_local);
#if defined(TRACK_EVICTION_V_AGE)
    // 如果跟踪淘汰年龄，保持创建时间
    main->find(main, params->req_local, false)->create_time = obj->create_time;
  } else {
    // 记录淘汰年龄
    record_eviction_age(cache, obj, CURR_TIME(cache, req) - obj->create_time);
#else
  } else {
#endif
    // 如果不移动到主缓存，插入到幽灵缓存
    if (ghost != NULL) {
      ghost->get(ghost, params->req_local);
    }
  }

  // 从FIFO缓存淘汰，但不更新统计信息
  fifo->evict(fifo, req);
}

/**
 * @brief 从缓存中移除对象
 * 这与cache_evict不同，因为它用于用户触发的移除
 * 而淘汰是由缓存用来为新对象腾出空间
 *
 * 在返回前需要调用cache_remove_obj_base
 * 它会更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param obj_id 对象ID
 * @return 如果对象被移除返回true，如果对象不在缓存中返回false
 */
static bool QDLP_remove(cache_t *cache, const obj_id_t obj_id) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;
  bool removed = false;
  // 尝试从FIFO缓存移除
  removed = removed || params->fifo->remove(params->fifo, obj_id);
  // 尝试从幽灵缓存移除（如果存在）
  removed = removed || (params->fifo_ghost &&
                        params->fifo_ghost->remove(params->fifo_ghost, obj_id));
  // 尝试从主缓存移除
  removed = removed || params->main_cache->remove(params->main_cache, obj_id);

  return removed;
}

/**
 * 获取缓存占用的字节数
 * 
 * @param cache 缓存对象
 * @return 占用的字节数
 */
static inline int64_t QDLP_get_occupied_byte(const cache_t *cache) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;
  // 返回FIFO缓存和主缓存占用字节数之和
  return params->fifo->get_occupied_byte(params->fifo) +
         params->main_cache->get_occupied_byte(params->main_cache);
}

/**
 * 获取缓存中的对象数量
 * 
 * @param cache 缓存对象
 * @return 对象数量
 */
static inline int64_t QDLP_get_n_obj(const cache_t *cache) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;
  // 返回FIFO缓存和主缓存对象数量之和
  return params->fifo->get_n_obj(params->fifo) +
         params->main_cache->get_n_obj(params->main_cache);
}

/**
 * 检查是否可以插入对象
 * 
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果可以插入返回true，否则返回false
 */
static inline bool QDLP_can_insert(cache_t *cache, const request_t *req) {
  QDLP_params_t *params = (QDLP_params_t *)cache->eviction_params;

  // 对象大小必须小于FIFO缓存大小，且满足默认插入条件
  return req->obj_size <= params->fifo->cache_size && cache_can_insert_default(cache, req);
}

// ***********************************************************************
// ****                                                               ****
// ****                parameter set up functions                     ****
// ****                                                               ****
// ***********************************************************************
/**
 * 获取当前QDLP参数的字符串表示
 * 
 * @param params QDLP参数
 * @return 参数字符串
 */
static const char *QDLP_current_params(QDLP_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "fifo-size-ratio=%.4lf,main-cache=%s\n",
           params->fifo_size_ratio, params->main_cache->cache_name);
  return params_str;
}

/**
 * 解析QDLP特定参数
 * 
 * @param cache 缓存对象
 * @param cache_specific_params 特定参数字符串
 */
static void QDLP_parse_params(cache_t *cache,
                                const char *cache_specific_params) {
  QDLP_params_t *params = (QDLP_params_t *)(cache->eviction_params);

  // 复制参数字符串以便修改
  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;

  // 解析参数字符串
  while (params_str != NULL && params_str[0] != '\0') {
    /* 不同参数用逗号分隔，
     * 键和值用等号分隔 */
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    // 跳过空格
    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    // 根据键设置相应的参数
    if (strcasecmp(key, "fifo-size-ratio") == 0) {
      params->fifo_size_ratio = strtod(value, NULL);
    } else if (strcasecmp(key, "ghost-size-ratio") == 0) {
      params->ghost_size_ratio = strtod(value, NULL);
    } else if (strcasecmp(key, "move-to-main-threshold") == 0) {
      params->move_to_main_threshold = atoi(value);
    } else if (strcasecmp(key, "main-cache") == 0) {
      strncpy(params->main_cache_type, value, 30);
    } else if (strcasecmp(key, "print") == 0) {
      // 打印当前参数并退出
      printf("parameters: %s\n", QDLP_current_params(params));
      exit(0);
    } else {
      // 未知参数，报错并退出
      ERROR("%s does not have parameter %s\n", cache->cache_name, key);
      exit(1);
    }
  }

  // 释放复制的参数字符串
  free(old_params_str);
}

#ifdef __cplusplus
}
#endif
