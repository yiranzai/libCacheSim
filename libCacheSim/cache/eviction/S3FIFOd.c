//
//  Quick demotion + lazy promotion v2
//  快速降级 + 懒惰提升 策略的第二版实现
//
//  FIFO + Clock 组合策略
//  FIFO区域的比例是动态决定的
//  基于FIFO-ghost和main cache的边际命中率
//  我们跟踪FIFO-ghost和main cache的命中分布
//  如果FIFO-ghost在位置0的命中分布大于
//  main cache在位置-1的命中分布
//  我们将FIFO大小增加1
//
//
//  S3FIFOd.c
//  libCacheSim
//
//  Created by Juncheng on 1/24/23
//  Copyright © 2018 Juncheng. All rights reserved.
//

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../include/libCacheSim/evictionAlgo.h"

#ifdef __cplusplus
extern "C" {
#endif

// S3FIFOd算法的参数结构体
typedef struct {
  cache_t *fifo;            // FIFO缓存区
  cache_t *fifo_ghost;      // FIFO的ghost列表，用于记录从FIFO淘汰的对象
  cache_t *main_cache;      // 主缓存区，可配置为不同类型(Clock、LRU等)
  bool hit_on_ghost;        // 是否命中ghost列表的标志
  int move_to_main_threshold; // 对象从FIFO提升到main_cache的访问频率阈值

  double fifo_size_ratio;   // FIFO区域占总缓存的比例
  char main_cache_type[32]; // 主缓存的类型名称

  cache_t *fifo_eviction;      // 跟踪从FIFO淘汰的对象
  cache_t *main_cache_eviction; // 跟踪从main_cache淘汰的对象
  int32_t fifo_eviction_hit;    // FIFO淘汰对象的命中次数
  int32_t main_eviction_hit;    // main缓存淘汰对象的命中次数

  request_t *req_local;     // 本地请求对象，用于临时存储
} S3FIFOd_params_t;

// 默认的缓存参数设置
static const char *DEFAULT_CACHE_PARAMS =
    "fifo-size-ratio=0.10,main-cache=Clock2,move-to-main-threshold=1";

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************
cache_t *S3FIFOd_init(const common_cache_params_t ccache_params,
                     const char *cache_specific_params);
static void S3FIFOd_free(cache_t *cache);
static bool S3FIFOd_get(cache_t *cache, const request_t *req);

static cache_obj_t *S3FIFOd_find(cache_t *cache, const request_t *req,
                                const bool update_cache);
static cache_obj_t *S3FIFOd_insert(cache_t *cache, const request_t *req);
static cache_obj_t *S3FIFOd_to_evict(cache_t *cache, const request_t *req);
static void S3FIFOd_evict(cache_t *cache, const request_t *req);
static bool S3FIFOd_remove(cache_t *cache, const obj_id_t obj_id);
static inline int64_t S3FIFOd_get_occupied_byte(const cache_t *cache);
static inline int64_t S3FIFOd_get_n_obj(const cache_t *cache);
static inline bool S3FIFOd_can_insert(cache_t *cache, const request_t *req);
static void S3FIFOd_parse_params(cache_t *cache,
                                const char *cache_specific_params);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 初始化S3FIFOd缓存，分配资源并设置参数
 * 
 * @param ccache_params 通用缓存参数
 * @param cache_specific_params 特定缓存参数
 * @return 初始化好的缓存实例
 */
cache_t *S3FIFOd_init(const common_cache_params_t ccache_params,
                     const char *cache_specific_params) {
  // 初始化基本缓存结构
  cache_t *cache = cache_struct_init("S3FIFOd", ccache_params, cache_specific_params);
  // 注册缓存操作函数
  cache->cache_init = S3FIFOd_init;
  cache->cache_free = S3FIFOd_free;
  cache->get = S3FIFOd_get;
  cache->find = S3FIFOd_find;
  cache->insert = S3FIFOd_insert;
  cache->evict = S3FIFOd_evict;
  cache->remove = S3FIFOd_remove;
  cache->to_evict = S3FIFOd_to_evict;
  cache->get_n_obj = S3FIFOd_get_n_obj;
  cache->get_occupied_byte = S3FIFOd_get_occupied_byte;
  cache->can_insert = S3FIFOd_can_insert;

  cache->obj_md_size = 0;

  // 分配参数结构体内存
  cache->eviction_params = malloc(sizeof(S3FIFOd_params_t));
  memset(cache->eviction_params, 0, sizeof(S3FIFOd_params_t));
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;
  params->req_local = new_request();
  params->hit_on_ghost = false;

  // 解析缓存参数
  S3FIFOd_parse_params(cache, DEFAULT_CACHE_PARAMS);
  if (cache_specific_params != NULL) {
    S3FIFOd_parse_params(cache, cache_specific_params);
  }

  // 计算各个缓存区域的大小
  int64_t fifo_cache_size =
      (int64_t)ccache_params.cache_size * params->fifo_size_ratio;
  int64_t main_cache_size = ccache_params.cache_size - fifo_cache_size;
  int64_t fifo_ghost_cache_size = main_cache_size;

  // 初始化FIFO缓存区
  common_cache_params_t ccache_params_local = ccache_params;
  ccache_params_local.cache_size = fifo_cache_size;
  params->fifo = FIFO_init(ccache_params_local, NULL);

  // 初始化FIFO ghost缓存区
  ccache_params_local.cache_size = fifo_ghost_cache_size;
  params->fifo_ghost = FIFO_init(ccache_params_local, NULL);
  snprintf(params->fifo_ghost->cache_name, CACHE_NAME_ARRAY_LEN, "FIFO-ghost");

  // 初始化main缓存区，支持多种缓存算法
  ccache_params_local.cache_size = main_cache_size;
  if (strcasecmp(params->main_cache_type, "FIFO") == 0) {
    params->main_cache = FIFO_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "clock") == 0) {
    params->main_cache = Clock_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "clock2") == 0) {
    params->main_cache = Clock_init(ccache_params_local, "n-bit-counter=2");
  } else if (strcasecmp(params->main_cache_type, "clock3") == 0) {
    params->main_cache = Clock_init(ccache_params_local, "n-bit-counter=3");
  } else if (strcasecmp(params->main_cache_type, "sieve") == 0) {
    params->main_cache = Sieve_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LRU") == 0) {
    params->main_cache = LRU_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "ARC") == 0) {
    params->main_cache = ARC_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LHD") == 0) {
    params->main_cache = LHD_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LeCaR") == 0) {
    params->main_cache = LeCaR_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "Cacheus") == 0) {
    params->main_cache = Cacheus_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "twoQ") == 0) {
    params->main_cache = TwoQ_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LIRS") == 0) {
    params->main_cache = LIRS_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "Hyperbolic") == 0) {
    params->main_cache = Hyperbolic_init(ccache_params_local, NULL);
  } else {
    ERROR("S3FIFOd does not support %s \n", params->main_cache_type);
  }

  // 初始化跟踪淘汰对象的缓存
  ccache_params_local.cache_size = ccache_params.cache_size / 10;
  ccache_params_local.hashpower -= 4;
  params->fifo_eviction = FIFO_init(ccache_params_local, NULL);
  params->main_cache_eviction = FIFO_init(ccache_params_local, NULL);
  snprintf(params->fifo_eviction->cache_name, CACHE_NAME_ARRAY_LEN,
           "FIFO-evicted");
  snprintf(params->main_cache_eviction->cache_name, CACHE_NAME_ARRAY_LEN, "%s",
           "main-evicted");

#if defined(TRACK_EVICTION_V_AGE)
  params->fifo->track_eviction_age = false;
  params->main_cache->track_eviction_age = false;
  params->fifo_ghost->track_eviction_age = false;
  params->fifo_eviction->track_eviction_age = false;
  params->main_cache_eviction->track_eviction_age = false;
#endif

  // 设置缓存名称
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "S3FIFOd-%s-%d",
           params->main_cache_type, params->move_to_main_threshold);

  return cache;
}

/**
 * 释放缓存占用的资源
 *
 * @param cache 要释放的缓存实例
 */
static void S3FIFOd_free(cache_t *cache) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;
  free_request(params->req_local);
  params->fifo->cache_free(params->fifo);
  params->fifo_ghost->cache_free(params->fifo_ghost);
  params->main_cache->cache_free(params->main_cache);
  free(cache->eviction_params);
  cache_struct_free(cache);
}

/**
 * @brief 动态更新FIFO缓存大小的函数
 * 基于FIFO淘汰对象和main缓存淘汰对象的命中情况调整大小比例
 * 
 * @param cache 缓存实例
 * @param req 请求对象
 */
static void S3FIFOd_update_fifo_size(cache_t *cache, const request_t *req) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;

  // 计算调整步长，最小为1，最大为两个缓存中较小者的1/1000
  int step = 20;
  step = MAX(
      1, MIN(params->fifo->cache_size, params->main_cache->cache_size) / 1000);
  
  // 检查是否满足调整条件
  bool cond1 = params->fifo_eviction_hit + params->main_eviction_hit > 100;
  bool cond2 = params->main_cache_eviction->get_occupied_byte(
                   params->main_cache_eviction) > 0;
  
  // 如果跟踪淘汰对象的缓存为空，重置命中计数
  if (!cond2) {
    params->fifo_eviction_hit = 0;
    params->main_eviction_hit = 0;
  }

  // 根据命中情况调整缓存大小
  if (cond1 && cond2) {
    if (params->fifo_eviction_hit > params->main_eviction_hit * 2) {
      // 如果FIFO淘汰对象的命中率高，增加FIFO大小
      if (params->main_cache->cache_size > cache->cache_size / 100) {
        params->fifo->cache_size += step;
        params->fifo_ghost->cache_size += step;
        params->main_cache->cache_size -= step;
      }
    } else if (params->main_eviction_hit > params->fifo_eviction_hit * 2) {
      // 如果main缓存淘汰对象的命中率高，减小FIFO大小
      if (params->fifo->cache_size > cache->cache_size / 100) {
        params->fifo->cache_size -= step;
        params->fifo_ghost->cache_size -= step;
        params->main_cache->cache_size += step;
      }
    }
    
    // 衰减命中计数，避免历史数据影响过大
    params->fifo_eviction_hit = params->fifo_eviction_hit * 0.8;
    params->main_eviction_hit = params->main_eviction_hit * 0.8;
  }
}

/**
 * @brief 另一种动态更新FIFO缓存大小的函数(代码中未使用)
 * 每次只调整1个单位的大小
 * 
 * @param cache 缓存实例
 * @param req 请求对象
 */
static void S3FIFOd_update_fifo_size2(cache_t *cache, const request_t *req) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;

  assert(params->fifo_eviction_hit + params->main_eviction_hit <= 1);
  if (params->fifo_eviction_hit == 1 && params->main_cache->cache_size > 1) {
    params->fifo->cache_size += 1;
    params->main_cache->cache_size -= 1;
  } else if (params->main_eviction_hit == 1 && params->fifo->cache_size > 1) {
    params->main_cache->cache_size += 1;
    params->fifo->cache_size -= 1;
  }
  params->fifo_eviction_hit = 0;
  params->main_eviction_hit = 0;
}

/**
 * @brief 处理缓存请求的用户接口函数
 * 执行逻辑：
 * 1. 先动态调整FIFO和main_cache的大小比例
 * 2. 处理请求(查找/插入)
 * 
 * @param cache 缓存实例
 * @param req 请求对象
 * @return 命中返回true，未命中返回false
 */
static bool S3FIFOd_get(cache_t *cache, const request_t *req) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;
  DEBUG_ASSERT(params->fifo->get_occupied_byte(params->fifo) +
                   params->main_cache->get_occupied_byte(params->main_cache) <=
               cache->cache_size);

  // 动态调整FIFO和main_cache的大小
  S3FIFOd_update_fifo_size(cache, req);

  // 处理请求
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
 * @param cache 缓存实例
 * @param req 请求对象
 * @param update_cache 是否更新缓存元数据
 *  为true时，会更新对象的访问状态
 *  如果对象已过期，会从缓存中移除
 * @return 找到的对象指针，未找到返回NULL
 */
static cache_obj_t *S3FIFOd_find(cache_t *cache, const request_t *req,
                                const bool update_cache) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;

  // 如果不需要更新缓存，只检查对象是否存在
  if (!update_cache) {
    cache_obj_t *obj = params->fifo->find(params->fifo, req, false);
    if (obj != NULL) {
      return obj;
    }
    obj = params->main_cache->find(params->main_cache, req, false);
    if (obj != NULL) {
      return obj;
    }
    return NULL;
  }

  /* 从这里开始需要更新缓存 */
  params->hit_on_ghost = false;
  // 先在FIFO缓存中查找
  cache_obj_t *obj = params->fifo->find(params->fifo, req, true);
  if (obj != NULL) {
    return obj;
  }

  // 检查ghost列表中是否存在该对象
  if (params->fifo_ghost->remove(params->fifo_ghost, req->obj_id)) {
    params->hit_on_ghost = true;
  }

  // 在main缓存中查找
  obj = params->main_cache->find(params->main_cache, req, update_cache);

  // 检查是否命中过去淘汰的对象
  if (params->fifo_eviction->find(params->fifo_eviction, req, false) != NULL) {
    params->fifo_eviction->remove(params->fifo_eviction, req->obj_id);
    params->fifo_eviction_hit++;
  }

  if (params->main_cache_eviction->find(params->main_cache_eviction, req,
                                        true) != NULL) {
    params->main_cache_eviction->remove(params->main_cache_eviction,
                                        req->obj_id);
    params->main_eviction_hit++;
  }

  return obj;
}

/**
 * @brief 向缓存中插入对象
 * 更新哈希表和缓存元数据
 * 此函数假设缓存有足够空间，应在调用前执行淘汰
 *
 * @param cache 缓存实例
 * @param req 请求对象
 * @return 插入的对象指针
 */
static cache_obj_t *S3FIFOd_insert(cache_t *cache, const request_t *req) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;
  cache_obj_t *obj = NULL;

  if (params->hit_on_ghost) {
    /* 如果命中ghost，直接插入main缓存 */
    params->hit_on_ghost = false;
    params->main_cache->get(params->main_cache, req);
    obj = params->main_cache->find(params->main_cache, req, false);
  } else {
    /* 否则插入FIFO缓存 */
    obj = params->fifo->insert(params->fifo, req);
  }

  // 确保频率初始化为0
  assert(obj->misc.freq == 0);

#if defined(TRACK_EVICTION_V_AGE)
  obj->create_time = CURR_TIME(cache, req);
#endif

  return obj;
}

/**
 * @brief 查找要淘汰的对象
 * 此函数不实际淘汰对象或更新元数据
 * S3FIFOd不支持此功能，因为淘汰逻辑不能与查找候选对象分离
 *
 * @param cache 缓存实例
 * @return 要淘汰的对象指针
 */
static cache_obj_t *S3FIFOd_to_evict(cache_t *cache, const request_t *req) {
  assert(false);
  return NULL;
}

/**
 * @brief 从缓存中淘汰对象
 * 在返回前需要调用cache_evict_base
 * 以更新一些元数据如n_obj、占用大小和哈希表
 *
 * @param cache 缓存实例
 * @param req 请求对象，此处未使用
 */
static void S3FIFOd_evict(cache_t *cache, const request_t *req) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;

  cache_t *fifo = params->fifo;
  cache_t *ghost = params->fifo_ghost;
  cache_t *main = params->main_cache;

  if (fifo->get_occupied_byte(fifo) == 0) {
    // 如果FIFO为空，从main缓存淘汰
    assert(main->get_occupied_byte(main) <= cache->cache_size);
    cache_obj_t *obj = main->to_evict(main, req);
#if defined(TRACK_EVICTION_V_AGE)
    record_eviction_age(cache, obj, CURR_TIME(cache, req) - obj->create_time);
#endif
    // 记录被淘汰的对象
    copy_cache_obj_to_request(params->req_local, obj);
    params->main_cache_eviction->get(params->main_cache_eviction,
                                     params->req_local);
    main->evict(main, req);
    return;
  }

  // 从FIFO淘汰
  cache_obj_t *obj = fifo->to_evict(fifo, req);
  assert(obj != NULL);
  // 在淘汰前复制对象
  copy_cache_obj_to_request(params->req_local, obj);

#if defined(TRACK_EVICTION_V_AGE)
  if (obj->misc.freq >= params->move_to_main_threshold) {
    // 如果访问频率达到阈值，提升到main缓存
    cache_obj_t *new_obj = main->insert(main, params->req_local);
    new_obj->create_time = obj->create_time;
    // 从FIFO中移除
    bool removed = fifo->remove(fifo, params->req_local->obj_id);
    assert(removed);

    // 如果main缓存超过大小限制，淘汰对象
    while (main->get_occupied_byte(main) > main->cache_size) {
      obj = main->to_evict(main, req);
      copy_cache_obj_to_request(params->req_local, obj);
      params->main_cache_eviction->get(params->main_cache_eviction,
                                       params->req_local);
      main->evict(main, req);
    }
  } else {
    // 从FIFO移除
    bool removed = fifo->remove(fifo, params->req_local->obj_id);
    assert(removed);

    // 记录淘汰时间
    record_eviction_age(cache, obj, CURR_TIME(cache, req) - obj->create_time);
    // 插入ghost列表
    ghost->get(ghost, params->req_local);
    params->fifo_eviction->get(params->fifo_eviction, params->req_local);
  }

#else
  // 从FIFO移除
  bool removed = fifo->remove(fifo, params->req_local->obj_id);
  assert(removed);

  if (obj->misc.freq >= params->move_to_main_threshold) {
    // 如果访问频率达到阈值，提升到main缓存
    main->insert(main, params->req_local);

    // 如果main缓存超过大小限制，淘汰对象
    while (main->get_occupied_byte(main) > main->cache_size) {
      obj = main->to_evict(main, req);
      copy_cache_obj_to_request(params->req_local, obj);
      params->main_cache_eviction->get(params->main_cache_eviction,
                                       params->req_local);
      main->evict(main, req);
    }
  } else {
    // 插入ghost列表
    ghost->get(ghost, params->req_local);
    params->fifo_eviction->get(params->fifo_eviction, params->req_local);
  }
#endif
}

/**
 * @brief 从缓存中删除指定对象
 * 这与cache_evict不同，它用于用户触发的删除
 * 而不是缓存为新对象腾出空间的淘汰
 *
 * 在返回前需要调用cache_remove_obj_base
 * 以更新一些元数据如n_obj、占用大小和哈希表
 *
 * @param cache 缓存实例
 * @param obj_id 要删除的对象ID
 * @return 如果对象被删除返回true，如果对象不在缓存中返回false
 */
static bool S3FIFOd_remove(cache_t *cache, const obj_id_t obj_id) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;
  bool removed = false;
  // 尝试从各个缓存区域删除对象
  removed = removed || params->fifo->remove(params->fifo, obj_id);
  removed = removed || params->fifo_ghost->remove(params->fifo_ghost, obj_id);
  removed = removed || params->main_cache->remove(params->main_cache, obj_id);

  return removed;
}

/**
 * @brief 获取缓存当前占用的字节数
 * 
 * @param cache 缓存实例
 * @return 占用的字节数
 */
static inline int64_t S3FIFOd_get_occupied_byte(const cache_t *cache) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;
  return params->fifo->get_occupied_byte(params->fifo) +
         params->main_cache->get_occupied_byte(params->main_cache);
}

/**
 * @brief 获取缓存中的对象数量
 * 
 * @param cache 缓存实例
 * @return 对象数量
 */
static inline int64_t S3FIFOd_get_n_obj(const cache_t *cache) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;
  return params->fifo->get_n_obj(params->fifo) +
         params->main_cache->get_n_obj(params->main_cache);
}

/**
 * @brief 检查是否可以将对象插入缓存
 * 
 * @param cache 缓存实例
 * @param req 请求对象
 * @return 如果可以插入返回true，否则返回false
 */
static inline bool S3FIFOd_can_insert(cache_t *cache, const request_t *req) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)cache->eviction_params;

  return req->obj_size <= params->fifo->cache_size && cache_can_insert_default(cache, req);
}

// ***********************************************************************
// ****                                                               ****
// ****                parameter set up functions                     ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 获取当前S3FIFOd参数的字符串表示
 * 
 * @param params S3FIFOd参数结构体
 * @return 参数字符串
 */
static const char *S3FIFOd_current_params(S3FIFOd_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "fifo-size-ratio=%.4lf,main-cache=%s\n",
           params->fifo_size_ratio, params->main_cache->cache_name);
  return params_str;
}

/**
 * @brief 解析缓存特定参数
 * 
 * @param cache 缓存实例
 * @param cache_specific_params 特定参数字符串
 */
static void S3FIFOd_parse_params(cache_t *cache,
                                const char *cache_specific_params) {
  S3FIFOd_params_t *params = (S3FIFOd_params_t *)(cache->eviction_params);

  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;
  // char *end;

  while (params_str != NULL && params_str[0] != '\0') {
    /* 不同参数由逗号分隔，键和值由等号分隔 */
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    // 跳过空格
    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    if (strcasecmp(key, "fifo-size-ratio") == 0) {
      params->fifo_size_ratio = strtod(value, NULL);
    } else if (strcasecmp(key, "main-cache") == 0) {
      strncpy(params->main_cache_type, value, 30);
    } else if (strcasecmp(key, "move-to-main-threshold") == 0) {
      params->move_to_main_threshold = atoi(value);
    } else if (strcasecmp(key, "print") == 0) {
      printf("parameters: %s\n", S3FIFOd_current_params(params));
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
