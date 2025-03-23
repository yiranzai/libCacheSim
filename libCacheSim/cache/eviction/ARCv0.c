//
//  ARCv0 缓存替换算法实现
//  基于论文：https://www.usenix.org/conference/fast-03/ARCv0-self-tuning-low-overhead-replacement-cache
//
//  ARCv0算法概述：
//  - 自适应替换缓存(Adaptive Replacement Cache)的一种变体实现
//  - 结合了最近使用(LRU)和频率使用(LFU)的优点
//  - 使用四个列表管理缓存对象：
//    * T1：存储最近一次访问的对象
//    * T2：存储最近多次访问的对象
//    * B1：T1的ghost列表，记录从T1淘汰的对象
//    * B2：T2的ghost列表，记录从T2淘汰的对象
//  - 通过参数p动态调整T1和T2的大小，适应不同的访问模式
//  - 相比标准ARC，此实现使用了子缓存组件而非直接管理链表
//
//  实现细节：
//  - 与https://github.com/trauzti/cache/blob/master/ARCv0.py交叉检查
//  - 关于delta和p参数是整型还是浮点型，论文中没有明确说明
//  - 最初使用整型实现，后改为浮点型以匹配参考实现
//  - 可选配置LAZY_PROMOTION和QUICK_DEMOTION用于性能优化
//
//  libCacheSim
//
//  Created by Juncheng on 09/28/20.
//  Copyright © 2020 Juncheng. All rights reserved.
//

#include <string.h>

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../include/libCacheSim/evictionAlgo.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define DEBUG_MODE
// #undef DEBUG_MODE

// #define LAZY_PROMOTION
// #define QUICK_DEMOTION

typedef struct ARCv0_params {
  // L1_data is T1 in the paper, L1_ghost is B1 in the paper
  cache_t *T1;
  cache_t *B1;
  cache_t *T2;
  cache_t *B2;

  double p;
  bool curr_obj_in_L1_ghost;
  bool curr_obj_in_L2_ghost;
  int64_t vtime_last_req_in_ghost;
  request_t *req_local;
} ARCv0_params_t;

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************

static void ARCv0_parse_params(cache_t *cache,
                               const char *cache_specific_params);
static void ARCv0_free(cache_t *cache);
static bool ARCv0_get(cache_t *cache, const request_t *req);
static cache_obj_t *ARCv0_find(cache_t *cache, const request_t *req,
                               const bool update_cache);
static cache_obj_t *ARCv0_insert(cache_t *cache, const request_t *req);
static cache_obj_t *ARCv0_to_evict(cache_t *cache, const request_t *req);
static void ARCv0_evict(cache_t *cache, const request_t *req);
static bool ARCv0_remove(cache_t *cache, const obj_id_t obj_id);
static int64_t ARCv0_get_occupied_byte(const cache_t *cache);
static int64_t ARCv0_get_n_obj(const cache_t *cache);

/* internal functions */

/* this is the case IV in the paper */
static void _ARCv0_evict_miss_on_all_queues(cache_t *cache,
                                            const request_t *req);
static void _ARCv0_replace(cache_t *cache, const request_t *req);
static cache_obj_t *_ARCv0_to_evict_miss_on_all_queues(cache_t *cache,
                                                       const request_t *req);
static cache_obj_t *_ARCv0_to_replace(cache_t *cache, const request_t *req);

static bool ARCv0_get_debug(cache_t *cache, const request_t *req);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ****                       init, free, get                         ****
// ***********************************************************************

/**
 * @brief 初始化ARCv0缓存
 * 设置缓存的各种参数、函数指针和子缓存组件
 *
 * @param ccache_params 通用缓存参数，包含缓存大小、哈希表大小等基本配置
 * @param cache_specific_params ARCv0特定参数，可以使用parse_params函数解析
 *        或者使用cachesim二进制文件的-e "print"选项查看可用参数
 * @return 初始化好的缓存对象
 */
cache_t *ARCv0_init(const common_cache_params_t ccache_params,
                    const char *cache_specific_params) {
  // 初始化基本缓存结构
  cache_t *cache =
      cache_struct_init("ARCv0", ccache_params, cache_specific_params);
  
  // 设置缓存操作的函数指针
  cache->cache_init = ARCv0_init;           // 初始化函数
  cache->cache_free = ARCv0_free;           // 释放资源函数
  cache->get = ARCv0_get;                   // 获取对象函数
  cache->find = ARCv0_find;                 // 查找对象函数
  cache->insert = ARCv0_insert;             // 插入对象函数
  cache->evict = ARCv0_evict;               // 淘汰对象函数
  cache->remove = ARCv0_remove;             // 移除对象函数
  cache->to_evict = ARCv0_to_evict;         // 确定淘汰对象函数
  cache->can_insert = cache_can_insert_default;  // 检查是否可以插入函数
  cache->get_occupied_byte = ARCv0_get_occupied_byte;  // 获取占用字节数函数
  cache->get_n_obj = ARCv0_get_n_obj;       // 获取对象数量函数

  // 设置对象元数据大小
  if (ccache_params.consider_obj_metadata) {
    // 两个指针（前后链接）+ ghost元数据
    cache->obj_md_size = 8 * 2 + 8 * 3;
  } else {
    cache->obj_md_size = 0;
  }

  // 分配ARCv0特定参数的内存
  cache->eviction_params = my_malloc_n(ARCv0_params_t, 1);
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  // 初始化参数p为0，p控制T1和T2的目标大小
  params->p = 0;

  // 复制通用缓存参数
  common_cache_params_t ccache_params_local = ccache_params;
  
  // 初始化四个子缓存组件
  params->T1 = LRU_init(ccache_params_local, NULL);  // 最近一次访问的对象列表
  params->B1 = LRU_init(ccache_params_local, NULL);  // T1的ghost列表
  
#ifdef LAZY_PROMOTION
  // 延迟提升模式：使用Clock算法实现T2，减少对象移动开销
  params->T2 = Clock_init(ccache_params_local, NULL);
#else
  // 标准模式：使用LRU算法实现T2
  params->T2 = LRU_init(ccache_params_local, NULL);  // 多次访问的对象列表
#endif
  params->B2 = LRU_init(ccache_params_local, NULL);  // T2的ghost列表

  // 初始化ghost缓存相关标志
  params->curr_obj_in_L1_ghost = false;
  params->curr_obj_in_L2_ghost = false;
  params->vtime_last_req_in_ghost = -1;
  // 创建本地请求对象，用于内部操作
  params->req_local = new_request();

  // 根据编译选项设置缓存名称
#if defined(LAZY_PROMOTION) && defined(QUICK_DEMOTION)
  // 同时启用延迟提升和快速降级
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "ARCv0-LP-QD");
#elif defined(LAZY_PROMOTION)
  // 仅启用延迟提升
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "ARCv0-LP");
#elif defined(QUICK_DEMOTION)
  // 仅启用快速降级
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "ARCv0-QD");
#endif

  return cache;
}

/**
 * @brief 释放缓存使用的资源
 * 依次释放四个子缓存组件、本地请求对象和参数结构体
 *
 * @param cache 要释放的缓存对象
 */
static void ARCv0_free(cache_t *cache) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  // 释放四个子缓存组件
  params->T1->cache_free(params->T1);  // 释放T1（最近一次访问的对象列表）
  params->T2->cache_free(params->T2);  // 释放T2（多次访问的对象列表）
  params->B1->cache_free(params->B1);  // 释放B1（T1的ghost列表）
  params->B2->cache_free(params->B2);  // 释放B2（T2的ghost列表）

  // 释放本地请求对象
  free_request(params->req_local);
  // 释放参数结构体
  my_free(sizeof(ARCv0_params_t), params);
  // 释放缓存基本结构
  cache_struct_free(cache);
}

/**
 * @brief 用户面向的API，处理缓存请求的主要入口
 * 执行以下逻辑：
 *
 * ```
 * 如果对象在缓存中：
 *    更新元数据
 *    返回true（缓存命中）
 * 否则：
 *    如果缓存没有足够空间：
 *        淘汰对象直到有足够空间插入
 *    插入新对象
 *    返回false（缓存未命中）
 * ```
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果缓存命中返回true，未命中返回false
 */
static bool ARCv0_get(cache_t *cache, const request_t *req) {
  // ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
#ifdef DEBUG_MODE
  // 调试模式：使用带有详细输出的调试版本
  return ARCv0_get_debug(cache, req);
#else
  // 正常模式：使用基础缓存获取函数
  // cache_get_base实现了通用的缓存获取逻辑
  return cache_get_base(cache, req);
#endif
}

// ***********************************************************************
// ****                                                               ****
// ****       developer facing APIs (used by cache developer)         ****
// ****                                                               ****
// ***********************************************************************

/**
 * @brief 在缓存中查找对象并根据需要更新缓存状态
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @param update_cache 是否更新缓存，
 *  如果为true，则对象会被提升到适当位置
 *  如果对象已过期，则会从缓存中移除
 * @return 找到的对象指针，如果未找到则返回NULL
 */
static cache_obj_t *ARCv0_find(cache_t *cache, const request_t *req,
                               const bool update_cache) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  // 首先在T1和T2缓存中查找对象
  cache_obj_t *obj_t1 = params->T1->find(params->T1, req, false);
  cache_obj_t *obj_t2 = params->T2->find(params->T2, req, false);
  // 确保对象不会同时出现在T1和T2中
  DEBUG_ASSERT(obj_t1 == NULL || obj_t2 == NULL);
  // 获取找到的对象（如果存在）
  cache_obj_t *obj = obj_t1 ? obj_t1 : obj_t2;

  // 如果不需要更新缓存，直接返回找到的对象
  if (!update_cache) {
    return obj;
  }

  // 在ghost缓存(B1和B2)中查找对象
  cache_obj_t *obj_b1 = params->B1->find(params->B1, req, false);
  cache_obj_t *obj_b2 = params->B2->find(params->B2, req, false);
  // 确保对象不会同时出现在B1和B2中
  DEBUG_ASSERT(obj_b1 == NULL || obj_b2 == NULL);
  cache_obj_t *obj_ghost = obj_b1 ? obj_b1 : obj_b2;
  // 确保对象不会同时出现在数据缓存和ghost缓存中
  DEBUG_ASSERT(obj == NULL || obj_ghost == NULL);

  // 如果对象既不在数据缓存也不在ghost缓存中，返回NULL
  if (obj == NULL && obj_ghost == NULL) {
    return NULL;
  }

  // 重置ghost缓存标志
  params->curr_obj_in_L1_ghost = false;
  params->curr_obj_in_L2_ghost = false;

  // 获取B1和B2的当前大小，用于调整参数p
  int64_t b1_size = params->B1->get_occupied_byte(params->B1);
  int64_t b2_size = params->B2->get_occupied_byte(params->B2);

  if (obj_ghost != NULL) {
    // 记录当前请求时间，表示在ghost缓存中命中
    params->vtime_last_req_in_ghost = cache->n_req;
    // 缓存未命中，但在ghost缓存中命中
    if (obj_b1 != NULL) {
      // 对象在L1的ghost缓存(B1)中
      params->curr_obj_in_L1_ghost = true;
      // 情况II：对象在L1_ghost中，增加参数p
      // delta计算：取B2大小除以B1大小和1中的较大值
      double delta = MAX((double)b2_size / b1_size, 1);
      // 增加p值，但不超过缓存大小
      params->p = MIN(params->p + delta, cache->cache_size);
      // 从B1中移除对象
      params->B1->remove(params->B1, obj_b1->obj_id);
    } else {
      // 对象在L2的ghost缓存(B2)中
      params->curr_obj_in_L2_ghost = true;
      // 情况III：对象在L2_ghost中，减少参数p
      // delta计算：取B1大小除以B2大小和1中的较大值
      double delta = MAX((double)b1_size / b2_size, 1);
      // 减少p值，但不小于0
      params->p = MAX(params->p - delta, 0);
      // 从B2中移除对象
      params->B2->remove(params->B2, obj_b2->obj_id);
    }
#ifdef QUICK_DEMOTION
    // 快速降级模式：将p值限制为缓存大小的1/10
    // params->p = MIN(params->p, cache->cache_size/10);
    params->p = cache->cache_size / 10;
#endif
  } else {
    // 缓存命中，情况I：对象在T1或T2中
    if (obj_t1 != NULL) {
      // 对象在T1中，将频率标记设为1，表示已多次访问
      obj_t1->misc.freq = 1;
#ifndef LAZY_PROMOTION
      // 非延迟提升模式：将对象从T1移动到T2
      params->T1->remove(params->T1, obj_t1->obj_id);
      params->T2->get(params->T2, req);
#endif
    } else {
      // 对象在T2中，更新其位置到T2的头部
      params->T2->find(params->T2, req, true);
    }
  }

  return obj;
}

/**
 * @brief 将对象插入到缓存中，更新哈希表和缓存元数据
 * 此函数假设缓存有足够的空间，在调用此函数前应该执行淘汰操作
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 插入的对象指针
 */
static cache_obj_t *ARCv0_insert(cache_t *cache, const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  cache_obj_t *obj = NULL;

  // 检查是否是ghost缓存命中后的插入操作
  if (params->vtime_last_req_in_ghost == cache->n_req &&
      (params->curr_obj_in_L1_ghost || params->curr_obj_in_L2_ghost)) {
    // 如果对象之前在ghost缓存中被访问过，直接插入到T2（多次访问队列）的头部
    // 这是因为ghost缓存命中表明该对象有重复访问的可能性
    obj = params->T2->insert(params->T2, req);

    // 重置ghost缓存相关标志
    params->curr_obj_in_L1_ghost = false;
    params->curr_obj_in_L2_ghost = false;
    params->vtime_last_req_in_ghost = -1;
  } else {
    // 对于首次访问的对象，插入到T1（最近访问队列）的头部
    obj = params->T1->insert(params->T1, req);
    // 设置频率为0，表示首次访问
    obj->misc.freq = 0;
  }

  return obj;
}

/**
 * @brief 找出将要被淘汰的对象
 * 此函数不会实际淘汰对象或更新元数据，仅确定淘汰候选对象
 * 并非所有淘汰算法都支持此函数，因为有些算法的淘汰逻辑无法与查找淘汰候选对象分离
 * 如果无法支持此函数，应使用assert(false)
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 将要被淘汰的对象
 */
static cache_obj_t *ARCv0_to_evict(cache_t *cache, const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  // 记录生成淘汰候选对象的时间
  cache->to_evict_candidate_gen_vtime = cache->n_req;
  
  // 根据不同情况选择淘汰策略
  if (params->vtime_last_req_in_ghost == cache->n_req &&
      (params->curr_obj_in_L1_ghost || params->curr_obj_in_L2_ghost)) {
    // 情况1：当前请求在ghost缓存中命中，使用替换策略
    // 这种情况下，我们需要为新对象腾出空间，同时考虑ghost缓存的命中情况
    cache->to_evict_candidate = _ARCv0_to_replace(cache, req);
  } else {
    // 情况2：当前请求在所有队列中都未命中，使用常规淘汰策略
    cache->to_evict_candidate = _ARCv0_to_evict_miss_on_all_queues(cache, req);
  }
  return cache->to_evict_candidate;
}

/**
 * @brief 从缓存中淘汰一个对象
 * 在返回前需要调用cache_evict_base函数，该函数会更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param req 请求对象
 */
static void ARCv0_evict(cache_t *cache, const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  
  // 根据不同情况选择淘汰策略
  if (params->vtime_last_req_in_ghost == cache->n_req &&
      (params->curr_obj_in_L1_ghost || params->curr_obj_in_L2_ghost)) {
    // 情况1：当前请求在ghost缓存中命中
    // 这种情况下，我们需要调整T1和T2的大小，并从适当的列表中淘汰对象
    _ARCv0_replace(cache, req);
  } else {
    // 情况2：当前请求在所有队列中都未命中
    // 使用常规淘汰策略，可能需要从ghost列表中淘汰对象以维持大小限制
    _ARCv0_evict_miss_on_all_queues(cache, req);
  }
  
  // 重置淘汰候选对象的生成时间，表示已完成淘汰操作
  cache->to_evict_candidate_gen_vtime = -1;
}

/**
 * @brief 从缓存中移除指定对象
 * 这与缓存淘汰(evict)操作不同，移除是由用户触发的操作，而淘汰是缓存自动执行以为新对象腾出空间
 *
 * 在返回前需要调用cache_remove_obj_base函数，该函数会更新一些元数据，如对象数量、占用大小和哈希表
 *
 * @param cache 缓存对象
 * @param obj_id 要移除的对象ID
 * @return 如果对象被成功移除返回true，如果对象不在缓存中返回false
 */
static bool ARCv0_remove(cache_t *cache, const obj_id_t obj_id) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  bool removed = false;
  
  // 尝试从T1和T2中移除对象，如果任一操作成功，则返回true
  // 使用逻辑OR操作(|=)累积移除结果
  removed |= params->T1->remove(params->T1, obj_id);
  removed |= params->T2->remove(params->T2, obj_id);

  return removed;
}

/**
 * @brief 获取缓存当前占用的总字节数
 * 计算T1和T2两个数据缓存列表占用的总字节数
 *
 * @param cache 缓存对象
 * @return 缓存占用的总字节数
 */
static int64_t ARCv0_get_occupied_byte(const cache_t *cache) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  // 返回T1和T2缓存列表占用的总字节数之和
  return params->T1->get_occupied_byte(params->T1) +
         params->T2->get_occupied_byte(params->T2);
}

/**
 * @brief 获取缓存中当前存储的对象数量
 * 计算T1和T2两个数据缓存列表中的对象总数
 *
 * @param cache 缓存对象
 * @return 缓存中的对象总数
 */
static int64_t ARCv0_get_n_obj(const cache_t *cache) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  // 返回T1和T2缓存列表中对象数量之和
  return params->T1->get_n_obj(params->T1) + params->T2->get_n_obj(params->T2);
}

// ***********************************************************************
// ****                                                               ****
// ****                  cache internal functions                     ****
// ****                                                               ****
// ***********************************************************************
/**
 * @brief 找出要替换的对象但不执行实际淘汰操作
 * 这是ARC算法的核心部分，根据参数p和当前状态决定从T1还是T2中淘汰对象
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 将要被淘汰的对象
 */
static cache_obj_t *_ARCv0_to_replace(cache_t *cache, const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  cache_obj_t *obj = NULL;
  int64_t t1_size = params->T1->get_occupied_byte(params->T1);

  if (t1_size > 0 && (t1_size > params->p ||
                      (t1_size == params->p && params->curr_obj_in_L2_ghost))) {
    // delete the LRU in L1 data, move to L1_ghost
    obj = params->T1->to_evict(params->T1, req);
  } else {
    // delete the item in L2 data, move to L2_ghost
    obj = params->T2->to_evict(params->T2, req);
  }
  DEBUG_ASSERT(obj != NULL);
  return obj;
}

/**
 * @brief 实现论文中的REPLACE函数，执行实际的替换操作
 * 根据参数p和当前状态从T1或T2中淘汰对象，并将其移动到相应的ghost列表
 *
 * @param cache 缓存对象
 * @param req 请求对象
 */
static void _ARCv0_replace(cache_t *cache, const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  // 获取T1和T2的当前大小
  int64_t t1_size = params->T1->get_occupied_byte(params->T1);
  int64_t t2_size = params->T2->get_occupied_byte(params->T2);

  // 定义决策条件
  bool cond1 = t1_size > 0;                                   // T1不为空
  bool cond2 = t1_size > params->p;                           // T1大小大于参数p
  bool cond3 = t1_size == params->p && params->curr_obj_in_L2_ghost; // T1大小等于p且当前请求在L2 ghost中命中
  bool cond4 = t2_size == 0;                                  // T2为空

  // 决定从T1还是T2中淘汰对象
  if ((cond1 && (cond2 || cond3)) || cond4) {
    // 从T1中淘汰最近最少使用的对象
    cache_obj_t *obj = params->T1->to_evict(params->T1, req);
    DEBUG_ASSERT(obj != NULL);
    // 将对象信息复制到本地请求对象中，用于后续操作
    copy_cache_obj_to_request(params->req_local, obj);
    
#ifdef LAZY_PROMOTION
    // 延迟提升模式：根据对象的访问频率决定去向
    if (obj->misc.freq > 0) {
      // 如果对象已被多次访问，将其移动到T2（多次访问队列）
      params->T2->get(params->T2, params->req_local);
    } else {
      // 如果对象仅被访问一次，将其移动到B1（T1的ghost列表）
      params->B1->get(params->B1, params->req_local);
    }
#else
    // 非延迟提升模式：直接将对象移动到B1（T1的ghost列表）
    params->B1->get(params->B1, params->req_local);
#endif
    // 从T1中实际淘汰对象
    params->T1->evict(params->T1, req);
  } else {
    // 从T2中淘汰最近最少使用的对象
    cache_obj_t *obj = params->T2->to_evict(params->T2, req);
    DEBUG_ASSERT(obj != NULL);
    // 将对象信息复制到本地请求对象中，用于后续操作
    copy_cache_obj_to_request(params->req_local, obj);
    // 从T2中实际淘汰对象
    params->T2->evict(params->T2, req);
    // 将对象移动到B2（T2的ghost列表）
    params->B2->get(params->B2, params->req_local);
  }
}

/**
 * @brief 在所有队列中都未命中的情况下找出要淘汰的对象，但不执行实际淘汰操作
 * 根据T1、B1的大小和即将插入对象的大小决定淘汰策略
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 将要被淘汰的对象
 */
static cache_obj_t *_ARCv0_to_evict_miss_on_all_queues(cache_t *cache,
                                                       const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  // 获取T1和B1的当前大小
  int64_t t1_size = params->T1->get_occupied_byte(params->T1);
  int64_t b1_size = params->B1->get_occupied_byte(params->B1);

  // 计算即将插入对象的总大小（包括元数据）
  int64_t incoming_size = req->obj_size + cache->obj_md_size;
  
  // 判断T1和B1的总大小加上新对象是否超过缓存容量
  if (t1_size + b1_size + incoming_size > cache->cache_size) {
    // 情况A：L1 = T1 ∪ B1 的大小已经达到或超过缓存容量c
    if (b1_size > 0) {
      // 如果B1不为空，使用替换策略
      return _ARCv0_to_replace(cache, req);
    } else {
      // 如果B1为空，说明T1已经占用了全部缓存空间，直接从T1淘汰
      return params->T1->to_evict(params->T1, req);
    }
  } else {
    // 情况B：L1 = T1 ∪ B1 的大小未达到缓存容量c，使用替换策略
    return _ARCv0_to_replace(cache, req);
  }
}

/**
 * @brief 实现论文中的情况IV，处理在所有队列中都未命中的情况下的淘汰操作
 * 根据T1、B1的大小和即将插入对象的大小决定淘汰策略
 *
 * @param cache 缓存对象
 * @param req 请求对象
 */
static void _ARCv0_evict_miss_on_all_queues(cache_t *cache,
                                            const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  // 获取T1和B1的当前大小
  int64_t t1_size = params->T1->get_occupied_byte(params->T1);
  int64_t b1_size = params->B1->get_occupied_byte(params->B1);

  // 计算即将插入对象的总大小（包括元数据）
  int64_t incoming_size = req->obj_size + cache->obj_md_size;
  
  // 判断T1和B1的总大小加上新对象是否超过缓存容量
  if (t1_size + b1_size + incoming_size > cache->cache_size) {
    // 情况A：L1 = T1 ∪ B1 的大小已经达到或超过缓存容量c
    if (b1_size > 0) {
      // 如果B1不为空（即T1 < c），从B1中淘汰最近最少使用的对象
      // 注意：我们不使用t1_size < cache->cache_size作为条件
      // 因为对于可变大小的对象，这种判断方式不适用
      params->B1->evict(params->B1, req);
      // 然后使用替换策略淘汰一个对象
      return _ARCv0_replace(cache, req);
    } else {
      // 如果B1为空（即T1 >= c），说明T1已经占用了全部缓存空间
      // 直接从T1中淘汰对象
#ifdef LAZY_PROMOTION
      // 延迟提升模式：根据对象的访问频率决定去向
      cache_obj_t *obj = params->T1->to_evict(params->T1, req);
      DEBUG_ASSERT(obj != NULL);
      // 将对象信息复制到本地请求对象中，用于后续操作
      copy_cache_obj_to_request(params->req_local, obj);
      // 如果对象已被多次访问，将其移动到T2（多次访问队列）
      if (obj->misc.freq > 0) {
        params->T2->get(params->T2, params->req_local);
      }
      // 从T1中实际淘汰对象
      return params->T1->evict(params->T1, req);
#else
      // 非延迟提升模式：直接从T1中淘汰对象
      return params->T1->evict(params->T1, req);
#endif
    }
  } else {
    // 情况B：L1 = T1 ∪ B1 的大小未达到缓存容量c
    int64_t t2_size = params->T2->get_occupied_byte(params->T2);
    // 确保T1和B1的总大小小于缓存容量
    DEBUG_ASSERT(t1_size + b1_size < cache->cache_size);
    
    // 如果所有列表（T1、B1、T2、B2）的总大小超过缓存容量的两倍
    // 从B2（T2的ghost列表）中淘汰对象，直到总大小小于缓存容量的两倍
    while (t1_size + b1_size + t2_size +
               params->B2->get_occupied_byte(params->B2) >=
           cache->cache_size * 2) {
      // 从B2中淘汰最近最少使用的对象
      params->B2->evict(params->B2, req);
    }
    // 使用替换策略淘汰一个对象
    return _ARCv0_replace(cache, req);
  }
}

// ***********************************************************************
// ****                                                               ****
// ****                parameter set up functions                     ****
// ****                                                               ****
// ***********************************************************************
static const char *ARCv0_current_params(ARCv0_params_t *params) {
  static __thread char params_str[128];
  snprintf(params_str, 128, "\n");
  return params_str;
}

static void ARCv0_parse_params(cache_t *cache,
                               const char *cache_specific_params) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  char *params_str = strdup(cache_specific_params);
  char *old_params_str = params_str;

  while (params_str != NULL && params_str[0] != '\0') {
    /* different parameters are separated by comma,
     * key and value are separated by = */
    char *key = strsep((char **)&params_str, "=");
    // char *value = strsep((char **)&params_str, ",");

    // skip the white space
    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    if (strcasecmp(key, "print") == 0) {
      printf("parameters: %s\n", ARCv0_current_params(params));
      exit(0);
    } else {
      ERROR("%s does not have parameter %s\n", cache->cache_name, key);
      exit(1);
    }
  }

  free(old_params_str);
}

// ***********************************************************************
// ****                                                               ****
// ****                       debug functions                         ****
// ****                                                               ****
// ***********************************************************************
static void print_cache(cache_t *cache) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);
  printf("T1: ");
  params->T1->print_cache(params->T1);
  printf("T2: ");
  params->T2->print_cache(params->T2);
  printf("B1: ");
  params->B1->print_cache(params->B1);
  printf("B2: ");
  params->B2->print_cache(params->B2);
}

static bool ARCv0_get_debug(cache_t *cache, const request_t *req) {
  ARCv0_params_t *params = (ARCv0_params_t *)(cache->eviction_params);

  cache->n_req += 1;

  printf("%ld obj_id %ld: p %.2lf\n", (long)cache->n_req, (long)req->obj_id,
         params->p);
  print_cache(cache);
  printf("==================================\n");

  cache_obj_t *obj = cache->find(cache, req, true);

  if (obj != NULL) {
    return true;
  }

  while (cache->get_occupied_byte(cache) + req->obj_size + cache->obj_md_size >
         cache->cache_size) {
    cache->evict(cache, req);
  }

  cache->insert(cache, req);

  return false;
}

#ifdef __cplusplus
}
#endif
