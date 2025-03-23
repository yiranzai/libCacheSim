//
//  W-TinyLFU 缓存替换算法实现
//
//  算法特点：
//  - 使用近似LFU结构(TinyLFU)作为准入控制机制
//  - 可与任何驱逐策略结合使用
//  - 结合窗口化LRU结构(Windowed LRU)
//  - 能够抵抗临时热点对象的突发流量
//
//  工作原理：
//  - 使用计数布隆过滤器(CBF)高效跟踪对象访问频率
//  - 维护一个小型窗口缓存(window cache)和一个主缓存(main cache)
//  - 新对象首先进入窗口缓存
//  - 窗口缓存满时，将对象提升到主缓存前进行频率比较
//  - 只有频率较高的对象才能进入主缓存，有效过滤"一次性"访问
//
//  注意事项：
//  - 此实现不是线程安全的
//  - 部分实现细节参考自 https://github.com/mandreyel/w-tinylfu
//
//  WTinyLFU.c
//  libCacheSim
//
//  创建者: Ziyue
//  创建日期: 2023年1月14日
//

#include "../../dataStructure/hashtable/hashtable.h"
#include "../../dataStructure/minimalIncrementCBF.h"
#include "../../include/libCacheSim/evictionAlgo.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_MODE
#undef DEBUG_MODE
#define DEBUG_MODE_2
#undef DEBUG_MODE_2

// WTinyLFU算法参数结构体，存储算法运行所需的所有状态和配置
typedef struct WTinyLFU_params {
  cache_t *LRU;         // LRU作为窗口缓存，新对象首先进入此缓存
  cache_t *main_cache;  // 主缓存，可以是任何驱逐策略
  double window_size;   // 窗口缓存占总缓存大小的比例
  int64_t n_admit_bytes; // 记录从窗口缓存提升到主缓存的字节数
  struct minimalIncrementCBF *CBF; // 计数布隆过滤器，用于跟踪对象访问频率
  size_t max_request_num;  // CBF衰减前处理的最大请求数
  size_t request_counter;  // 当前已处理的请求计数
  char main_cache_type[32]; // 主缓存使用的算法类型

  request_t *req_local; // 本地请求对象，用于内部操作
} WTinyLFU_params_t;

// 默认参数配置：主缓存使用SLRU，窗口大小为总缓存的1%
static const char *DEFAULT_PARAMS = "main-cache=SLRU,window-size=0.01";

// ***********************************************************************
// ****                                                               ****
// ****                   function declarations                       ****
// ****                                                               ****
// ***********************************************************************

static void WTinyLFU_free(cache_t *cache);
static cache_obj_t *WTinyLFU_find(cache_t *cache, const request_t *req,
                                  const bool update);
static bool WTinyLFU_get(cache_t *cache, const request_t *req);
static cache_obj_t *WTinyLFU_insert(cache_t *cache, const request_t *req);
static cache_obj_t *WTinyLFU_to_evict(cache_t *cache, const request_t *req);
static void WTinyLFU_evict(cache_t *cache, const request_t *req);
static bool WTinyLFU_remove(cache_t *cache, const obj_id_t obj_id);
static void WTinyLFU_parse_params(cache_t *cache,
                                  const char *cache_specific_params);

bool WTinyLFU_can_insert(cache_t *cache, const request_t *req);
static int64_t WTinyLFU_get_occupied_byte(const cache_t *cache);
static int64_t WTinyLFU_get_n_obj(const cache_t *cache);

// ***********************************************************************
// ****                                                               ****
// ****                   end user facing functions                   ****
// ****                                                               ****
// ****                       init, free, get                         ****
// ***********************************************************************

/**
 * @brief 初始化WTinyLFU缓存
 * 
 * 该函数创建并初始化WTinyLFU缓存结构，包括：
 * 1. 创建窗口缓存(LRU)和主缓存
 * 2. 初始化计数布隆过滤器(CBF)
 * 3. 设置各种参数和回调函数
 *
 * @param ccache_params 通用缓存参数，如缓存大小
 * @param cache_specific_params 算法特定参数，如窗口大小和主缓存类型
 * @return 初始化好的缓存对象
 */
cache_t *WTinyLFU_init(const common_cache_params_t ccache_params,
                       const char *cache_specific_params) {
  // 初始化基本缓存结构
  cache_t *cache =
      cache_struct_init("WTinyLFU", ccache_params, cache_specific_params);
  
  // 设置各种回调函数
  cache->cache_init = WTinyLFU_init;
  cache->cache_free = WTinyLFU_free;
  cache->get = WTinyLFU_get;
  cache->find = WTinyLFU_find;
  cache->insert = WTinyLFU_insert;
  cache->evict = WTinyLFU_evict;
  cache->remove = WTinyLFU_remove;
  cache->to_evict = WTinyLFU_to_evict;
  cache->can_insert = WTinyLFU_can_insert;
  cache->get_occupied_byte = WTinyLFU_get_occupied_byte;
  cache->get_n_obj = WTinyLFU_get_n_obj;

  // 分配WTinyLFU特定参数的内存
  cache->eviction_params =
      (WTinyLFU_params_t *)malloc(sizeof(WTinyLFU_params_t));
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)(cache->eviction_params);

  // 设置对象元数据大小
  if (ccache_params.consider_obj_metadata) {
    cache->obj_md_size = params->main_cache->obj_md_size;
    // TODO: not sure whether it works
  } else {
    cache->obj_md_size = 0;
  }

  // 解析参数，先应用默认参数
  WTinyLFU_parse_params(cache, DEFAULT_PARAMS);
  // 如果提供了特定参数，则覆盖默认参数
  if (cache_specific_params != NULL) {
    WTinyLFU_parse_params(cache, cache_specific_params);
  }

  // 复制通用缓存参数用于创建子缓存
  common_cache_params_t ccache_params_local = ccache_params;

  // 创建窗口缓存(LRU)，大小为总缓存的window_size比例
  ccache_params_local.cache_size *= params->window_size;
  params->LRU = LRU_init(ccache_params_local, NULL);
  
  // 创建主缓存，大小为总缓存减去窗口缓存的大小
  ccache_params_local.cache_size = ccache_params.cache_size;
  ccache_params_local.cache_size -= params->LRU->cache_size;

  // 根据配置的主缓存类型初始化主缓存
  if (strcasecmp(params->main_cache_type, "LRU") == 0) {
    params->main_cache = LRU_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "SLRU") == 0) {
    params->main_cache = SLRU_init(ccache_params_local, "seg-size=1:4");
  } else if (strcasecmp(params->main_cache_type, "LFU") == 0) {
    params->main_cache = LFU_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "FIFO") == 0) {
    params->main_cache = FIFO_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "FIFO-Reinsertion") == 0) {
    params->main_cache = FIFO_Reinsertion_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "ARC") == 0) {
    params->main_cache = ARC_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "clock") == 0) {
    params->main_cache = Clock_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LeCaR") == 0) {
    params->main_cache = LeCaR_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "Cacheus") == 0) {
    params->main_cache = Cacheus_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "Hyperbolic") == 0) {
    params->main_cache = Hyperbolic_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "LHD") == 0) {
    params->main_cache = LHD_init(ccache_params_local, NULL);
  } else if (strcasecmp(params->main_cache_type, "SIEVE") == 0) {
    params->main_cache = Sieve_init(ccache_params_local, NULL);
  } else {
    ERROR("WTinyLFU does not support %s \n", params->main_cache_type);
  }

  // 设置缓存名称，包含窗口大小和主缓存类型信息
  snprintf(cache->cache_name, CACHE_NAME_ARRAY_LEN, "WTinyLFU-w%.2lf-%s",
           params->window_size, params->main_cache_type);

  // 创建本地请求对象，用于内部操作
  params->req_local = new_request();
  params->n_admit_bytes = 0;

  // 设置CBF衰减的最大请求数，采样率为32
  params->max_request_num =
      32 * params->main_cache->cache_size;  // sample size is 32

  // 初始化计数布隆过滤器(CBF)
  params->CBF =
      (struct minimalIncrementCBF *)malloc(sizeof(struct minimalIncrementCBF));
  DEBUG_ASSERT(params->CBF != NULL);
  params->CBF->ready = 0;

  // 初始化CBF，设置条目数为主缓存大小，错误率为0.001
  int ret = minimalIncrementCBF_init(params->CBF,
                                     params->main_cache->cache_size, 0.001);
  if (ret != 0) {
    ERROR("CBF init failed\n");
  }

#ifdef DEBUG_MODE
  minimalIncrementCBF_print(params->CBF);
#endif

  // 初始化请求计数器
  params->request_counter = 0;

#if defined(TRACK_DEMOTION)
  params->LRU->track_demotion = false;
  params->main_cache->track_demotion = false;
#endif

  return cache;
}

/**
 * 释放WTinyLFU缓存使用的资源
 *
 * 该函数清理所有分配的内存，包括：
 * 1. 窗口缓存和主缓存
 * 2. 计数布隆过滤器
 * 3. 本地请求对象
 * 4. 缓存结构本身
 *
 * @param cache 要释放的缓存对象
 */
static void WTinyLFU_free(cache_t *cache) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)(cache->eviction_params);
  
  // 释放窗口缓存和主缓存
  params->LRU->cache_free(params->LRU);
  params->main_cache->cache_free(params->main_cache);

  // 释放计数布隆过滤器
  minimalIncrementCBF_free(params->CBF);
  free(params->CBF);
  
  // 释放本地请求对象
  free_request(params->req_local);

  // 释放缓存结构
  cache_struct_free(cache);
}

/**
 * 处理缓存获取请求
 *
 * 该函数是对基础缓存获取函数的包装，用于处理对象访问请求
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果对象在缓存中找到则返回true，否则返回false
 */
static bool WTinyLFU_get(cache_t *cache, const request_t *req) {
  /* because this field cannot be updated in time since segment LRUs are
   * updated, so we should not use this field */
  DEBUG_ASSERT(cache->occupied_byte == 0);

  bool ck = cache_get_base(cache, req);
  return ck;
}

/**
 * 在缓存中查找对象
 *
 * 该函数在窗口缓存和主缓存中查找请求的对象，并在找到时更新访问频率
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @param update_cache 是否更新缓存状态
 * @return 如果找到对象则返回对象指针，否则返回NULL
 */
static cache_obj_t *WTinyLFU_find(cache_t *cache, const request_t *req,
                                  const bool update_cache) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)(cache->eviction_params);

  cache_obj_t *obj_window = NULL;
  cache_obj_t *obj_main = NULL;

  // 分别在窗口缓存和主缓存中查找对象
  obj_window = params->LRU->find(params->LRU, req, update_cache);
  obj_main = params->main_cache->find(params->main_cache, req, update_cache);

  // 如果在任一缓存中找到，则返回对象
  cache_obj_t *obj = obj_window != NULL ? obj_window : obj_main;

  // 如果不需要更新缓存状态，直接返回结果
  if (!update_cache) {
    return obj;
  }

  // 如果在主缓存中找到对象，更新其访问频率
  if (obj_main != NULL) {
    // 在CBF中增加对象的频率计数
    minimalIncrementCBF_add(params->CBF, (void *)&req->obj_id, sizeof(obj_id_t));

    // 增加请求计数，当达到最大请求数时执行CBF衰减
    params->request_counter++;
    if (params->request_counter >= params->max_request_num) {
      params->request_counter = 0;
      minimalIncrementCBF_decay(params->CBF);
    }
  }

  return obj;
}

/**
 * 向缓存中插入新对象
 *
 * 该函数将新对象插入到窗口缓存中，并更新其访问频率
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 插入的缓存对象指针
 */
cache_obj_t *WTinyLFU_insert(cache_t *cache, const request_t *req) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)(cache->eviction_params);

  // 将对象插入窗口缓存
  cache_obj_t *obj = NULL;
  obj = params->LRU->insert(params->LRU, req);

  // 更新对象的访问频率
  minimalIncrementCBF_add(params->CBF, (void *)&req->obj_id, sizeof(obj_id_t));

#if defined(TRACK_DEMOTION)
  obj->create_time = cache->n_req;
#endif

  return obj;
}

/**
 * 选择要驱逐的对象
 *
 * 警告：此函数不应被使用，WTinyLFU使用自己的驱逐逻辑
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 总是返回NULL
 */
static cache_obj_t *WTinyLFU_to_evict(cache_t *cache, const request_t *req) {
  // Warning: don't use this function
  DEBUG_ASSERT(false);
  return NULL;
}

/**
 * 从缓存中驱逐对象
 *
 * 该函数实现了WTinyLFU的核心驱逐逻辑：
 * 1. 优先从窗口缓存中驱逐对象
 * 2. 当窗口缓存中的对象需要提升到主缓存时，比较其频率与主缓存中待驱逐对象的频率
 * 3. 只有频率更高的对象才能进入主缓存
 *
 * @param cache 缓存对象
 * @param req 请求对象
 */
static void WTinyLFU_evict(cache_t *cache, const request_t *req) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)(cache->eviction_params);

  cache_t *window = params->LRU;
  cache_t *main = params->main_cache;

  bool evicted = false;
  while (!evicted) {
    // 如果窗口缓存不为空，尝试从窗口缓存中驱逐
    if (window->get_occupied_byte(window) > 0) {
      // 选择窗口缓存中要驱逐的对象
      cache_obj_t *window_victim = window->to_evict(window, req);
      DEBUG_ASSERT(window_victim != NULL);

      // 将窗口缓存中的受害者对象信息复制到本地请求对象
      copy_cache_obj_to_request(params->req_local, window_victim);

      // 如果主缓存有足够空间，直接将窗口缓存中的对象提升到主缓存
      if (main->get_occupied_byte(main) + params->req_local->obj_size +
              cache->obj_md_size <=
          main->cache_size) {
        // 插入到主缓存
        main->insert(main, params->req_local);

#if defined(TRACK_DEMOTION)
        printf("%ld keep %ld %ld\n", cache->n_req, window_victim->create_time,
               window_victim->misc.next_access_vtime);
#endif
        // 记录提升到主缓存的字节数
        params->n_admit_bytes += params->req_local->obj_size;

        // 从窗口缓存中移除该对象
        window->remove(window, window_victim->obj_id);

      } else {
        // 主缓存已满，需要比较频率决定是否替换
        
        // 选择主缓存中要驱逐的对象
        cache_obj_t *main_cache_victim = main->to_evict(main, req);
        DEBUG_ASSERT(main_cache_victim != NULL);
        
        // 比较窗口缓存对象和主缓存对象的访问频率
        if (minimalIncrementCBF_estimate(params->CBF,
                                         (void *)&window_victim->obj_id,
                                         sizeof(window_victim->obj_id)) >
            minimalIncrementCBF_estimate(params->CBF,
                                         (void *)&main_cache_victim->obj_id,
                                         sizeof(main_cache_victim->obj_id))) {
          // 窗口缓存对象频率更高，替换主缓存中的对象
#if defined(TRACK_DEMOTION)
          printf("%ld keep %ld %ld\n", cache->n_req, window_victim->create_time,
                 window_victim->misc.next_access_vtime);
#endif

          // 从主缓存中驱逐对象
          main->evict(main, req);

          // 从窗口缓存中移除对象
          bool ret = window->remove(window, window_victim->obj_id);
          DEBUG_ASSERT(ret);

          // 将窗口缓存对象插入主缓存
          cache_obj_t *cache_obj = main->insert(main, params->req_local);
          params->n_admit_bytes += params->req_local->obj_size;

        } else {
          // 窗口缓存对象频率较低，直接从窗口缓存中驱逐
#if defined(TRACK_DEMOTION)
          printf("%ld demote %ld %ld\n", cache->n_req,
                 window_victim->create_time,
                 window_victim->misc.next_access_vtime);
#endif

          window->evict(window, req);
          evicted = true;
        }
      }
      // 更新被处理对象的访问频率
      minimalIncrementCBF_add(params->CBF, (void *)(&params->req_local->obj_id),
                              sizeof(obj_id_t));
    } else {
      // 窗口缓存为空，直接从主缓存中驱逐
      DEBUG_ASSERT(window->get_occupied_byte(window) == 0);
      return main->evict(main, req);
    }
  }
}

/**
 * 从缓存中移除指定对象
 *
 * 该函数尝试从窗口缓存和主缓存中移除指定ID的对象
 *
 * @param cache 缓存对象
 * @param obj_id 要移除的对象ID
 * @return 如果成功移除则返回true，否则返回false
 */
static bool WTinyLFU_remove(cache_t *cache, const obj_id_t obj_id) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)(cache->eviction_params);
  
  // 尝试从窗口缓存中移除
  if (params->LRU->remove(params->LRU, obj_id)) {
    return true;
  }
  
  // 尝试从主缓存中移除
  if (params->main_cache->remove(params->main_cache, obj_id)) {
    return true;
  }
  
  // 对象不在缓存中
  return false;
}

/**
 * 解析WTinyLFU特定的参数
 *
 * 该函数解析用户提供的参数字符串，设置WTinyLFU的配置参数
 *
 * @param cache 缓存对象
 * @param cache_specific_params 参数字符串，格式为"key1=value1,key2=value2"
 */
static void WTinyLFU_parse_params(cache_t *cache,
                                  const char *cache_specific_params) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)cache->eviction_params;

  // 复制参数字符串以便修改
  char *params_str = strdup(cache_specific_params);
  while (params_str != NULL && params_str[0] != '\0') {
    /* different parameters are separated by comma,
     * key and value are separated by = */
    char *key = strsep((char **)&params_str, "=");
    char *value = strsep((char **)&params_str, ",");

    // 跳过空格
    while (params_str != NULL && *params_str == ' ') {
      params_str++;
    }

    // 解析不同的参数
    if (strcasecmp(key, "main-cache") == 0) {
      // 设置主缓存类型
      strncpy(params->main_cache_type, value, 30);
    } else if (strcasecmp(key, "window-size") == 0) {
      // 设置窗口大小比例
      params->window_size = strtod(value, NULL);
      if (params->window_size < 0 || params->window_size >= 1) {
        ERROR("window_size must be in [0, 1)\n");
        exit(1);
      }
    } else {
      ERROR("%s does not have parameter %s\n", cache->cache_name, key);
      exit(1);
    }
  }
  return;
}

/**
 * 检查是否可以插入对象
 *
 * 该函数检查对象大小是否超过窗口缓存和主缓存的限制
 *
 * @param cache 缓存对象
 * @param req 请求对象
 * @return 如果可以插入则返回true，否则返回false
 */
bool WTinyLFU_can_insert(cache_t *cache, const request_t *req) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)cache->eviction_params;
  
  // 检查基本插入条件
  bool can_insert = cache_can_insert_default(cache, req);

  // 检查对象大小是否超过窗口缓存和主缓存的限制
  return can_insert &&
         (req->obj_size + cache->obj_md_size <= params->LRU->cache_size) &&
         (params->main_cache->can_insert(params->main_cache, req));
}

/**
 * 获取缓存已占用的字节数
 *
 * 该函数计算窗口缓存和主缓存已占用的总字节数
 *
 * @param cache 缓存对象
 * @return 已占用的字节数
 */
static int64_t WTinyLFU_get_occupied_byte(const cache_t *cache) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)cache->eviction_params;
  int64_t occupied_byte = 0;

  // 累加窗口缓存和主缓存的已占用字节数
  occupied_byte += params->LRU->occupied_byte;
  occupied_byte += params->main_cache->get_occupied_byte(params->main_cache);

  return occupied_byte;
}

/**
 * 获取缓存中的对象数量
 *
 * 该函数计算窗口缓存和主缓存中的对象总数
 *
 * @param cache 缓存对象
 * @return 缓存中的对象数量
 */
static int64_t WTinyLFU_get_n_obj(const cache_t *cache) {
  WTinyLFU_params_t *params = (WTinyLFU_params_t *)cache->eviction_params;
  int64_t n_obj = 0;

  // 累加窗口缓存和主缓存中的对象数量
  n_obj += params->LRU->n_obj;
  n_obj += params->main_cache->get_n_obj(params->main_cache);
  return n_obj;
}

#ifdef __cplusplus
}
#endif
