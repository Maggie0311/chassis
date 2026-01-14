# [模块名称] API文档

## API概览
### 函数分类
- **初始化函数**: UI模块初始化和注册UI对象
- **控制函数**: 查找待发送的UI模块元素并发送至裁判系统/对已注册的UI对象进行发送频率和发送方式的修改
- **数据函数**: 从云台端得到当前绘制UI需要的具体数据，并在回调函数中去进行UI元素的绘制
- **注册函数**: 注册某个待发送元素的类型、颜色、发送方式、间隔时间、回调函数
- **回调函数**: 一个元素就有一个对应的回调函数，回调函数中会把该元素的位置、颜色、图层、数值更新

## 详细API说明

### 初始化配置 API

#### `UI_Init`
`rt_err_t UI_Init(void)`:UI模块初始化和注册UI对象

 >此函数完成以下操作：
 >1. 创建UI线程，并初始化信号量
 >2. 创建线程定时器，在定时器中断函数中释放信号量，用于线程间同步
 >3. 注册UI对象V
 >
 > 参数 ： void  
 > 返回值：  rt_err_t  
 >RT_EOK: 初始化成功
 >其他 ：初始化失败，返回具体的错误类型
 >RT_ERROR : 通用错误 
 >RT_ETIMEOUT : 超时错误 
 >RT_EFULL : 资源已满 
 >RT_EEMPTY : 资源为空
 >RT_ENOMEM : 内存不足 
 >RT_ENOSYS ：系统不支持 
 >RT_EBUSY ：系统繁忙
 >RT_EIO ：IO错误 
 >RT_EINTR ：中断系统调用 
 >RT_EINVAL ： 非法参数 
 >
 >注意：此函数必须在main.c文件中调用
#### `Attach_UI_init`
```c
void Attach_UI_init();
/**
 * @brief 注册全部待发送的UI对象，这里的对象是单个直线/圆/矩形/圆弧/整形/浮点型/字符串
 * 
 * @details
 * UI对象是单个直线/圆/矩形/圆弧/整形/浮点型/字符串，在上层我们直接调用这个函数时，底层会自动把每一个对象所对应的
 * 内容（包括对象类型、裁判系统索引、图层、颜色、发送方式、发送周期、回调函数等）写到object[]数组中和
 * UI_trans_ctrl_info[] UI控制器数组中，用于在线程中调用回调函数，通过串口向裁判系统发送UI相关消息
 * 
 *  @param void
 *  @return void
 * 
 * @note 该函数是通过调用Attach_UI_Object()函数来进行具体实现，下面将介绍Attach_UI_Object()函数的用法
 */
```
### 数据操作 API
#### `Get_UI_Data()`
```c
void Get_UI_Data(void);
/**
 * @brief 从云台端获取UI绘制所需数据，并写入UiExData结构体中（全局变量）
 * 
 * @param void
 * @return void 
 * 
 * @note 
 * (1)该函数中调用多个Get函数，注意接口的预留，且不同的机器人需要用到的数据不同，可以根据自身需求
 * 修改或添加。
 * (2)该函数中有一些较为特殊的函数，主要是用于应对触发式发送（时而需要显示在屏幕上，时而无需显示，
 * 例如卡弹标志位/自瞄状态）的元素，通过判断标志位是否改变，来确定该元素此时需要设置为可见/不可见
 * 需要设置为可见的元素，设置操作类型为空UI_NULL，在底层发送前会自动设置为创建UI_CREATE
 * 需要设置为不可见的元素，设置操作类型为空UI_DELETE，在底层发送前会设置操作类型为删除
 * 检查堵弹状态变化并更新UI显示，例如堵弹状态改变的函数（如下）
 */
   void UI_If_Stuck_Change()
   {
       static uint8_t last_strike_stuck = 0; // 卡弹上次保存状态
       UI_Checkout_Object_Change(UiExData.Strike_Stuck, last_strike_stuck, 1, UI_StrikeStuck_Info);
       last_strike_stuck= UiExData.Strike_Stuck; // 更新卡弹状态，堵弹时标志位为1
   }
/**调用的UI_Checkout_Object_Change()函数用于判断标志位是否发生改变
 * 若发生改变，则判断是否与需要显示的值相等，相等，就设置为可见；不等，就设置为不可见
 * 还有一些待耦合的UI数据信息（视觉是否正常工作），之后可能要修改？
```


#### `Drv_Find_Send_UI_Object`
```c 
void Drv_Find_Send_UI_Object();
/**
 * @brief 该函数用来查找并发送需要发送的UI元素（包括字符串和图形元素）, 调用回调函数去更新元素相关信息，
 * 然后字符串与图形分别打包并发送至串口
 * 
 * @param void
 * @return void
 * 
 * @note 在ui线程中调用
 */
```

#### `Drv_Send_UI_Object`
```c
Drv_Send_UI_Object(index_graph, index_string);
/**
 * @brief 该函数用来查找并发送需要发送的UI元素（包括字符串和图形元素）, 调用回调函数去更新元素相关信息，
 * 然后字符串与图形分别打包并发送至串口
 * 
 * @param void
 * @return void
 * 
 * @note 在ui线程中调用
 */

```

### 状态控制 API
#### `module_start`
```c
/**
 * @brief 启动模块运行
 * 
 * @param timeout_ms 启动超时时间
 * @return module_status_t 
 *   - MODULE_RUNNING: 启动成功
 *   - MODULE_ERROR: 启动失败
 *   - MODULE_BUSY: 模块忙
 * 
 * @note 启动后模块开始处理数据
 */
module_status_t module_start(uint32_t timeout_ms);
```
#### `module_stop`
```c 
/**
 * @brief 停止模块运行
 * 
 * @param flush_data 是否清空待处理数据
 *   - true: 清空所有待处理数据
 *   - false: 处理完当前数据后停止
 * @return module_status_t 停止后的状态
 */
module_status_t module_stop(bool flush_data);
```
### 回调函数 API
- 事件回调注册
```c 
/**
 * @brief 事件回调函数类型定义
 * 
 * @param event_type 事件类型
 * @param event_data 事件相关数据
 * @param user_data 用户自定义数据
 */
typedef void (*module_event_cb_t)(module_event_t event_type, 
                                  const void *event_data, 
                                  void *user_data);

/**
 * @brief 注册事件回调函数
 * 
 * @param event_type 要监听的事件类型
 * @param callback 回调函数指针
 * @param user_data 用户数据，会传递给回调函数
 * @return int32_t 
 *   - 0: 注册成功
 *   - -1: 参数错误
 *   - -2: 事件类型不支持
 * 
 * @note 同一事件类型可注册多个回调函数
 */
int32_t module_register_callback(module_event_t event_type,
                                module_event_cb_t callback,
                                void *user_data);
```
### 数据结构说明
#### 配置结构体
```c
/**
 * @brief 模块配置参数结构体
 */
typedef struct {
    uint32_t sample_rate;          ///< 采样率，单位Hz，范围10-10000
    uint8_t data_bits;            ///< 数据位宽，支持8/16/32
    bool enable_crc;              ///< CRC校验使能
    float calibration_offset;     ///< 校准偏移量
    uint16_t buffer_size;         ///< 内部缓冲区大小
    operation_mode_t mode;        ///< 运行模式
} module_config_t;
```
#### 状态结构体
```c
/**
 * @brief 模块实时状态结构体
 */
typedef struct {
    module_status_t status;        ///< 当前状态
    uint32_t data_processed;      ///< 已处理数据量
    uint32_t error_count;         ///< 错误计数
    float cpu_usage;              ///< CPU使用率(%)
    uint32_t queue_usage;         ///< 队列使用情况
    timestamp_t last_update;      ///< 最后更新时间戳
} module_status_info_t;
```
### 使用示例
#### 基础使用
```c
#include "module_api.h"

// 配置参数
module_config_t config = {
    .sample_rate = 1000,
    .data_bits = 16,
    .enable_crc = true,
    .buffer_size = 1024,
    .mode = HIGH_PERFORMANCE_MODE
};

// 初始化
if (module_init(&config) != 0) {
    // 错误处理
    return -1;
}

// 启动模块
if (module_start(5000) != MODULE_RUNNING) {
    module_deinit();
    return -1;
}

// 数据处理循环
while (running) {
    module_send_data(tx_data, data_size, 100);
    // ... 其他操作
}

// 停止和清理
module_stop(false);
module_deinit();
```
#### 事件回调使用
```c 
void my_event_handler(module_event_t event, 
                     const void *data, 
                     void *user_data) {
    switch (event) {
        case MODULE_EVENT_DATA_READY:
            printf("Data ready event received\n");
            break;
        case MODULE_EVENT_ERROR:
            printf("Error event: %s\n", (const char*)data);
            break;
    }
}

// 注册回调
module_register_callback(MODULE_EVENT_DATA_READY, 
                        my_event_handler, 
                        NULL);
```
### 错误码说明

|错误码|	宏定义|	说明	|建议处理|
|-----|----------|---------|--------|
|0	|MODULE_SUCCESS	|操作成功|	-|
|-1	|MODULE_ERROR_PARAM	|参数错误	|检查输入参数|
|-2	|MODULE_ERROR_STATE	|状态错误	|检查模块状态机|
|-3	|MODULE_ERROR_MEMORY	|内存不足	|检查系统内存|
|-4	|MODULE_ERROR_HARDWARE	|硬件错误	|检查硬件连接|
|-5	|MODULE_ERROR_TIMEOUT	|操作超时	|增加超时时间或检查系统负载|