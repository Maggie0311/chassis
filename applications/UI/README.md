# UI/ 目录

裁判系统 UI 绘制模块。

## 文件清单

- `app_UI.c` - UI 刷新线程，绘制模式、功率、电容电压等信息
- `drv_draw_UI.c` - 图形元素绘制 API
- `drv_send_UI.c` - 底层 UI 数据包发送
- `CustomUI_GraphParameters.h` - UI 绘制参数配置

## 模块文档

详见 [UI 模块文档](UI.md)

## 功能说明

通过裁判系统向操作手客户端发送自定义 UI，显示机器人实时状态，帮助操作手做出决策。
