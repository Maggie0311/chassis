# 2026 底盘项目协作指引

本项目是基于 RT-Thread RTOS 的 RoboMaster 底盘控制系统，支持哨兵、步兵、英雄等多机型。

> **⚡ 进行中的跨线任务：RDK X5 迁移**——新会话请先读 [交接文档](docs/SESSION_HANDOFF.md) 和 [板子操作记录](docs/rdk_x5_board_log.md) 恢复上下文；对板子的任何新操作继续登记到操作记录里。
> 第一阶段（三步上手 + 1kHz 实时骨架）已完成：[实现过程实录](docs/rdk_x5_stage1_impl_log.md)（怎么一步步做出来的，含弯路）｜[跟做手册](docs/rdk_x5_stage1_tutorial.md)（提炼后的复现路径）。

根据你要执行的任务类型，查看对应指引：

- [编译构建指引](docs/build_guide.md) - 如何编译和烧录固件
- [代码探索指引](docs/code_exploration.md) - 理解代码结构和模块功能
- [新代码开发指引](docs/development_guide.md) - 添加新功能或修改现有代码
- [测试验收指引](docs/testing_guide.md) - 测试和验证功能
- [命令执行指引](docs/command_guide.md) - 执行编译、调试等命令的注意事项


## MCP工具

### PDF阅读工具（pdf-reader）

本项目配置了 `pdf-reader` MCP server，可直接在对话中读取 PDF 文件。**需要阅读数据手册、论文、规格书等 PDF 文档时，主动使用这些工具，无需让用户手动提取内容。**

可用工具：

| 工具 | 用途 |
|------|------|
| `get_pdf_info` | 获取 PDF 基本信息（页数、作者、文件大小等） |
| `read_pdf_text` | 提取文字内容，支持指定页码范围 |
| `analyze_pdf_structure` | 分析文档结构（文字页/图片页/混合页分布） |
| `extract_pdf_images` | 提取文档中的图片 |
| `read_pdf_with_ocr` | OCR 识别扫描版 PDF，中文用 `ocr_language: "chi_sim+eng"` |