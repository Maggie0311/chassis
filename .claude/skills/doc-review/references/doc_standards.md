# 文档规范详细说明

## 文档层级体系（三层结构）

### L0：根入口文档
- **文件名**：`CLAUDE.md`（固定）
- **位置**：项目根目录
- **内容**：任务分类入口，指向 5 个 L1 任务类型指引
- **约束**：
  - 只包含任务分类和对应的 L1 文档链接
  - 不包含具体操作步骤
  - 不直接链接 L2 文档

**示例结构**：
```markdown
# 项目协作指引

根据你要执行的任务类型，查看对应指引：

- [编译构建指引](docs/build_guide.md)
- [代码探索指引](docs/code_exploration.md)
- [新代码开发指引](docs/development_guide.md)
- [测试验收指引](docs/testing_guide.md)
- [命令执行指引](docs/command_guide.md)
```

### L1：任务类型指引文档
- **数量**：5 个固定类型
- **命名规范**：`{任务类型}_guide.md`
- **位置**：`docs/` 目录
- **内容**：该任务类型的总体说明和 L2 文档索引
- **约束**：
  - 只包含典型操作的简要说明
  - 详细内容、原理分析、踩坑记录等放在 L2 文档
  - 可以包含简单的命令示例（1-2 行）

**5 个任务类型**：

#### 1. 编译构建指引 (build_guide.md)
```markdown
# 编译构建指引

## 典型编译命令
- Keil MDK：打开 `MDK-ARM/f334_buck_boost.uvprojx`，点击 Build
- 命令行：`cd MDK-ARM && uvbuild f334_buck_boost.uvprojx`

## 详细文档
- [编译原理分析](build/build_process.md)
- [常见编译错误](build/build_errors.md)
- [优化编译速度](build/build_optimization.md)
```

#### 2. 代码探索指引 (code_exploration.md)
```markdown
# 代码探索指引

代码按目录组织，每个目录都有对应的探索指引：

- [applications/ - 应用层代码](../applications/README.md)
- [drivers/ - 驱动代码](../drivers/README.md)
- [board/ - 板级配置](../board/README.md)

## 探索方法
- 每个目录的 README.md 说明该目录的作用和文件清单
- 独立模块有单独的设计文档或使用手册
```

#### 3. 新代码开发指引 (development_guide.md)
```markdown
# 新代码开发指引

## 新增代码模块时的操作
- [将文件加入编译](dev/add_to_build.md)
- [增加日志打印](dev/logging.md)
- [使用 RT-Thread API](dev/rtthread_api.md)
```

#### 4. 测试验收指引 (testing_guide.md)
```markdown
# 测试验收指引

## 测试流程
1. 编译并烧录固件
2. 按测试文档执行测试用例
3. 记录测试结果

## 测试文档位置
- 功能测试：`tests/functional/`
- 性能测试：`tests/performance/`
```

#### 5. 命令执行指引 (command_guide.md)
```markdown
# 命令执行指引

## 命令类型
- [Python 命令注意事项](commands/python_commands.md)
- [编译命令注意事项](commands/build_commands.md)
- [Bash 命令注意事项](commands/bash_commands.md)
```

### L2：具体操作文档
- **命名规范**：`{主题}_xxx.md`（使用下划线分隔）
- **位置**：按主题组织在子目录中
- **内容**：详细的操作步骤、原理分析、踩坑记录
- **约束**：
  - 可以继续引用其他 L2 文档（但建议深度 ≤3 层）
  - 必须在文档头部说明"从哪个 L1 文档链接过来"

**示例**：
```markdown
# 编译原理分析

> 从 [编译构建指引](../build_guide.md) 链接

## Keil MDK 编译流程
1. 预处理：展开宏定义和头文件
2. 编译：生成 .o 目标文件
3. 链接：生成 .axf 可执行文件
...
```

## 目录内探索指引规范

每个代码目录应包含 `README.md`，说明：
1. 该目录的作用
2. 目录内文件/子目录的清单和简要说明
3. 强关联代码的位置（如果有）

**简单模块示例**（直接列出文件）：
```markdown
# applications/ 目录

应用层代码，包含主函数和业务逻辑。

## 文件清单
- `main.c` - 主函数，初始化系统
- `func_powerctrl.c` - 功率控制逻辑
- `func_wireless.c` - 无线充电控制

## 强关联代码
- 功率控制依赖 `drivers/adc_driver.c` 的 ADC 采样
```

**复杂模块示例**（拆分子文档）：
```markdown
# drivers/ 目录

硬件驱动代码。

## 子目录
- `adc/` - ADC 驱动，详见 [ADC 驱动说明](adc/README.md)
- `pwm/` - PWM 驱动，详见 [PWM 驱动说明](pwm/README.md)
- `uart/` - UART 驱动，详见 [UART 驱动说明](uart/README.md)

## 强关联代码
- 驱动被 `applications/` 中的业务逻辑调用
```

## 独立模块的设计文档

如果目录包含相对独立的代码模块，应在同目录下创建设计文档或使用手册：
- **设计文档**：`{模块名}_design.md` - 说明架构、接口、实现原理
- **使用手册**：`{模块名}_usage.md` - 说明如何使用该模块的 API

**示例**：
```
applications/
├── README.md
├── func_powerctrl.c
├── func_powerctrl.h
└── power_control_design.md  # 功率控制模块设计文档
```

## 文档元数据块（可选）

在文档开头可以添加状态元数据：
```markdown
---
status: Active | Draft | Deprecated
last_updated: 2026-03-28
related_code: applications/func_powerctrl.c
---
```

## 命名规范

### 文件名
- 使用小写字母和下划线：`build_guide.md`、`power_control_design.md`
- 避免使用空格和特殊字符
- 中文项目可以使用拼音或英文

### 链接格式
- 相对路径：`[文档标题](../path/to/doc.md)`
- 锚点链接：`[章节标题](doc.md#章节标题)`

## 引用链路规范

### 层数计算
- L0 → L1：1 层
- L1 → L2：2 层
- L2 → L2：继续累加

### 深度建议
- **合格**：≤3 层
- **警告**：4-5 层（考虑重构）
- **不合格**：>5 层（必须重构）

### 断链检查
- 链接指向的文件必须存在
- 锚点链接指向的章节必须存在

### 孤岛文档
- 除 CLAUDE.md 外，所有文档都应该被至少一个文档引用
- 孤岛文档应该删除或补充引用

