# DeepSeek Harness (DSH) 安装与上手指南

> DSH 是 DeepSeek 的编码 agent harness：浏览器 GUI 里运行主 agent，主 agent 可以派生子 agent（subagent）并行干活，内置文件读写、命令执行、编译、联网搜索等工具。团队现在使用的界面（http://127.0.0.1:3080）就是它。
>
> 官方仓库：<https://github.com/deepseek-ai/deepseek-harness>

## 一、环境准备

1. **Node.js 20 或更高（建议 LTS）**
   - 官网 <https://nodejs.org> 下载安装包，或用 nvm-windows 管理
   - 验证：`node -v` 能输出版本号
2. **（可选）pnpm**：只有需要用 `dsh plugin` 给 profile 增删插件时才会用到。日常使用 Web 界面**不需要**。

## 二、安装

```sh
npm install -g @deepseek-ai/dsh
```

验证安装成功：

```sh
dsh --help
```

## 三、配置 API Key

DeepSeek 官方模型路由默认读取环境变量 `DEEPSEEK_API_KEY`：

**Windows（PowerShell，永久生效）：**

```powershell
setx DEEPSEEK_API_KEY "sk-xxxxxxxx"
```

> 注意：`setx` 之后要**新开一个终端**才生效。

**macOS / Linux：**

```sh
echo 'export DEEPSEEK_API_KEY="sk-xxxxxxxx"' >> ~/.bashrc   # 或 ~/.zshrc
source ~/.bashrc
```

也可以启动后在网页 **设置 → 模型** 里配置（界面操作，优先级以显式配置为准）。

## 四、启动 Web GUI

**第 1 步：先切到你的项目目录** —— dsh 把启动时的工作目录作为默认 workspace 根目录：

```powershell
cd /d D:\HERO\hero2026\rt-thread_-hero\bsp\src\2026_chassis_test
```

**第 2 步：启动：**

```sh
dsh web
```

- 首次运行会自动从内置模板初始化，然后自动打开默认浏览器
- 默认地址：**http://127.0.0.1:3080**
- 保持这个终端开着，关掉终端服务就停了

常用启动参数（加在 `dsh web` 后面）：

| 参数 | 作用 |
|------|------|
| `--port 8080` | 换端口（默认 3080 被占用时用） |
| `--no-open` | 不自动开浏览器（SSH 远程时有用） |
| `--trusted-host` | 额外信任的主机名（极少用） |

> 安全限制：`--host 0.0.0.0` 被刻意禁用（防止把远程代码执行暴露到局域网），服务只绑本机回环地址。其他机器想访问请走 SSH 端口转发。

## 五、验证能用

1. 浏览器打开 GUI，新建会话发一句"你好"，能正常回复说明 API Key 和网络都没问题
2. 让主 agent 干活时如果派生了子 agent，会话标题旁会出现 **`/ N`** 计数——点开可以看到每个子 agent 的状态、耗时、token 用量，还能点进去直接跟它对话或按 Stop（本团队界面已内置此功能）

## 六、其他用法速查

```sh
# 无 GUI 的单次任务：跑完打印最终答案就退出
dsh --profile headless "帮我检查 applications 目录下有没有未使用的头文件包含"

# 管理 profile 插件（需要 pnpm）
dsh plugin --profile web add <package>

# 升级到最新版
npm install -g @deepseek-ai/dsh@latest
```

## 七、官方资源与延伸阅读

**官方（优先看这些）：**

| 资源 | 链接 |
|------|------|
| GitHub 仓库（源码 + Issue 反馈） | <https://github.com/deepseek-ai/deepseek-harness> |
| 官方 README（英文） | <https://github.com/deepseek-ai/deepseek-harness/blob/master/README.md> |
| 官方 README（中文） | <https://github.com/deepseek-ai/deepseek-harness/blob/master/README.zh.md> |
| npm 包页面（版本与更新日志） | <https://www.npmjs.com/package/@deepseek-ai/dsh> |
| DeepSeek 开放平台（申请 API Key） | <https://platform.deepseek.com> |

> 说明：官方文档目前就是 GitHub 仓库的 README（多语言），没有独立文档站；仓库内每个子包也各自带 README 讲解细节。

**第三方教程（非官方，仅供参考）：**

- [DataCamp 中文教程：DeepSeek Harness 设置指南](https://www.datacamp.com/zh/tutorial/deepseek-harness)
- [DeepSeek Harness 安装与运行入门（Kanaries）](https://docs.kanaries.net/topics/AICoding/deepseek-harness-how-to-use)

## 常见问题

| 现象 | 处理 |
|------|------|
| 端口被占用 | 启动时加 `--port` 换一个端口 |
| Key 配了但不生效 | `setx` 后没开新终端；或终端里 `$env:DEEPSEEK_API_KEY` 为空 |
| 想在别的电脑上用 | 在那台电脑上按本流程装一遍；GUI 不支持暴露到局域网 |
| 网页打不开 | 确认 `dsh web` 进程还在跑；看终端里打印的 `dsh web: <URL>` 实际地址 |
