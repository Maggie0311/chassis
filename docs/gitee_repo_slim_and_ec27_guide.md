# heroec-2026 仓库瘦身实录 & 27 赛季建仓指引

> 日期：2026-09-12 ｜ 操作人：电控组（AI 辅助执行）
> 背景：Gitee 免费版单仓库上限 **1024MB**，`hero_robomaster/heroec-2026`（电控26）服务端达 **1090.5MB**，推送被拒。

## 一、体检结论

| 仓库 | 服务端体积 | 结论 |
|---|---|---|
| `heroec-2026`（电控26） | 1090MB，超限 | **元凶**，已瘦身 |
| `rt-thread_-hero`（RT-Thread 基座/索引仓） | ~22MB | 健康，无需处理（本地 1.6GB 是 `.git/modules` 子模块缓存，不占服务端） |
| `2026_chassis_test`（底盘） | ~31MB | 健康 |

heroec-2026 历史中的体积构成（打包后）：Keil 编译产物 `.crf/.o` 约 425MB、备份压缩包 `.zip/.rar` 约 341MB、IDE 缓存/JLinkLog 约 95MB、`2025/` 整树备份 74MB，真实源码仅约 127MB。

## 二、已执行的操作（可复现）

```powershell
$r = "D:\HERO\hero2026\rt-thread_-hero\bsp\src\old\heroec-2026"

# 1. 全量备份（回滚保险，含所有分支完整旧历史，921MB）
git -C $r bundle create D:\HERO\heroec-2026-full-backup-20260912.bundle --all

# 2. 同步服务端并确认未分叉
git -C $r fetch origin --prune

# 3. 历史重写：删除4类垃圾（编译产物/压缩包/IDE缓存/2025备份目录）
git filter-repo --force --invert-paths `
  --path-glob '*.crf' --path-glob '*.o' --path-glob '*.axf' `
  --path-glob '*.zip' --path-glob '*.rar' `
  --path-glob '*.VC.db' --path-glob '*.VC.db-shm' --path-glob '*.VC.db-wal' `
  --path-glob '*.vsidx' --path-glob '*.ipch' --path-glob '*JLinkLog*' `
  --path 2025

# 4. 重连远端并全量强推（filter-repo 会移除 origin，需重新添加）
git -C $r remote add origin git@gitee.com:hero_robomaster/heroec-2026.git
git -C $r push origin --force --all
git -C $r push origin --force --tags
```

**结果：922MB → 112MB（-88%），47 个分支、1425 个提交全部保留且可读，垃圾文件零残留。**

## 三、待办清单

### 管理员（1 分钟）
- [ ] 打开 <https://gitee.com/hero_robomaster/heroec-2026/settings#git-gc>，点击 **仓库 GC**，触发服务端回收旧对象（否则页面显示的 1090MB 不会掉下来，且推送次数仍被限制为 50 次）
- [ ] GC 完成后在 仓库设置 → 基本信息 确认体积降到 ~112MB

### 全队（重要！）
- [ ] **删除旧的 heroec-2026 克隆，重新 `git clone`**。旧克隆里还是垃圾历史，谁用旧克隆强推一次，900MB 垃圾就全回来了
- [ ] `rt-thread_-hero` 里 `bsp/src/Code2026_HERO` 子模块需重新 `git submodule update --init`，并在基座仓提交一次新指针
- [ ] 需要旧文件（某次比赛的编译产物/压缩包）时，从备份 bundle 恢复：
  `git clone D:\HERO\heroec-2026-full-backup-20260912.bundle old-heroec`

## 四、27 赛季建仓步骤（跟以往赛季同样的模式）

1. 组织下新建私有空仓 `heroec-2027`（不要导入任何旧历史，从零开始）。
2. 在 `rt-thread_-hero` 的 `.gitmodules` 增加条目并初始化：
   ```ini
   [submodule "bsp/src/Code2027_HERO"]
       path = bsp/src/Code2027_HERO
       url = git@gitee.com:hero_robomaster/heroec-2027.git
       branch = master
   ```
3. 各模块代码按目录放入：`chassis/ gimbal/ shooter/ board/ docs/`，一人一分支。
4. 现有底盘私有仓 `Jichen_Li_good/2026_chassis_test` 迁入（保留历史）：
   ```powershell
   cd <2026_chassis_test 目录>
   git filter-repo --to-subdirectory-filter chassis   # 全部历史移入 chassis/ 子目录
   git remote add ec27 git@gitee.com:hero_robomaster/heroec-2027.git
   git push ec27 master
   ```
   迁完可删除/转公开个人私有仓，腾出一个私有仓名额。

## 五、防复发规约（写进团队 README）

1. **编译产物绝不入库**——`.gitignore` 必须包含（模板见下）。
2. **备份压缩包绝不入库**——代码历史本身就是备份，打包备份放网盘/Release。
3. PDF 手册、原理图、PCB、串口屏资源等二进制尽量放网盘或 Release 附件；必须入库时先压缩到最小。
4. 每月看一眼仓库设置页的体积，**超过 500MB 就该警惕**。
5. 赛季结束仓归档（Archive），新赛季开新仓。

`.gitignore` 模板（Keil + VS + RT-Thread）：

```gitignore
# ---- 编译产物 ----
*.o
*.d
*.crf
*.axf
*.lst
*.dep
*.iex
*.htm
build/
Output/
Listings/
Objects/
# ---- IDE/调试 ----
.vs/
*.uvguix.*
JLinkLog.txt
*.VC.db*
*.ipch
# ---- 备份压缩包 ----
*.zip
*.rar
*.7z
```

## 六、回滚方法（万一）

```powershell
git clone D:\HERO\heroec-2026-full-backup-20260912.bundle heroec-2026-restore
cd heroec-2026-restore
git remote add origin git@gitee.com:hero_robomaster/heroec-2026.git
git push origin --force --all
```
