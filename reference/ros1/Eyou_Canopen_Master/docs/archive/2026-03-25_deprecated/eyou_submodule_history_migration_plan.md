# Eyou_Canopen_Master 父子仓库关系与历史迁移计划（Commit 级）

更新时间：2026-03-22

## 1. 当前基线（已核实）

- 父仓库：`/home/dianhua/Robot24_catkin_ws/src`
  - 分支：`main`
  - HEAD：`8055583`
- 子仓库：`/home/dianhua/Robot24_catkin_ws/src/Eyou_Canopen_Master`
  - 分支：`main`
  - HEAD：`c42be7c`
  - 远端：`git@github.com:dianhuaeven/Eyou_Canopen_master.git`
- 父子关系：已转换为 `submodule`
  - `.gitmodules`:
    - `path = Eyou_Canopen_Master`
    - `url = git@github.com:dianhuaeven/Eyou_Canopen_master.git`
  - 当前 gitlink 指向：`c42be7c37498f211be6d0454d0f6b1b1c2354e23`

---

## 2. 父子关系建立提交计划（父仓库）

| 顺序 | 仓库 | 计划提交信息 | 改动内容 | 目的 |
|---|---|---|---|---|
| P-01 | 父仓库 `src` | `chore(submodule): convert Eyou_Canopen_Master to git submodule` | 新增 `.gitmodules`；将 `Eyou_Canopen_Master` 从普通目录替换为 gitlink（指向 `c42be7c`） | 建立标准父子关系 |
| P-02 | 父仓库 `src` | `docs(migration): add commit-level history migration plan` | 新增本文件 `docs/eyou_submodule_history_migration_plan.md` | 固化迁移方案与执行规范 |

> 备注：P-01、P-02 可合并为一个提交；若需要审阅清晰，建议分成两个提交。

---

## 3. 历史迁移计划表（源提交 -> 子仓库）

说明：
- 源提交来自父仓库路径历史：`git log --reverse --oneline -- Eyou_Canopen_Master`
- 检查口径：
  - `forward_check`: `git apply --check -p2`（能否直接应用到子仓库）
  - `reverse_check`: `git apply --reverse --check -p2`（当前子仓库是否已包含该补丁效果）

| 序号 | 源提交（父仓库） | 标题 | forward_check | reverse_check | 迁移动作（子仓库） | 父仓库跟随提交（submodule 指针） |
|---|---|---|---|---|---|---|
| M-00 | `7605dbd` | feat: add full Eyou_Canopen_Master package (code, urdf, docs, dcf) | conflict | no | **跳过**（子仓库已有独立历史，不重复导入整包） | 无 |
| M-01 | `c1ad549` | docs: archive outdated reports and refresh canopen fix baseline | conflict | no | 手工迁移为 1 个 docs 提交（建议消息：`migrate(src:c1ad549): docs archive baseline`） | `chore(submodule): bump Eyou_Canopen_Master to <child_sha> (src:c1ad549)` |
| M-02 | `e9fda64` | fix(canopen): publish controlword in rpdo cycle | conflict | already_applied_exact | **标记已覆盖**（当前子仓库树已包含该补丁效果） | 可选：不单独 bump |
| M-03 | `f23c328` | fix(canopen): align shutdown flow with disable request | conflict | already_applied_exact | **标记已覆盖**（无需重复迁移） | 可选：不单独 bump |
| M-04 | `12b457b` | feat(canopen): add boot identity mismatch diagnostics | conflict | no | 手工迁移为 1 个功能提交（建议消息：`migrate(src:12b457b): boot identity diagnostics`） | `chore(submodule): bump Eyou_Canopen_Master to <child_sha> (src:12b457b)` |
| M-05 | `2475555` | test(canopen): add startup and operational regression | conflict | no | 手工迁移为 1 个测试提交（建议消息：`migrate(src:2475555): startup regression tests`） | `chore(submodule): bump Eyou_Canopen_Master to <child_sha> (src:2475555)` |
| M-06 | `84de487` | docs(canopen): finalize validation checklist for field | conflict | already_applied_exact | **标记已覆盖**（无需重复迁移） | 可选：不单独 bump |
| M-07 | `047886d` | fix(canopen): cover boot identity diagnostics and harden state machine | conflict | already_applied_exact | **标记已覆盖**（无需重复迁移） | 可选：不单独 bump |
| M-08 | `240a408` | fix(canopen): restore ROS Ctrl+C shutdown behavior | conflict | no | 手工迁移为 1 个 ROS 修复提交（建议消息：`migrate(src:240a408): restore ROS Ctrl+C shutdown`） | `chore(submodule): bump Eyou_Canopen_Master to <child_sha> (src:240a408)` |
| M-09 | `8055583` | refactor: update Eyou canopen and remove yiyou_canopen package | conflict | already_applied_exact | **标记已覆盖**（当前子仓库树已覆盖该补丁效果） | 可选：不单独 bump |

---

## 4. 精确执行顺序（建议）

1. 先完成父仓库关系提交：P-01（必要）+ P-02（建议）。
2. 在子仓库新建迁移分支：`migration/from-src-history`。
3. 按表执行仅需迁移项：`M-01 -> M-04 -> M-05 -> M-08`（每项一个提交）。
4. 每完成 1 个子仓库提交并 push 后，在父仓库创建 1 个对应指针提交（message 中带 `src:<源commit>`）。
5. 全部完成后，在父仓库打 1 个汇总说明提交（可选）。

---

## 5. 每个迁移项的标准操作模板（可复用）

### 5.1 子仓库迁移提交模板

```bash
# 1) 查看源提交差异（父仓库）
git -C /home/dianhua/Robot24_catkin_ws/src show <SRC_COMMIT> -- Eyou_Canopen_Master

# 2) 在子仓库手工落地等价修改（按该提交边界）
cd /home/dianhua/Robot24_catkin_ws/src/Eyou_Canopen_Master
# 编辑文件...

# 3) 单提交提交并推送
git add <files>
git commit -m "migrate(src:<SRC_COMMIT>): <summary>"
git push origin main
```

### 5.2 父仓库 submodule 指针提交模板

```bash
cd /home/dianhua/Robot24_catkin_ws/src

git add Eyou_Canopen_Master
git commit -m "chore(submodule): bump Eyou_Canopen_Master to <CHILD_SHA> (src:<SRC_COMMIT>)"
```

---

## 6. 验收标准

- 父仓库中 `Eyou_Canopen_Master` 为 gitlink（非普通目录树）。
- `.gitmodules` 正确记录子仓库 URL。
- 迁移项中每个“需迁移”的源提交均在子仓库存在 1 个对应提交（message 带 `src:<commit>`）。
- 父仓库有与之对应的 submodule bump 提交，保证审计链可回溯。
