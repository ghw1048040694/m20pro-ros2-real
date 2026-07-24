# 多人协作开发约定

GitLab 是公司开发源，`main` 是已验证基线。跨楼层统一导航在
`feature/unified-navigation-v2` 作为集成分支开发，并同步到 GitHub 备份；
Gitee 只发布经过验证的 `main`，供机器狗部署使用。

## 开发优先级

先让唯一、最小的运行链形成可测试闭环，再依据真实录包和现场失效逐项加固。
首轮只保留直接影响闭环与设备控制的必要判定：用户停止、关键通信超时、
阶段超时，以及切图或重定位失败。未经实测证明必要，不新增哈希认证、多层
readiness、重复仲裁、平行动作链或复杂回退状态机。

安全加固不能靠预设大量条件代替真实验收。每一项新增判定都必须能说明它所
对应的已观测风险、触发后的确定行为和回归测试；不能通过在故障点外层继续
叠加条件来掩盖根因。

## 分支规则

每个人从集成分支创建自己的短分支，不直接向集成分支或 `main` 推送：

```bash
git fetch gitlab
git switch feature/unified-navigation-v2
git pull --ff-only
git switch -c feature/unified-nav-<area>
```

建议的工作边界：

- `feature/unified-nav-contract`：统一任务/路线数据模型和离线测试；
- `feature/unified-nav-terrain`：106 本地 terrain guard 与点云录包分析；
- `feature/unified-nav-executor`：连接边执行器和安全停止状态机；
- `feature/unified-nav-frontend`：统一计划的前端展示和任务编排。

每个分支只负责一个边界，不直接复制 `floor_manager` 或新增第二套 Nav2。

## 合并请求要求

提交 Merge Request 到 `feature/unified-navigation-v2`，标题说明领域和
行为变化。合并前必须：

1. 通过相关纯契约测试、Python/JavaScript 语法检查和 `git diff --check`；
2. 说明是否改变 ROS topic、QoS、参数、启动文件或运动控制；
3. 说明录包验证方式和失败时的停止行为；
4. 不在没有现场验收的情况下修改 `main` 或部署 104/106。

集成分支只由负责人合并经过审查的 MR。冲突优先回到统一计划和唯一感知链
边界解决，不通过重复条件判断临时绕过。
