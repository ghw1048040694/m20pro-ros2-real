# 多人协作开发约定

GitLab 是公司开发源，`main` 是已验证基线。跨楼层统一导航在
`feature/unified-navigation-dddmr` 作为集成分支开发；GitHub 和 Gitee 不接收
开发分支，只发布经过验证的 `main`。

## 分支规则

每个人从集成分支创建自己的短分支，不直接向集成分支或 `main` 推送：

```bash
git fetch gitlab
git switch feature/unified-navigation-dddmr
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

提交 Merge Request 到 `feature/unified-navigation-dddmr`，标题说明领域和
行为变化。合并前必须：

1. 通过相关纯契约测试、Python/JavaScript 语法检查和 `git diff --check`；
2. 说明是否改变 ROS topic、QoS、参数、启动文件或运动控制；
3. 说明录包验证方式和失败时的停止行为；
4. 不在没有现场验收的情况下修改 `main` 或部署 104/106。

集成分支只由负责人合并经过审查的 MR。冲突优先回到统一计划和唯一感知链
边界解决，不通过重复条件判断临时绕过。

