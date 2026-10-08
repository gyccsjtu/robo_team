# WorkBuddy任务单：白色人物识别诊断

用户授权Codex统筹分工。任务仓库D:\a\.robocup\robo_team，分支codex/radar-coordination-20261003。先读根AGENTS.md、legacy_chain_postrun_v141_20261008.md与predata_comparison_20261008.md。此任务单已准备，尚未证明WorkBuddy已收到/执行。

## 目标

用现有原图查清白色真人从颜色框到人体/衣服证明的拒绝位置，给出最小可执行修复候选。不要把单帧失败推成模型全面失明，不通过统一降阈值放行静态装饰或车辆。

## 输入

coordination/docs/validation/legacy_chain_v140_20261008，以及之前的reporting_alignment_physical_20261008、previous_rate_40_20261008证据；完整本轮在WSL /root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159。缺少目标原图要明确标注，不能自行生成证据。

## 允许的工作

离线读取感知/共享服务，重放已保存原图，记录每个白色框的颜色conf、人体conf、IoU/支持框、衣服检查、实际像素尺寸、拒绝理由、模型SHA。把白色真人与静态干扰样本分开。可修改自己的诊断工具和任务报告，不修改perception_real.py、visual_observation.py、桥、manager、agent、路线/STOP核心；这些由Codex维护。实际模型重放与数值构造区分。

## 运行协调

本次先离线，不启动六机/单机仿真，不停或清理任何仿真进程。Codex本轮已结束且自有端口已关闭，后续是否运行由Codex协调，避免重复起栈。不要改裁判/人物/传感器/WSL内存；诊断真值仅离线核验，不进入控制。

## 交付

写coordination/docs/workbuddy_white_review_20261008.md，附原图索引、准确可重现命令、已验证/推测/缺证三类结论、最多两个候选修法与受影响文件。不推主干，不覆盖已有修改。先交给Codex复核，再整合生产代码。
