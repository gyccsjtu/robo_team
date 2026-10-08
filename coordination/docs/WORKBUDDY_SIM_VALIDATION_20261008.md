# WorkBuddy六机验证：现成入口与交付任务

用户已授权WorkBuddy自行验证。本单覆盖旧批量任务中的“不得起仿真”，只授权下面一轮，不能并行第二套或原样连续重跑。Codex本轮已因用户转交提前收尾，非基础服务自行崩溃：目录/root/robocup_runs/codex_v141_white_observer_20261008/round_1_seed_8159，轨迹169.344秒、实际裁判0/6，最高真实3.388m，六机已移动；不是600秒结果。最新核查自有仿真与观察器进程为空，11375/11376/19731关闭；启动前仍重新核对。不要全局pkill、WSL shutdown或清理他人的栈。

## 代码与要验证的事

使用codex/radar-coordination-20261003最新提交，至少包含22137ae；先git status与log，保留自己的修改，只能干净时pull --ff-only，不强制reset。生产控制为v1.41蓝色导航运动身份候选修复，尚无成功效果；新的观察器修复是诊断代码，不代表白色识别已经修好。此次验证蓝色导航候选→任务→接近→三原图官方上报是否连通、白色实际覆盖及近处原图。仍以真实裁判6/6与无建筑接触为项目目标。

观察器必须使用Codex新修版：ROS时钟与图像头统一、geometry_msgs/PoseStamped、只在选中图像时解码、无/clock时仍轮询退出。白色真值使用/gazebo/get_model_state；不要用/model_states静态演员位置。旧48张初始补图投影身份无效，文件原样保留。服务拉取相机位姿晚于原图，pose_alignment明确近似，image_pose_delay_s才是原图到取姿态的延迟；pose_pull_delay_s仅RPC耗时。补图只能供目视，不能当作精确同图定位证明。

## 启动（不需要虚拟机、不重新配置WSL）

Windows PowerShell先进入WSL交互shell，后面的命令在WSL Bash内执行，避免Windows/Git Bash吞变量：

```powershell
wsl.exe -d Ubuntu-20.04
```

WSL终端A，runner前台运行。只复用现有入口，不写新run脚本：

```bash
cd /mnt/d/a/.robocup/robo_team
git branch --show-current
git log -3 --oneline
ss -ltn | grep -E '11375|11376|19731'
export WB_VALIDATION_ROOT="/root/robocup_runs/wb_v141_validation_$(date -u +%Y%m%dT%H%M%SZ)"
printf '%s\n' "$WB_VALIDATION_ROOT"
CITY_PHYSICS_RATE=40 python3 coordination/scripts/validate_fast_city.py \
  --single-seed 8159 --root "$WB_VALIDATION_ROOT"
```

端口检查若有输出，先核对所属任务，不启动。保存打印出来的绝对root；新root不能复用旧budget。runner自动设置600秒、共享GPU、GPU服务python、CPU轻客户端、19731、GPS/插件/Gazebo环境等；无需复制旧单机六条export或切换环境。保持40Hz，与此前速度一致。不要setsid/nohup脱离runner；前台错误必须能看到。Gazebo GUI不是必要条件，先完成主链验证。

## 挂只读观察器

WSL终端B，把下面root替换成终端A实际打印的同一路径。等flight/wiring.json出现且ROS11375监听后再执行。观察器可以后台，runner必须前台。每个PID和日志写进本轮，检查日志确认没有导入/话题错误。

```bash
cd /mnt/d/a/.robocup/robo_team
export WB_VALIDATION_ROOT=/root/robocup_runs/替换成实际root
export WB_ROUND="$WB_VALIDATION_ROOT/round_1_seed_8159"
test -f "$WB_ROUND/flight/wiring.json"
source /root/robo_team_build/codex_authority_v2/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11375
export ROS_IP=127.0.0.1
mkdir -p "$WB_ROUND/observers"
for tag in green blue brown white red; do
  python3 coordination/scripts/observe_visual_accuracy.py \
    --run "$WB_ROUND/observers" --result-file "$WB_ROUND/flight/result.json" \
    --tag "$tag" --wall-seconds 5400 > "$WB_ROUND/observers/$tag.log" 2>&1 &
  printf '%s %s\n' "$!" "$tag" >> "$WB_ROUND/observers/owned_pids.txt"
done
python3 coordination/scripts/capture_visual_frames.py \
  --output "$WB_ROUND/observers/frames" --wall-seconds 5400 --loss-color white \
  > "$WB_ROUND/observers/frames.log" 2>&1 &
printf '%s frames\n' "$!" >> "$WB_ROUND/observers/owned_pids.txt"
python3 coordination/scripts/white_evidence_observer.py --enable \
  --wiring "$WB_ROUND/flight/wiring.json" --out-dir "$WB_ROUND/observers/white_dev" \
  --result-file "$WB_VALIDATION_ROOT/budget.json" --total-cap 120 \
  --min-expected-px 16 --max-range-m 22 \
  > "$WB_ROUND/observers/white_dev.log" 2>&1 &
printf '%s white_dev\n' "$!" >> "$WB_ROUND/observers/owned_pids.txt"
```

16像素/22米只是优先补近图的诊断采样条件，不是生产门限或赛事规定，也不保证一定采到；没有合格近图就是缺证。补图仅保存文件，人物真值禁止进入感知/manager/控制。记录所有完整cmdline、PID、脚本SHA和启动时间；启动失败先修自己的接入，不重复起仿真。

## 监控与收尾

budget.attempts[-1].status结合runner/launcher实际存活判断，不能只看RUNNING；attempt的output指向本轮。检查新轨迹增长、judge.log与city_events.jsonl的left_actors、六路frame_probe新原图、shared服务与客户端实际ready、观察器日志与索引。不要将connectivity_armed=false起栈快照解释成没起飞，不将“全部到达”当成功。

上限600仿真秒，6/6提前收尾；建筑接触harness会终止，基础服务死亡则及时收尾。若已证实的新阻塞继续等无益，记录具体证据后终端A Ctrl+C。注明实际停止原因/实际秒数，不改原budget/result，把解释放独立sidecar。不因为0/6暂时无进展随意宣称控制失败。

结束后核对owned_pids的/proc/PID/cmdline含本轮绝对路径且脚本正确，先SIGTERM；仍活才复核后SIGKILL，只处理本轮进程。核查三个端口及自有runner/launcher、观察器退出。不能按旧PID或名字全局清场。

## 一次性交付给Codex

提交WORKBUDDY_SIM_RESULT_20261008.md：执行HEAD、run_id、root、原始预算/结果、真实裁判删除时刻、轨迹时间跨度、六机位移和最高真实高度、原生接触覆盖/建筑/人物/机间分别统计、实际运行源码SHA。观察器输出与故障独立记录；空文件不是PASS。

逐段列蓝色候选/任务/接近/上报的时间及断点；白色原图数量、原图时刻、实际人物真值来源、近似相机取姿态时差、人工可见性和模型重放。只用有证据的近图判断识别能力。精选原图及小日志入仓，大日志留WSL绝对路径/SHA。写已验证/推测/缺证、剩余问题和最小修法，先自审再交Codex；不能自行放宽生产识别/净空门或改裁判。只一轮，不自动第二轮。
