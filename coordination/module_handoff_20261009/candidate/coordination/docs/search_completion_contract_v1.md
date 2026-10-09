# 搜索完成证据 v1

纯Python `completion_state(unfinished_cells,official_seen,official_left,unconfirmed_targets)` 冻结为四种结果：SEARCH_OR_WAIT_FOR_ROUTE、OFFICIAL_EVIDENCE_MISSING、TARGETS_REMAIN、OFFICIAL_COMPLETION_CONFIRMED。没有候选、几何筛查失败、租约保留、tracker为空都不是完成证明。已确认收到过官方非空清单，之后官方目标为空且没有未确认目标时，允许完成；不要求继续搜索与目标消除无关的空格。

实际manager在拍卖不完整时，尚有搜索格或执行中格则继续搜索/等待；缺官方结果且覆盖耗尽则重新巡逻，不广播MISSION_FINISHED。官方剩余目标清单只接受JSON整数列表，ID为0..5、不能重复；空字符串、任意文字、负数/越界ID、布尔值、字符串数字和非列表均拒绝，保留既有证据。首条空清单仍不能证明本轮完成，沿用现有 `_left_seen` 纪律。

任务授权v2保持不变，几何筛查和完成证据都不释放未知占用。该纯判定不是正式计分器，也不证明相机识别的15s规则；官方话题的来源与运行隔离仍由接入负责。
