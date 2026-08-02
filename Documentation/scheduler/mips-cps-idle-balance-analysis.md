# MIPS 1004K CPS 到 CFS `idle_balance()` 的源码链路审计

本文针对本树（Linux 3.18.21）进行静态源码审计。结论先行：源码能够证明
`CM/GCR -> cpu_data/cpu_sibling_map -> sched_domain -> sd_llc_id ->
cpus_share_cache()` 以及 `pick_next_task_fair() -> idle_balance()` 两段链路；但**不能**
证明题设中的完整因果链，更不能仅凭拓扑证明 livelock。尤其是 CPS 路径没有用
CP0 EBase 的 CPUNum（有些文档称 GlobalNumber）建立 Linux 调度拓扑。因此，若现场
认为 GlobalNumber 错误导致调度域错误，必须补充寄存器、启动日志和调度域转储。

## 1. 硬件发现与 GCR 数据流

### 1.1 CM 探测

`arch/mips/kernel/mips-cm.c::__mips_cm_phys_base()` 先检查
`Config3.CMGCR`，再用 CP0 `CMGCRBase` 求 GCR 物理地址：

```c
if (!(config3 & MIPS_CONF3_CMGCR))
	return 0;
cmgcr = read_c0_cmgcrbase();
return (cmgcr & MIPS_CMGCRF_BASE) << (36 - 32);
```

`mips_cm_probe()` 将其映射为 `mips_cm_base`，读取 `GCR_BASE` 校验映射，并把
CM 默认目标设为内存。`arch/mips/include/asm/mips-cm.h` 定义四个块：GCB
(`0x0000`)、Core Local (`0x2000`)、Core Other (`0x4000`) 和 GDB
(`0x6000`)；`BUILD_CM_*` 生成 `read_gcr_*()`/`write_gcr_*()` MMIO 访问器。

调用关系是：

```text
platform early init
  -> mips_cm_probe()
     -> mips_cm_phys_base()
        -> __mips_cm_phys_base()
           -> read_c0_config3(), read_c0_cmgcrbase()
     -> ioremap_nocache(..., MIPS_CM_GCR_SIZE)
     -> read_gcr_base()/write_gcr_base()
  -> register_cps_smp_ops()
     -> register_smp_ops(&cps_smp_ops)
```

`arch/mips/kernel/smp-cps.c::register_cps_smp_ops()` 明确要求 CM 已存在，并从
`GCR_GIC_STATUS` 确认 GIC 后才注册 CPS SMP 操作。

### 1.2 GCR 枚举 core/VPE

`arch/mips/kernel/smp-cps.c::cps_smp_setup()` 是关键拓扑生产者：

```c
ncores = mips_cm_numcores();
for (c = nvpes = 0; c < ncores; c++) {
	core_vpes = core_vpe_count(c);
	if (!c)
		smp_num_siblings = core_vpes;
	for (v = 0; v < min_t(int, core_vpes, NR_CPUS - nvpes); v++) {
		cpu_data[nvpes + v].core = c;
		cpu_data[nvpes + v].vpe_id = v;
	}
	nvpes += core_vpes;
}
```

其中：

* `mips_cm_numcores()` 读取 `GCR_CONFIG.PCORES`，返回该字段加一；
* `core_vpe_count()` 向 `GCR_CL_OTHER.CORENUM` 写目标 core，再读
  `GCR_CO_CONFIG.PVPE`，返回该字段加一；
* 结果写入全局 `struct cpuinfo_mips cpu_data[]` 的 `core` 和（启用
  `CONFIG_MIPS_MT_SMP` 时）`vpe_id`；core 0 的 VPE 数还写入
  `smp_num_siblings`；
* 随后 VPE 被线性编号为 Linux CPU，possible/present mask 与
  `__cpu_number_map[]`、`__cpu_logical_map[]` 均按 `v = 0..nvpes-1`
  设置为恒等映射。

因此本树的实际数据流是：

```text
GCR_CONFIG.PCORES --------------------------> ncores
GCR_CL_OTHER.CORENUM -> GCR_CO_CONFIG.PVPE -> core_vpes
                                               |
                     +-------------------------+-------------------+
                     v                         v                   v
          cpu_data[logical].core   cpu_data[].vpe_id   smp_num_siblings
```

`cps_smp_setup()` 还通过 `change_c0_config(CONF_CM_CMASK, 0x5)` 选择 coherent
write-back CCA，并通过 `write_gcr_cl_coherence(0xff)` 使 core 0 加入全部一致性域。
`cps_prepare_cpus()` 会再次检查 CCA；若不适合多核，会把非零 core 的 CPU 从
present mask 删除。这些代码证明 CM 是 CPS 可用性与硬件一致性的基础，但不代表
调度器会直接读取一致性寄存器。

## 2. CP0 EBase.CPUNum（GlobalNumber）不是 CPS 调度拓扑输入

本树没有名为 `GLOBALNUMBER` 的宏或 C 访问器。相关硬件编号在源码中表现为
CP0 EBase（CP0 register 15 select 1）的低位 CPUNum：

```c
static inline unsigned int get_ebase_cpunum(void)
{
	return read_c0_ebase() & 0x3ff;
}
```

但 `arch/mips/kernel/cpu-probe.c::cpu_probe_mips()` 对 CPS 明确排除此路径：

```c
#ifndef CONFIG_MIPS_CPS
if (cpu_has_mips_r2) {
	c->core = get_ebase_cpunum();
	...
}
#endif
```

CPS 确实在 `arch/mips/kernel/cps-vec.S` 中读取 EBase.CPUNum，不过用途仅是从
CPUNum 低位求**当前 core 内的 VPE ID**，以索引 `vpe_boot_config`：

```asm
/* Retrieve the VPE ID from EBase.CPUNum */
mfc0	t9, $15, 1
and	t9, t9, t1
```

同一段汇编中的 core ID 则来自 `GCR_CL_ID`。它没有把 EBase 值写入
`cpu_data[].core`、`cpu_sibling_map[]` 或任何 `sched_domain`。此外，MIPS 的
`raw_smp_processor_id()` 是 `current_thread_info()->cpu`，并非实时读取 EBase。

所以从本仓库能证明的是：

```text
GCR_CL_ID + EBase.CPUNum(low VPE bits) -> 选择 CPS 启动配置
GCR_CONFIG/GCR_CO_CONFIG              -> Linux cpu_data 拓扑
```

不能证明 `GCR -> CP0 GlobalNumber -> Linux cpu topology` 这一串行链。即使硬件由
CM 给各 VPE 分配全局编号，本版本 Linux 的 CPS 调度拓扑仍由 GCR 枚举顺序显式
构造，而不是从该编号反解。

## 3. `cpu_data` 到 Linux topology mask

数据结构定义如下：

* `arch/mips/include/asm/cpu-info.h::struct cpuinfo_mips` 保存 `package`、
  `core` 和 `vpe_id`；静态零初始化意味着此 CPS 代码未另行设置时所有 CPU 的
  package 均为 0；
* `arch/mips/kernel/smp.c` 定义 `cpu_sibling_map[NR_CPUS]`、
  `cpu_core_map[NR_CPUS]` 以及两个 setup mask；
* `arch/mips/include/asm/topology.h` 将 `topology_thread_cpumask(cpu)` 映射到
  `cpu_sibling_map[cpu]`，将 `topology_core_cpumask(cpu)` 映射到
  `cpu_core_map[cpu]`。

启动 CPU 在 `smp_prepare_cpus()`、次级 CPU 在 `start_secondary()` 中依次调用：

```text
set_cpu_sibling_map(cpu)
  -> 比较 cpu_data[cpu].package 与 .core
  -> 同 package 且同 core 的逻辑 CPU 互相加入 cpu_sibling_map[]

set_cpu_core_map(cpu)
  -> 比较 cpu_data[cpu].package
  -> 同 package 的逻辑 CPU 互相加入 cpu_core_map[]
```

`set_cpu_sibling_map()` 只有在 `smp_num_siblings > 1` 时才合并同 core VPE；否则
每个 CPU 的 sibling mask 只有自身。

## 4. topology mask 到 `sched_domain` 和 LLC ID

`kernel/sched/core.c::default_topology[]` 自底向上定义调度域：

```c
#ifdef CONFIG_SCHED_SMT
{ cpu_smt_mask, cpu_smt_flags, SD_INIT_NAME(SMT) },
#endif
#ifdef CONFIG_SCHED_MC
{ cpu_coregroup_mask, cpu_core_flags, SD_INIT_NAME(MC) },
#endif
{ cpu_cpu_mask, SD_INIT_NAME(DIE) },
```

对 MIPS 而言，`include/linux/topology.h::cpu_smt_mask()` 最终返回上节的
`cpu_sibling_map[cpu]`；`cpu_cpu_mask()` 返回所在 NUMA node 的在线 CPU mask
（普通非 NUMA BSP 即全部在线 CPU）。本 MIPS 头文件没有提供
`cpu_coregroup_mask()`，因此应特别核对产品配置是否启用 `CONFIG_SCHED_MC`；单凭
存在 `cpu_core_map[]` 不能认定调度器使用了 MC 层。

`build_sched_domains()` 对每个 CPU、每个 topology level 调用
`build_sched_domain()`；后者执行：

```c
sd = sd_init(tl, cpu);
cpumask_and(sched_domain_span(sd), cpu_map, tl->mask(cpu));
child->parent = sd;
sd->child = child;
```

`sd_init()` 把 `tl->sd_flags()` 并入 `sd->flags`，同时默认设置
`SD_LOAD_BALANCE | SD_BALANCE_NEWIDLE` 等标志。`cpu_smt_flags()` 返回
`SD_SHARE_CPUCAPACITY | SD_SHARE_PKG_RESOURCES`，所以 SMT sibling domain 同时被
视为共享执行容量和 package resource。

建域结束后 `cpu_attach_domain()` 将最低层 `sd` 发布到 `rq->sd`，再调用
`update_top_cache_domain(cpu)`：

```c
sd = highest_flag_domain(cpu, SD_SHARE_PKG_RESOURCES);
if (sd) {
	id = cpumask_first(sched_domain_span(sd));
	size = cpumask_weight(sched_domain_span(sd));
}
per_cpu(sd_llc_id, cpu) = id;
```

于是 `kernel/sched/core.c::cpus_share_cache()` 只是比较预计算 ID：

```c
return per_cpu(sd_llc_id, this_cpu) == per_cpu(sd_llc_id, that_cpu);
```

完整的软件数据结构流转为：

```text
GCR fields
 -> ncores/core_vpes
 -> cpu_data[].{core,vpe_id}, smp_num_siblings
 -> cpu_sibling_map[]
 -> topology_thread_cpumask()/cpu_smt_mask()
 -> sched_domain_topology_level.mask
 -> sched_domain.span + SD_SHARE_PKG_RESOURCES
 -> rq->sd
 -> per_cpu(sd_llc_id)
 -> cpus_share_cache()
```

注意：`cpus_share_cache()` 不在 `idle_balance()` 的调用路径上。此版本中它用于
`select_idle_sibling()` 和 `ttwu_queue()`；后者决定唤醒是远程排队还是直接锁目标
runqueue。它可能改变竞态时序，但不能据此声称它直接调用或控制
`idle_balance()`。

## 5. CFS `pick_next_task_fair()` 与 `idle_balance()`

直接调用链位于 `kernel/sched/fair.c`：

```text
schedule()/调度类选择
 -> fair_sched_class.pick_next_task
 -> pick_next_task_fair(rq, prev)
    -> 本地 cfs_rq 无 runnable entity
       -> idle_balance(rq)
          -> for_each_domain(this_cpu, sd)
             -> load_balance(..., sd, CPU_NEWLY_IDLE, ...)
    -> new_tasks > 0: goto again
    -> new_tasks < 0: RETRY_TASK
    -> new_tasks == 0: 返回 NULL，选择 idle class
```

`idle_balance()` 先检查 `avg_idle` 与 root-domain `overload`。真正扫描时它释放
`this_rq->lock`，对 `rq->sd` 的每一级且带 `SD_LOAD_BALANCE`、
`SD_BALANCE_NEWIDLE` 的 domain 调用 `load_balance()`，随后重新获得锁。其返回值
还会因并发入队被改写：

```c
if (this_rq->cfs.h_nr_running && !pulled_task)
	pulled_task = 1;
...
if (this_rq->nr_running != this_rq->cfs.h_nr_running)
	pulled_task = -1;
```

`pick_next_task_fair()` 对正返回值执行 `goto again`。因此源码允许如下反复竞态：
在 `idle_balance()` 放锁窗口任务进入本 rq，使其返回正值；重新选取前任务又被
迁移、节流或出队，使 CFS 再次为空；代码再进入 `idle_balance()`。这解释了为何
采样栈可能持续落在 `pick_next_task_fair()/idle_balance()`，但它只是 livelock 的
必要代码形态，不是本 BSP 上已发生该竞态的证明。

## 6. 对题设因果链的审计判定

| 声称的边 | 本树判定 | 源码依据 |
|---|---|---|
| 1004K CPS hardware -> CM | 条件成立 | CPS SMP 注册要求 `mips_cm_present()` |
| CM -> GCR registers | 成立 | `mips_cm_probe()` 映射 GCR；访问器为 MMIO |
| GCR -> CP0 GlobalNumber | 本树不可证明 | 无 GLOBALNUMBER 符号；EBase 仅在启动汇编求 VPE ID |
| CP0 GlobalNumber -> Linux topology | CPS 路径不成立 | `cpu_probe_mips()` 在 `CONFIG_MIPS_CPS` 下排除 EBase core 推导 |
| GCR -> Linux topology | 成立 | `cps_smp_setup()` 写 `cpu_data[]` 与 sibling 数 |
| Linux topology -> sched_domain | 成立（受 Kconfig 约束） | SMT mask -> `build_sched_domain()` -> `rq->sd` |
| sched_domain -> cpus_share_cache | 成立 | `update_top_cache_domain()` 生成 `sd_llc_id` |
| cpus_share_cache -> idle_balance | 无直接调用边 | 前者只见于 idle sibling 选择和 TTWU 排队 |
| idle_balance -> livelock | 可能机制，不是静态证明 | 放锁与 `goto again` 允许竞态重复 |

因此应把目标链修正为两条相交于运行时竞态、而非静态直接调用的链：

```text
CM/GCR -> CPS enumeration -> topology masks -> sched_domain -> sd_llc_id
                                                     |
                                                     +-> load-balance scope

cpus_share_cache -> wakeup placement/TTWU locking timing
                                      |
                                      +--(runtime race)--+
                                                        v
pick_next_task_fair <-> idle_balance(lock drop) -> repeated retry/livelock
```

## 7. 现场验证与证伪清单

静态源码之外，至少采集以下证据才能把“可能机制”提升为 EN7561DU 的根因：

1. 保存 `VPE topology {...} total ...` 启动日志，并核对预期 core/VPE 数；该日志由
   `cps_smp_setup()` 打印。
2. 在每个逻辑 CPU 上记录 EBase、`GCR_CL_ID`、`cpu_data[cpu].core/vpe_id`，确认
   全局编号唯一且 GCR 枚举结果一致；不要把 EBase 数值直接当 Linux CPU ID。
3. 开启 `CONFIG_SCHED_DEBUG`，采集 `/proc/sched_debug` 与 sched-domain sysctl，核对
   SMT span、DIE span、`SD_SHARE_PKG_RESOURCES` 和 `SD_BALANCE_NEWIDLE`。
4. 记录产品 `.config` 中 `CONFIG_MIPS_CPS`、`CONFIG_MIPS_MT_SMP`、
   `CONFIG_SCHED_SMT`、`CONFIG_SCHED_MC`、`CONFIG_SCHED_DEBUG`；缺少这些信息无法
   唯一确定建出的 domain 层级。
5. 对 `idle_balance()` 入口/出口、放锁窗口、`load_balance()` 结果及
   `cfs.h_nr_running` 做有界 trace，证明同一 CPU 是否反复走正返回值与 `again`。
6. 同时追踪任务 enqueue/dequeue/migrate 与 TTWU 本地/远程路径；若无反复状态变化，
   应转查中断、LL/SC、cache coherency 或时钟，而不能把热点栈等同于调度器死循环。

这套验证能够区分三类问题：硬件编号/一致性错误、Linux 拓扑或 Kconfig 错误，以及
正确拓扑下由并发迁移触发的 CFS 重试竞态。
