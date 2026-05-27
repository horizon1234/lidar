# 激光扫描大气污染物与颗粒物系统路线图

> 目标：从零理解并规划一个“激光扫描大气，测量污染物、颗粒物，并进行 2D/3D 显示”的系统。  
> 结论先说在前面：这不是一台普通 3D LiDAR 能完成的事，而是“大气激光雷达 Atmospheric LiDAR + 光谱吸收/散射反演 + 地面传感器校准 + 时空可视化”的系统工程。

---

## 0. 最重要的现实判断

这里的LiDAR的含义是激光雷达，全称是Light Detection And Ranging，是用激光做测距和探测的雷达系统。而传统的Rader的含义是Radio Detection And Ranging，用的无线电波/微波。

1. **一台激光器不能同时准确测所有污染物。**
   - 颗粒物/气溶胶：主要靠弹性后向散射、偏振、Raman、HSRL。
   - 臭氧 O3：常用紫外 DIAL。
   - NO2、SO2：可用 DIAL/DOAS/吸收光谱，但工程难度高。
   - CH4、CO2：常用近红外 DIAL/IPDA，波长与探测器完全不同。
   - VOCs：很多成分很难用简单 LiDAR 定量，常需要 FTIR、TDLAS、质谱、被动光谱或点式传感器配合。

2. **LiDAR 对 PM2.5/PM10 不是“直接称重”。**
   - LiDAR 直接得到的是光学量：后向散射系数、消光系数、退偏振比、AOD、气溶胶层高度等。
   - PM2.5/PM10 质量浓度需要和地面 PM 监测仪、湿度、粒径分布、气溶胶类型一起标定。

3. **建议第一版不要直接做气体 DIAL。**
   - 更现实的路线是先做“颗粒物/气溶胶扫描 LiDAR + 地面站校准 + 2D/3D 可视化”。
   - 等数据链路、扫描、反演和软件跑通后，再选一个气体目标，例如 O3 或 CH4，单独做 DIAL/IPDA。

4. **激光安全是硬门槛。**
   - 大气 LiDAR 常涉及 Class 3B/4 激光，尤其 UV/1064 nm 不可见光，眼睛风险极高。
   - 户外发射还涉及空域/航空安全、激光安全员、联锁、遮光、监视和当地法规。

---

## 1. 系统能测什么

PM10指的是空气中直径小于10微米的颗粒物，也叫做可吸入颗粒物；而PM2.5是直径小于2.5微米的颗粒物，也叫细颗粒物。

| 目标 | 推荐方法 | 典型输出 | 难度 | 备注 |
|---|---|---|---|---|
| 气溶胶/颗粒物分布 | 弹性后向散射 LiDAR，偏振 LiDAR | attenuated backscatter、PBL 高度、云底、气溶胶层 | 中 | 最适合做第一版 |
| PM2.5/PM10 估算 | LiDAR 光学量 + 地面 PM 传感器 + 湿度校正 | 近地面/空间 PM 估算图 | 中高 | 必须做本地标定 |
| 沙尘/烟尘/污染气溶胶分类 | 多波长 + 偏振 + 机器学习 | dust/smoke/polluted/marine 等类型 | 高 | 532 nm 退偏振对沙尘有用 |
| 臭氧 O3 | UV DIAL | O3 垂直廓线 | 高 | 经典且成熟，但硬件复杂 |
| NO2 | 蓝紫光 DIAL / DOAS | NO2 廓线或路径平均浓度 | 高 | 常见吸收区约 400-450 nm |
| SO2 | UV DIAL / DOAS | SO2 廓线或羽流 | 高 | 常见在约 300 nm 紫外区 |
| CH4/CO2 | 近红外 DIAL/IPDA | 柱浓度/路径积分浓度 | 高 | 需要窄线宽可调激光和 InGaAs 探测 |
| 水汽 | Raman LiDAR / DIAL | 水汽混合比廓线 | 高 | 对气溶胶反演也有帮助 |
| VOCs | DIAL/FTIR/TDLAS/荧光，或点式传感器 | 单组分或类别估计 | 很高 | 不建议第一阶段做 |

---

## 2. 基本物理原理

一个大气LiDAR一般不是"激光器单独工作"，而是：

- 激光器负责照射目标体积；

- 望远镜负责收光；

- 探测器负责把光变成电信号；

- 采集系统负责按时间记录信号；

- 算法再把信号反演成后向散射、消光、边界层高度、PM估算等量。

### 2.1 弹性后向散射 LiDAR

先把这个词拆开看：

- **散射**：激光打进空气后，不是只沿直线继续前进；空气分子、气溶胶、烟尘、云滴会把一部分光打散到各个方向。
- **后向散射**：散射光里，恰好朝着 LiDAR 仪器方向返回的那一小部分，叫后向散射 backscatter。也就是“打出去的光，又有一小部分折返回来”。
- **弹性**：返回光的波长基本不变，只是方向变了、强度变弱了。这和 Raman 这类**非弹性散射**不同，后者返回光的波长会发生偏移。

所以，“弹性后向散射 LiDAR”可以理解成：

> 发出一束激光，观察大气里有多少光在**不改波长**的情况下被分子或颗粒物**散射回望远镜**。

它最适合回答的问题不是“这一定是哪种具体化学物质”，而是：

- 哪些高度或方向上有较强气溶胶/颗粒物；
- 云底、边界层、污染层顶大概在哪里；
- 某个烟羽或污染团有没有经过、变厚还是变薄。

可以把它想成一个“沿着激光束做分层探测的回声仪”，只是这里的“回声”不是声音，而是被大气散射回来的微弱光子。

NOAA 的这张示意图很适合建立直觉：左边是激光脉冲发射并从不同距离返回，右边是返回信号随距离衰减的典型曲线。

![NOAA LiDAR schematic](assets/noaa_lidar_schematic.gif)

图怎么读：

1. 激光器发出一个很短的脉冲。
2. 脉冲沿传播路径不断遇到空气分子、颗粒物、云滴。
3. 每一段距离都会有极少量光被散射回 LiDAR。
4. 望远镜把这些返回光收集到探测器。
5. 因为近处回波先到、远处回波后到，所以接收系统按时间采样，就能把时间换成距离。

接收望远镜按时间采样，时间对应距离：

$$
R = \frac{ct}{2}
$$

这里的 $2$ 不能漏，因为光要走一个来回：从仪器飞到目标体积一次，再从目标体积返回仪器一次。

对这类 LiDAR 来说，回波强通常意味着该距离处有更多散射体，或者散射更强；回波弱则可能意味着：

- 那里空气更干净；
- 激光传播途中已经被前面的介质衰减掉了；
- 距离更远，几何扩散更明显；
- 发射光束和接收视场在近距离还没有完全重叠。

还要注意，弹性后向散射 LiDAR 的原始输出更像一条“随距离变化的回波强度曲线”或“时间-高度剖面图”，不是普通相机那种直接拍到污染物外形的照片。

典型 LiDAR 方程可写成：

$$
P(R) = C E \frac{O(R)}{R^2} \beta(R) \exp\left[-2\int_0^R \alpha(r)\,dr\right]
$$

如果你数学基础比较弱，可以先不要把它当成“要手算的复杂公式”，而把它理解成一句话：

> 距离 $R$ 处收到的回波强度，等于“发射有多强”乘上“那里有多少东西把光打回来”再乘上“光在来回路上被削弱了多少”。

可以把这个式子拆成 5 块：

- $C$：系统常数。可以粗略理解成“这台仪器本身的整体效率”，里面打包了望远镜口径、光学透过率、探测器效率、电子增益等很多因素。
- $E$：激光单脉冲能量。激光打得越强，通常收到的回波也越强。
- $\dfrac{O(R)}{R^2}$：几何项。$O(R)$ 表示发射光束和望远镜视场在该距离处重叠得好不好；$R^2$ 表示距离越远，光能会摊得越开，所以回波会自然变弱。
- $\beta(R)$：后向散射系数。可以粗略理解成“距离 $R$ 那一层空气有多会把光散射回仪器”。
- $\exp\left[-2\int_0^R \alpha(r)\,dr\right]$：传输衰减项。它表示激光在去程和回程中，被大气散射和吸收削弱后的剩余比例。

### 2.1.1 这里的 exp 是什么意思

`exp(x)` 是数学里一种固定写法，意思就是：

$$
\exp(x) = e^x
$$

这里的 $e$ 是一个常数，约等于：

$$
e \approx 2.71828
$$

所以：

- $\exp(0) = 1$
- $\exp(-1) \approx 0.368$
- $\exp(-2) \approx 0.135$

你可以把它先当成一个“衰减按钮”：

- 里面如果是 $0$，说明不衰减，结果就是 $1$；
- 里面如果是负数，结果就在 $0$ 和 $1$ 之间，表示信号被削弱；
- 负得越厉害，结果越接近 $0$，表示衰减越强。

所以在 LiDAR 方程里：

$$
\exp\left[-2\int_0^R \alpha(r)\,dr\right]
$$

本质上就是在表达：

> 光走这一趟来回之后，还剩下多少比例没有被大气吃掉。

### 2.1.2 这里的积分到底是什么

你可以先把积分理解成：

> 沿着激光传播路径，把每一小段的衰减一点一点累加起来。

式子里的：

$$
\int_0^R \alpha(r)\,dr
$$

意思是：

- 从 $0$ 开始，也就是 LiDAR 仪器所在位置；
- 一直到距离 $R$；
- 看这一路上每个位置的消光系数 $\alpha(r)$；
- 然后把这些位置的衰减贡献全部加总起来。

这里的 $r$ 只是一个“路径上的临时位置变量”，你可以把它理解成“现在走到哪一段了”。

为什么这里不用 $R$ 而要写 $r$？

- $R$ 是最终想算的目标距离；
- $r$ 是从 $0$ 走到 $R$ 过程中，每一个中间位置。

比如你想算 1000 m 处的回波，积分就是把 0 m 到 1000 m 之间每一小段空气造成的衰减都累计起来。

### 2.1.3 为什么前面有个 -2

因为光不是只走单程，而是：

1. 从仪器飞到距离 $R$ 的那层空气；
2. 再从那层空气散射回仪器。

也就是同一段大气，光会穿过两次，所以衰减要乘以 2。

如果把它说得再直白一点，就是：

> 去的时候损失一次，回来的时候又损失一次。

### 2.1.4 这个积分怎么计算

如果 $\alpha$ 在整条路径上都差不多不变，也就是可以近似看成常数，那么积分就很简单：

$$
\int_0^R \alpha(r)\,dr \approx \alpha R
$$

这时方程里的衰减项就变成：

$$
\exp(-2\alpha R)
$$

举个非常粗略的数值例子。假设：

- $\alpha = 0.1\ \mathrm{km}^{-1}$
- $R = 3\ \mathrm{km}$

那么：

$$
\int_0^R \alpha(r)\,dr \approx 0.1 \times 3 = 0.3
$$

于是衰减项为：

$$
\exp(-2 \times 0.3) = \exp(-0.6) \approx 0.55
$$

这表示：

> 光在 3 km 处那一层空气完成一次来回后，大约只剩下原来的 55% 可以保留下来进入回波信号。

如果大气不均匀，$\alpha(r)$ 会随着距离变化，那就不能直接写成 $\alpha R$，而是要分段累加。最直观的算法是：

$$
\int_0^R \alpha(r)\,dr \approx \sum_i \alpha_i \Delta r
$$

这句话翻成大白话就是：

- 把整条路径切成很多小段；
- 每一小段都有一个自己的衰减系数 $\alpha_i$；
- 用“这一段的衰减系数 × 这一段的长度”；
- 最后把所有小段加起来。

例如分成 3 段，每段 1 km：

- 第 1 段：$\alpha_1 = 0.05\ \mathrm{km}^{-1}$
- 第 2 段：$\alpha_2 = 0.10\ \mathrm{km}^{-1}$
- 第 3 段：$\alpha_3 = 0.20\ \mathrm{km}^{-1}$

那么：

$$
\int_0^{3\mathrm{km}} \alpha(r)\,dr \approx 0.05 \times 1 + 0.10 \times 1 + 0.20 \times 1 = 0.35
$$

衰减项就是：

$$
\exp(-2 \times 0.35) = \exp(-0.7) \approx 0.50
$$

也就是说，来回之后大约剩一半。

### 2.1.5 你真正需要抓住的物理意思

如果暂时不管数学细节，这个方程最重要的是两件事：

1. 距离 $R$ 那一层本身要“能把光打回来”，这由 $\beta(R)$ 决定。
2. 光从仪器走到那里再走回来，途中不能被削弱得太厉害，这由 $\exp\left[-2\int_0^R \alpha(r)dr\right]$ 决定。

所以会出现一种常见情况：

- 远处其实有很多颗粒物；
- 但前面几层空气已经把激光削弱掉很多；
- 结果远处回波仍然可能很弱。

这也是为什么 LiDAR 反演并不是“哪里亮，哪里浓度就一定高”，而要结合整个传播路径一起分析。

含义：

- $P(R)$：距离 $R$ 处返回信号。
- $E$：单脉冲能量。
- $O(R)$：发射光束与接收视场的重叠函数。
- $\beta(R)$：后向散射系数，包含分子 Rayleigh 和颗粒物 Mie 散射。
- $\alpha(R)$：消光系数，包含散射和吸收导致的衰减。
- $R^{-2}$：距离平方衰减。

如果只用一句话概括这个方程，它表达的是：

> 某个距离上的回波强度，取决于那里“有多少东西把光散射回来”，也取决于激光在去程和回程中“被大气削弱了多少”。

问题是 `alpha` 和 `beta` 都未知，所以单波长弹性 LiDAR 必须引入假设，例如 lidar ratio：

$$
S = \frac{\alpha_{\mathrm{aerosol}}}{\beta_{\mathrm{aerosol}}}
$$

这就是 Klett/Fernald 反演方法的核心限制。

资料来源：

- NOAA Chemical Sciences Laboratory, LIDAR 原理页：<https://csl.noaa.gov/groups/csl3/instruments/dial/lidar.html>
- JPL NDACC, Instruments Description：<https://lidar.jpl.nasa.gov/ndacc/instruments/general.php>

前者对回波形成过程和示意图说明更直观，后者对“激光发射 - 望远镜接收 - 时间采样得到距离剖面”的定义更标准。

### 2.2 Raman LiDAR

Raman 通道接收的是非弹性散射，例如：

- 355 nm 发射，N2 Raman 约 387 nm。
- 532 nm 发射，N2 Raman 约 607 nm。

Raman 信号可以更独立地反演消光系数，减少 lidar ratio 假设，但信号弱，通常夜间效果好，需要更高能量和窄带滤光。

### 2.3 HSRL

HSRL 使用超窄谱滤波，把分子 Rayleigh 散射和颗粒物 Mie 散射分开，能独立求后向散射和消光。它很强，但光学系统复杂。

### 2.4 偏振 LiDAR

发射线偏振光，接收平行/垂直偏振分量，得到退偏振比：

$$
\delta = \frac{P_{\mathrm{cross}}}{P_{\mathrm{parallel}}}
$$

用途：

- 球形液滴退偏振低。
- 非球形沙尘、冰晶退偏振高。
- 可用于沙尘、烟尘、云、污染气溶胶分类。

### 2.5 DIAL

DIAL 用两束相近波长：

- $\lambda_{\mathrm{on}}$：目标气体吸收强。
- $\lambda_{\mathrm{off}}$：目标气体吸收弱。

通过两条回波曲线的差异反演目标气体浓度：

$$
N(R) \approx \frac{1}{2\left(\sigma_{\mathrm{on}} - \sigma_{\mathrm{off}}\right)} \frac{d}{dR} \ln\left(\frac{P_{\mathrm{off}}(R)}{P_{\mathrm{on}}(R)}\right)
$$

实际还要修正：

- 气溶胶差分后向散射。
- 分子散射。
- 温度/压力导致的吸收截面变化。
- 激光线宽和波长漂移。
- 两个波长发射时间差导致的大气变化误差。

---

## 3. 激光器选择

### 3.1 第一版推荐：颗粒物/气溶胶扫描

优先考虑以下路线：

| 路线 | 激光 | 探测器 | 优点 | 缺点 |
|---|---|---|---|---|
| 低功率 ceilometer/MPL | 905/910/1064/1550 nm 脉冲二极管或微脉冲 | APD/SPAD | 相对安全、可连续运行 | 定量能力有限，近地面重叠问题明显 |
| 科研型弹性 LiDAR | Nd:YAG 532/1064 nm，可加 355 nm | PMT/APD | 成熟、资料多、可做多波长 | Class 4，安全和法规要求高 |
| 多波长偏振 LiDAR | 355/532/1064 nm + 偏振通道 | PMT/APD | 可做气溶胶类型识别 | 光路、标定复杂 |
| Raman LiDAR | 355/532 nm + N2/H2O Raman 通道 | PMT | 消光反演更可靠 | 夜间优先，信号弱 |

如果你是从零开始，推荐：

```text
阶段 1：用公开数据和软件先做算法/可视化
阶段 2：买/借一台 ceilometer 或微脉冲 LiDAR 做连续廓线
阶段 3：升级到 532 nm 偏振 + 1064 nm 多波长
阶段 4：再做 Raman 或 DIAL
```

### 3.2 气体 DIAL 波长大致方向

| 气体 | 常见技术方向 | 典型波段示例 | 难点 |
|---|---|---|---|
| O3 | UV DIAL | 对流层常见 289/299 nm，平流层常见 308/355 nm | UV 光源、臭氧强吸收、眼/皮肤安全 |
| NO2 | 蓝紫光 DIAL | 约 446/448 nm 或 400-450 nm 区域 | 吸收谱复杂、太阳背景、气溶胶干扰 |
| SO2 | UV DIAL | 约 300 nm 附近 | UV 光源、臭氧/气溶胶交叉干扰 |
| CH4 | NIR/SWIR DIAL/IPDA | 约 1.65 um 或 2.3 um 附近 | 窄线宽、波长锁定、InGaAs/HgCdTe |
| CO2 | NIR/SWIR DIAL/IPDA | 约 1.57 um 或 2.05 um 附近 | 温压修正、柱浓度反演 |
| H2O | DIAL/Raman | 720/820/935 nm 或 Raman | 湿度变化快，需要气象资料 |

### 3.3 激光参数怎么定

关键指标：

- **波长**：决定测颗粒物还是某种气体。
- **脉冲宽度**：决定距离分辨率。10 ns 对应约 1.5 m 理论距离宽度，但实际常按 7.5 m、15 m、30 m 或 90 m 分箱。
- **重复频率**：高频能提高平均信噪比，适合连续监测。
- **单脉冲能量**：越高越远，但越危险。
- **线宽**：DIAL/IPDA 对线宽和波长稳定性要求高。
- **发散角**：影响空间分辨率、眼安全和接收重叠。
- **光束质量 M2**：影响准直和远距离能量密度。
- **稳定性**：回波反演需要发射能量监测并归一化。

---

## 4. 硬件系统结构

### 4.1 发射端

组成：

- 激光器。
- 扩束镜，降低发散角和近场能量密度。
- 快门/安全 shutter。
- 能量监测 photodiode。
- 波长监测，DIAL 需要 wavemeter 或锁频。
- 偏振控制，偏振 LiDAR 需要半波片/偏振片。
- 扫描头：二维转台、振镜、楔形棱镜或大口径扫描镜。

### 4.2 接收端

组成：

- 望远镜：牛顿、卡塞格林、折反射，常见口径 100-500 mm。
- 光纤或自由空间耦合。
- 分光：二向色镜、窄带滤光片、偏振分束器。
- 探测器：
  - PMT：UV/可见常用。
  - APD/SPAD：近红外或低光子计数。
  - InGaAs APD：1.5-1.7 um。
- 采集：
  - photon counting。
  - analog digitizer。
  - 多通道同步采样。

### 4.3 扫描方式

| 模式 | 含义 | 用途 |
|---|---|---|
| 垂直定点 | 朝天连续测 | 边界层、云底、气溶胶层 |
| RHI | 固定方位，扫仰角 | 垂直剖面、烟羽截面 |
| PPI | 固定仰角，扫方位 | 水平污染扩散 |
| Sector scan | 只扫重点方向 | 工厂/道路/港口源监测 |
| Volume scan | 多个 PPI/RHI 组合 | 三维体数据 |
| Stare mode | 固定看某一路径 | 高时间分辨率事件监测 |

### 4.4 标定与辅助传感器

必备：

- 地面 PM2.5/PM10 参考仪。
- 气象站：温度、湿度、气压、风速风向。
- GPS 时间和位置。
- 云量/全天空相机。
- 发射能量计。
- 暗电流/背景光采样。

强烈建议：

- AERONET 或太阳光度计，用 AOD 约束 LiDAR。
- 低成本传感器阵列，辅助做近地面空间校准。
- 风廓线雷达或多普勒 LiDAR，用于污染输送解释。

---

## 5. 数据处理全流程

### 5.1 原始数据

每个激光 shot 或累积 profile 应保存：

- 时间戳。
- 方位角、仰角。
- 波长、通道、偏振。
- 原始回波数组：counts 或 voltage。
- 发射能量。
- 探测器增益、门控状态。
- 温度、湿度、气压。
- 系统状态：激光温度、水冷、shutter、转台状态。

推荐文件格式：

- 原始：HDF5 或 NetCDF。
- 标准产品：CF-compliant NetCDF。
- 大规模在线：Zarr + object storage。
- 实时索引：PostgreSQL/PostGIS + TimescaleDB。

### 5.2 预处理

顺序建议：

1. 时间同步和角度同步。
2. 背景光扣除，常用远距离无信号区估计背景。
3. 暗电流扣除。
4. 探测器死时间校正，photon counting 必做。
5. afterpulse 校正。
6. 饱和检查。
7. 发射能量归一化。
8. 距离平方校正：

$$
\mathrm{RCS}(R) = P(R) R^2
$$

9. overlap function 校正，近距离尤其重要。
10. 时间/距离平滑或重采样。
11. 云/降水/异常信号 mask。
12. SNR 估计和质量标志。

### 5.3 气溶胶反演

第一版产品：

- range-corrected signal。
- attenuated backscatter。
- boundary layer height。
- cloud base height。
- aerosol layer top/bottom。

第二版产品：

- particle backscatter coefficient。
- extinction coefficient。
- lidar ratio。
- AOD。
- volume depolarization ratio。
- particle depolarization ratio。
- Angstrom exponent。

常用算法：

- Klett inversion。
- Fernald inversion。
- Raman extinction/backscatter retrieval。
- HSRL retrieval。
- AOD-constrained inversion。
- GRASP/GARRLiC：LiDAR + sun/sky photometer 联合反演。

### 5.4 PM2.5/PM10 估算

LiDAR 光学量转 PM 质量浓度，建议做成机器学习/统计标定问题：

$$
\mathrm{PM}_{2.5} = f\left(\mathrm{extinction}_{532}, \mathrm{backscatter}_{532}, \mathrm{depol}, RH, T, \mathrm{wind}, \mathrm{aerosol\_type}, \mathrm{hour}, \mathrm{season}\right)
$$

注意：

- 相对湿度 RH 对散射影响很大，颗粒吸湿增长会让光学量变大，但质量浓度不一定等比例变大。
- 不同地区气溶胶成分不同，模型需要本地训练。
- 雾、云、降水要单独 mask。
- PM2.5 与 PM10 的差别需要粒径信息，多波长和地面粒径谱仪会明显改善结果。

### 5.5 气体 DIAL 反演

需要额外数据：

- HITRAN/GEISA 吸收线数据库。
- 温度、压力廓线。
- 激光波长与线宽。
- on/off 波长的吸收截面。
- 气溶胶差分修正。

基础处理：

1. on/off 通道分别预处理。
2. 计算 $\ln\left(P_{\mathrm{off}} / P_{\mathrm{on}}\right)$。
3. 对距离求导，通常需要正则化或平滑。
4. 修正 Rayleigh、气溶胶、其他气体吸收。
5. 输出浓度廓线和不确定度。

---

## 6. 软件架构建议

### 6.1 后端数据管线

推荐技术栈：

- Python：NumPy、SciPy、xarray、dask、netCDF4、h5py、zarr。
- 反演算法：自研 Klett/Fernald/Raman/DIAL 模块，参考 SCC/Picasso/CloudnetPy。
- 气体谱线：HITRAN HAPI。
- Mie 散射：miepython 或 PyMieScatt。
- API：FastAPI。
- 时序数据库：TimescaleDB。
- 空间数据库：PostGIS。
- 异步任务：Celery/RQ/Prefect。
- 消息流：MQTT、Kafka 或 ZeroMQ。

### 6.2 产品分级

| 级别 | 内容 |
|---|---|
| L0 | 原始采样、原始 metadata |
| L1 | 背景扣除、能量归一化、距离校正、质量标志 |
| L1.5 | attenuated backscatter、depolarization、cloud mask |
| L2 | backscatter/extinction/AOD/PM 估算/气体浓度 |
| L3 | 网格化 2D/3D 产品、日/月统计、事件识别 |

### 6.3 2D 显示

必须做：

- 时间-高度 curtain plot。
- 距离-角度 RHI 剖面。
- PPI 极坐标污染图。
- 地图叠加：MapLibre/Cesium/Leaflet。
- 多通道曲线：原始信号、RCS、反演廓线。
- QA 图层：SNR、云 mask、饱和、雨雾标志。

推荐图表：

- ECharts/Plotly：曲线、热力图。
- Matplotlib/Cartopy：科研离线图。
- Datashader：大数据栅格渲染。

### 6.4 3D 显示

数据不是普通物体点云，而是扫描体素/射线网格。

推荐表达：

- 每条激光射线按距离分箱，形成 `(azimuth, elevation, range)`。
- 转换到本地 ENU 坐标：

$$
\begin{aligned}
x &= R \cos(\mathrm{elevation}) \sin(\mathrm{azimuth}) \\
y &= R \cos(\mathrm{elevation}) \cos(\mathrm{azimuth}) \\
z &= R \sin(\mathrm{elevation})
\end{aligned}
$$

显示方式：

- 点云：每个 range bin 一个带颜色/透明度的点。
- 体渲染：把 backscatter/extinction 插值到 3D grid。
- 等值面：显示污染羽流边界。
- 切片：水平切片、垂直切片、任意剖面。
- 动画：时间序列播放污染输送。

推荐库：

- Web 3D：Three.js、deck.gl、CesiumJS。
- 科研 3D：PyVista/VTK、Open3D、Mayavi。
- 大点云：Potree、Entwine、COPC/LAZ。

---

## 7. 现有软件 UI 参考

下面这些图来自公开的官方文档或开源项目示例。我已经把图片保存到本项目的 `assets/ui_references/` 目录，方便你直接打开这个 Markdown 看效果。注意：这些图主要用于理解界面形态，不代表可以直接复制商用软件设计或商标元素。

### 7.1 CloudnetPy / Cloudnet 风格：科研 quicklook 产品

CloudnetPy 是 ACTRIS Cloudnet 生态里的开源处理工具，常见 UI/图件不是华丽 3D，而是非常科研化的时间-高度图。用户重点看的是“某一天、某个站点、不同高度上云/气溶胶/降水随时间怎么变”。

来源：  
https://actris-cloudnet.github.io/cloudnetpy/quickstart.html

典型 LiDAR attenuated backscatter 图：

![CloudnetPy lidar attenuated backscatter](assets/ui_references/cloudnetpy_lidar_backscatter.png)

目标分类图：不同颜色代表不同大气目标，例如气溶胶、液态云、冰云、昆虫、降水等。这类图很适合做污染/云/雾/沙尘的质量控制和分类界面。

![CloudnetPy target classification](assets/ui_references/cloudnetpy_target_classification.png)

多仪器联合 quicklook：雷达反射率、Doppler 速度、LiDAR 衰减后向散射、液态水路径放在同一页，方便专家快速判断天气和气溶胶结构。

![CloudnetPy multi instrument overview](assets/ui_references/cloudnetpy_multi_instrument_overview.png)

这个界面给你的启发：

- 第一屏不一定是 3D，最常用的是 **时间-高度 curtain plot**。
- 颜色条必须清楚标单位，例如 $sr^{-1}\,m^{-1}$、$dBZ$、$m\,s^{-1}$。
- 空白/mask 区域很重要，不能为了“好看”把无效数据强行插值。
- 同一时间轴上叠加多个产品，比单独看一张图更有诊断价值。

### 7.2 Vaisala BL-View 风格：操作员实时监控 UI

Vaisala BL-View 是面向边界层和云高仪数据的软件，界面更像“仪器监控台”：顶部是站点、时间、工具和系统状态，中间是大面积时间-高度热力图，叠加边界层高度/云底等检测线。

来源：  
https://docs.vaisala.com/r/M211185EN-E/en-US/GUID-EF63D824-E0FA-437C-A1F8-FCFC6DFDADD7

BL-View 主界面：

![Vaisala BL-View main UI](assets/ui_references/vaisala_blview_main_clean.png)

归档数据查看界面：

![Vaisala BL-View archive plot](assets/ui_references/vaisala_blview_archive_clean.png)

这个界面给你的启发：

- 操作员需要的是 **持续运行、状态明确、异常容易发现**。
- 顶部应显示：当前站点、仪器状态、激光状态、采集时间、告警、数据延迟。
- 主画面要支持 live/default/archive 标签页，方便实时和历史切换。
- 图上应能叠加算法结果，例如边界层高度、云底高度、污染层顶/底、SNR mask。

### 7.3 Vaisala CL-View / CL61 风格：云高仪后向散射图

这类界面和大气污染 LiDAR 很接近：纵轴高度、横轴时间，颜色是后向散射强度，再叠加云底或边界层检测线。

来源：  
https://docs.vaisala.com/r/M212721EN-D/en-US/GUID-34D2DC29-AC43-404F-9F80-3199EF7F9E36

CL61 后向散射图：

![Vaisala CL61 backscatter profile](assets/ui_references/vaisala_cl61_backscatter_clean.png)

这个界面给你的启发：

- PM/气溶胶产品可以先做成类似“后向散射热力图 + 反演线 + 质量标志”的形式。
- 对业务用户来说，蓝绿黄红的颜色比复杂点云更直观。
- 需要支持鼠标悬停读数：时间、高度、后向散射、消光、估算 PM、SNR、质量标志。

### 7.4 如果做成自己的软件，建议 UI 长这样

#### 页面 1：实时监控

布局：

- 顶部状态栏：站点、经纬度、当前时间、激光安全状态、采集状态、风速风向、湿度、数据延迟。
- 左侧：地图和扫描扇区，显示 LiDAR 位置、扫描方向、污染源、地面站。
- 中间主图：实时 time-height curtain plot。
- 右侧：当前廓线曲线，显示 backscatter、extinction、PM2.5 估算、SNR。
- 底部：时间轴、播放/暂停、历史回放、事件标记。

#### 页面 2：RHI/PPI 扫描

布局：

- RHI：距离-高度剖面，适合看烟羽抬升和边界层结构。
- PPI：地图上的扇形扫描图，适合看水平扩散。
- 叠加风场箭头、污染源位置、行政区/道路/厂区边界。
- 支持切换产品：RCS、attenuated backscatter、extinction、PM2.5、depolarization、gas concentration。

#### 页面 3：3D 体显示

布局：

- 左侧图层管理：地形、建筑、污染源、地面站、扫描轨迹、气象场。
- 主视图：WebGL 体素/点云/切片。
- 右侧属性面板：当前点的时间、坐标、高度、浓度、质量标志。
- 底部时间滑块：播放污染羽流随时间移动。

#### 页面 4：数据质量与标定

布局：

- 原始信号、背景、暗电流、发射能量、探测器状态。
- overlap 校正曲线、SNR、云/雨/雾 mask。
- 地面 PM/O3 仪器对比。
- 日/月统计、误差、相关系数、模型漂移。

---

## 8. 第一阶段 MVP 方案

如果目标是“先做出能演示的软件和算法闭环”，建议这样做：

### MVP-1：公开数据可视化

目标：

- 下载 TOLNet/EARLINET/Cloudnet/AERONET/OpenAQ 或 EPA AQS 数据。
- 做时间-高度图、地图、廓线、污染事件回放。

输出：

- Web 页面：地图 + 2D curtain plot + 廓线图。
- 支持 NetCDF/HDF5 导入。
- 支持 PM/O3 地面站对比。

### MVP-2：模拟 LiDAR 数据

目标：

- 用标准大气 + Mie 散射 + 假设气溶胶层生成回波。
- 跑 Klett/Fernald 反演。
- 用噪声模型测试反演稳定性。

输出：

- 可控测试数据。
- 算法单元测试。
- 误差和不确定度图。

### MVP-3：低功率硬件接入

目标：

- 接入 ceilometer/MPL 或实验室低功率激光回波。
- 做实时采集、L1 预处理、2D 显示。

输出：

- 实时 profile。
- 云底/边界层高度。
- 与地面 PM 传感器对比。

### MVP-4：扫描和 3D

目标：

- 加二维转台。
- 做 RHI/PPI/volume scan。
- 转换到 3D 坐标并做体素显示。

输出：

- 三维污染羽流/气溶胶层浏览器。
- 事件录像和统计报表。

### MVP-5：单一气体 DIAL

目标：

- 只选一个气体，例如 O3。
- 做 on/off 波长控制、吸收截面、DIAL 反演。

输出：

- O3 廓线。
- 与臭氧探空/地面 O3 仪对比。

---

## 9. 关键风险

1. **近距离盲区**：发射光束和接收视场未完全重叠，几十到几百米内数据不可靠。
2. **太阳背景**：白天 UV/可见通道噪声大，Raman 尤其困难。
3. **多重散射**：浓雾、云、强污染时单散射假设失效。
4. **湿度影响**：RH 会显著改变颗粒散射。
5. **DIAL 波长误差**：波长偏一点，浓度误差可能很大。
6. **气溶胶类型变化**：同样 PM 浓度可能对应不同散射强度。
7. **安全/法规**：Class 3B/4、UV、不可见光、户外空域都是硬约束。
8. **定量验证**：没有参考仪器，就很难证明测得准。

---

## 10. 推荐论文与资料

### 9.1 入门与原理

- NOAA 对大气 LiDAR 的简明介绍：  
  https://csl.noaa.gov/groups/csl3/instruments/dial/lidar.html
- JPL/NDACC 大气 LiDAR 技术概览：  
  https://lidar.jpl.nasa.gov/ndacc/instruments/general.php
- JPL Rayleigh/Raman aerosol retrieval：  
  https://lidar.jpl.nasa.gov/ndacc/instruments/Aerosols.php

### 9.2 经典反演算法

- Klett, 1981, *Stable analytical inversion solution for processing lidar returns*：  
  https://pubmed.ncbi.nlm.nih.gov/20309093/
- Fernald, 1984, *Analysis of atmospheric lidar observations: some comments*：  
  https://opg.optica.org/abstract.cfm?uri=ao-23-5-652
- Ansmann et al., 1990, Raman aerosol extinction：  
  https://opg.optica.org/ol/abstract.cfm?uri=ol-15-13-746
- Ansmann et al., 1992, combined Raman elastic-backscatter lidar：  
  https://opg.optica.org/ao/abstract.cfm?uri=ao-31-33-7113
- Aerosol extinction/backscatter review, 2021：  
  https://www.sciencedirect.com/science/article/pii/S0022407320310207

### 9.3 DIAL 与气体污染物

- JPL DIAL ozone technique：  
  https://lidar.jpl.nasa.gov/ndacc/instruments/Ozone.php
- NOAA TOPAZ ozone/aerosol DIAL：  
  https://csl.noaa.gov/groups/csl3/instruments/topaz/
- NASA TOLNet 臭氧 LiDAR 网络：  
  https://impact.earthdata.nasa.gov/casei/instrument/TOLNet/
- Standoff chemical detection review, Remote Sensing 2020：  
  https://www.mdpi.com/2072-4292/12/17/2771/htm
- NO2 DIAL 示例，ScienceDirect 2024：  
  https://www.sciencedirect.com/science/article/pii/S0143816624003233

### 9.4 开源处理链与数据

- ACTRIS/EARLINET Single Calculus Chain 论文：  
  https://amt.copernicus.org/articles/8/4891/2015/
- EARLINET SCC 说明：  
  https://www.earlinet.org/index.php?id=281
- EARLINET 数据访问：  
  https://earlinet.eu/our-knowledge/access-to-data/
- CloudnetPy：  
  https://github.com/actris-cloudnet/cloudnetpy
- Cloudnet 数据门户：  
  https://cloudnet.fmi.fi/
- PollyNET/Picasso processing chain：  
  https://pollynet.github.io/Pollynet_Processing_Chain/overview.html
- PyLidar：  
  https://www.pylidar.org/
- ARC-ACTRIS Rayleigh/Raman scattering Python package：  
  https://pypi.org/project/arc-actris/
- GRASP open aerosol retrieval：  
  https://www.grasp-open.com/
- GARRLiC algorithm paper：  
  https://amt.copernicus.org/articles/6/2065/2013/amt-6-2065-2013.html

### 9.5 光谱、气溶胶和地面空气质量数据

- HITRAN 数据库：  
  https://hitran.org/
- HITRAN HAPI Python API：  
  https://hitran.org/hapi/
- miepython：  
  https://github.com/scottprahl/miepython
- PyMieScatt：  
  https://pymiescatt.readthedocs.io/
- AERONET：  
  https://aeronet.gsfc.nasa.gov/
- NASA AERONET 简介：  
  https://science.gsfc.nasa.gov/earth/projects/97/
- OpenAQ API：  
  https://docs.openaq.org/about/about
- EPA AQS：  
  https://www.epa.gov/aqs
- EPA AQS API：  
  https://aqs.epa.gov/aqsweb/documents/data_api.html

### 9.6 数据标准与安全

- CF NetCDF metadata conventions：  
  https://cfconventions.org/
- OGC SensorThings API：  
  https://ogcapi.ogc.org/sensorthings/overview.html
- FDA laser products and instruments：  
  https://www.fda.gov/radiation-emitting-products/home-business-and-entertainment-products/laser-products-and-instruments
- FAA Outdoor Laser Operations AC 70-1B：  
  https://www.faa.gov/regulations_policies/advisory_circulars/index.cfm/go/document.information/documentID/1040741

---

## 11. 推荐学习顺序

1. 先读 NOAA/JPL 的 LiDAR 入门页，理解大气回波不是普通点云。
2. 读 Klett/Fernald，理解为什么单波长弹性 LiDAR 需要假设。
3. 读 Raman/HSRL，理解如何减少假设。
4. 用 CloudnetPy、EARLINET、PollyNET 数据看真实产品长什么样。
5. 写一个模拟器：给定气溶胶层、消光、噪声，生成回波。
6. 写 Klett/Fernald 反演和质量控制。
7. 做 2D 时间-高度图和 RHI/PPI 图。
8. 做 3D 射线体素渲染。
9. 接地面 PM/O3 数据做标定。
10. 最后才做 DIAL，且一次只做一个气体。

---

## 12. 建议的项目目录

```text
atmospheric-lidar/
  docs/
    system_design.md
    safety_plan.md
    calibration_plan.md
  data/
    raw/
    l1/
    l2/
    public_samples/
  lidar_core/
    io/
    preprocessing/
    retrieval/
    simulation/
    calibration/
    qa/
  services/
    api/
    workers/
  web/
    map/
    curtain_plot/
    volume_viewer/
  notebooks/
    01_public_data.ipynb
    02_lidar_equation_simulation.ipynb
    03_klett_fernald.ipynb
    04_pm_calibration.ipynb
  tests/
```

---

## 13. 最小可行技术路线

如果现在就要启动，我建议这样落地：

1. **第 1 周：公开数据软件原型**
   - 读 NetCDF。
   - 显示时间-高度图。
   - 对接 OpenAQ/EPA 地面 PM/O3。

2. **第 2-3 周：LiDAR 模拟与反演**
   - 实现 LiDAR 方程模拟器。
   - 实现背景扣除、距离校正、SNR。
   - 实现 Klett/Fernald。

3. **第 4-6 周：2D/3D 可视化**
   - RHI/PPI 扫描格式。
   - 3D 坐标转换。
   - WebGL 体素/点云显示。

4. **第 7-12 周：真实硬件或公开真实数据验证**
   - 接 ceilometer/MPL 或公开 Raman/HSRL 数据。
   - 做 L1/L2 产品。
   - 与地面站校准 PM 估算。

5. **后续：单气体 DIAL**
   - 只选一个目标气体。
   - 用 HITRAN 建吸收模型。
   - 做 on/off 回波仿真。
   - 再进入硬件设计。
