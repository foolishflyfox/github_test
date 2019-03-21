---
export_on_save:
 html: true
---

# pypedsim

**注：本文中数学公式如果不能显示，可以访问 [html页面](http://htmlpreview.github.com/?https://github.com/foolishflyfox/pypedsim/blob/master/README.htm)**

该库将 pedsim 2.4.2 进行 python 版本的重实现，源码在本仓库的 pedsim_cpp 文件夹中，也可以从 <http://pedsim.silmaril.org/download/> 下载源代码。

PEDSIM 的官网地址为：<http://pedsim.silmaril.org/>。

本项目使用浏览器进行gui的构建，所使用的是 *bottle* 框架，该框架的指南源地址为 <http://bottlepy.org/docs/dev/tutorial.html#quickstart-hello-world>，为了记录以及方便，可以访问 [中文版](/bottle_tutorial.md)。

## pedsim 模型说明

**在该版本中，并没有使用下面的模型，主要是因为根据论文中的公式建模后，需要进行参数调节才能更好地进行拟合，目前并没有地找到进行参数调节的工具，因此所有的代码都从原始的 pedsimlib 库中改造而来，后期如果找到好的调参方法，将会重启下面的社会力计算公式。**

pedsim 使用的是微观社会力模型。

仿真代码需要考虑了个体间的物理影响，例如个体与环境以及其他个体之间的交互。通常解决这些问题的仿真技术有：

- 使用微观模型，每一个个体都有独立的逻辑。
- 在宏观或基于场的模拟中，粒子被聚集在场中，对应的数学模型是偏微分方程，为了在计算机中实现，需要进行离散化。
- 可将微观模型和基于场的方法相结合，这种方法有时候也成为平滑粒子流体动力学(smooth particle hydrodynaics)。在 SPH 中，每个粒子的特性都被保留。在每一步的更新过程中，将例子聚集成诸如密度的场量，然后根据这些场量进行速度计算，然后根据这些宏观的速度对粒子进行移动。
- 在某种程度上，有的研究团队提出了排队模拟模型：粒子在队列网络中移动，每个队列都有一个服务速率，在对一个例子完成服务后，该例子被移动到下一个队列中。

在 PEDSIM 中，我们需要保持每个粒子各自的属性，因为他们需要都是相互独立的个体，仿真过程中需要各自做出决策，例如路由选择，因此我们不会使用基于场的模型。另外我们的仿真过程还需要例子间的相互作用，因此队列模型和SPH都不合适。

在微观模拟中主要存在着两种技术：基于耦合微分方程的方法和元胞自动机(cellular automata, CA)模型。在我们的模型中，个体应当能够向着任意方向进行移动，因此元胞自动机模型也被排除了。

社会力模型(social force model)是用于行人模拟的最通用的耦合微分方程模型，由 Helbing 等人提出：
$$m_i \frac{\mathrm d \bm v_i}{\mathrm dt}=m_i\frac{\bm v_i^0-\bm v_i}{\tau_i}+\sum_{j\ne i}\bm f_{ij}+\sum_{W}\bm f_{iW} \tag{1}$$


其中：$m$ 是行人质量，$v$ 是其速度。$v_0$是其期望速度(可以达到的最大速度，desired velocity)。在等式右边第一项，指数逼近期望速度，其中 $\tau$ 是一个常数；第二项是行人间的相互作用；第三项是环境对行人的作用，例如墙、障碍物等。社会力模型应该被视为处理行人交互的一个例子。该模型易于理解和实现，之后PEDSIM实现的模型可能和原始的社会力模型可能会有所不同。

行人间交互包括避免碰撞（短程交互），被 enemies 所吸引（长程交互，表示行人的意愿(will)，当然，被 enemies 所吸引仅仅是一种示例，可以被更加复杂和有意义的函数所替代）。另外，对类似树的避障也已近实现。

运动仿真不仅仅需要对行人的物理特性进行仿真，还需要能够计算更高层次的仿真，例如行人的心理层面。

- 物理层面：处理系统的物理方面的逻辑，例如agents的运动、agents与环境之间的交互，agents之间的交互；
- 心理层面：实现人的智能（至少部分实现），可以能够使仿真模型更加逼真。实际上，如果精神层面的策略足够精妙，社会力模型的物理仿真就不需要了——所有的力都设为0。**Look Ahead** 心理策略是指：每个agent都会看着在其前面的行人，并且会估计在其左侧的行人数量以及在其右侧的行人数量，agent 将会选择人数更少的一侧。与墙以及其他agent的碰撞避免是通过agent自身实现的，而不是因为物理因素。立一个精神层面的例子是路由产生器（**Route Generator**），我们不仅仅需要agent能够随机地移动，在实际的应用中，我们需要仿真过程能够为每个行人产生真实可信的运动轨迹。如果要使用路由产生器计算行走路径，我们必须知道agents的目标位置。在交通运输的研究中，有种技术是为每个agent、每个活动指定确定的位置，并产生一天的活动连。有一些非常好的心理层面的模型，例如说**View Analyzer Module**，该模型描述了agents在移动的过程中所看到的场景。agents所观察的场景会被分析，并根据agent看到的内容进行系统描述。

## 模型详细分析

在 ped_agent 源代码中的模型逻辑：1、每个agent各自计算作用在其上的合力；2、根据计算结果，有scene对象更新行人位置、速度信息；

### 合力计算

$$\frac{\mathrm d\bm v_i}{\mathrm dt}=K_{des}f_{des}+K_{soc}f_{soc}+K_{obs}f_{obs}+K_{look}f_{look}+f_{my} \tag{2}$$

其中，des=desired force，soc=social force，obs=obstacle force，look=look ahead force，my=my force。$K_{\mathrm{xxx}}$表示常数因子。

#### disired force 的计算

假定agent当前的位置为A，目标位置为B，则$f_{des}$的计算公式为：$$f_{des}=v^0\vec{\bm e}_{AB} \tag{3}$$
注意：此处对原始的社会力模型公式(1)中的第一项进行了修改；


#### social force 的计算

注意：对社会力的计算和源代码完全不同，而是采用胡学敏等人在《基于人机社会力模型的人群疏散算法》中的公式；

假设 S 为所有对本 agent 产生影响的 neighbors；

其中的某个agent的id为 $j$，其位置为B，本agent的id为$i$，位置为A，则定义$\vec n_{ij} = \vec\bm e_{BA}$ 为从B到A的单位向量；

$\vec t_{ij}$ 表示 $\vec n_{ij}$ 的单位法向量，与具体的朝向无关；$\Delta\vec\bm v_{ij}=\vec\bm v_j - \vec\bm v_i$，表示 j 相对于 i 的相对运动速度，则 $(\Delta \vec \bm v_{ij}\cdot \vec\bm t_{ij})\vec\bm t_{ij}$ 表示由于agent接触后摩擦力引起的force；

设 $r_i$ 为 i 的半径，$r_j$ 为 $r_{j}$ 为 j 的半径，$r_{ij}=r_i+r_j$，$d_{ij}$ 表示 i 和 j 质心之间的距离；

再定义一个函数斜坡函数 $g(x)=\max(0, x)$

最后定义几个常数：$A_{soc}$ 表示力的作用强度，默认值为20，$B_{soc}$ 表示力的作用范围，默认值为0.08，$k$ 和 $\kappa$ 为常数，默认值分别为 1200 和 2400。(注意，这里和论文中的参数值不一样，因为pedsim合力其实是加速度，需要除以质量，这里假设质量为 100kg，方便计算)

<!-- 本agent的速度为$\vec\bm v_i$，id为 $j$ 的agent的速度为$\vec\bm v_j$，$\Delta \vec\bm v_{ji} = \vec\bm v_i-\vec\bm v_j$；（在 python 中进行了修改，该变量未经使用） -->

则$f_{soc}$的计算公式为：$$f_{soc}=\sum_{j\in S}f_j \tag{4}$$
其中 $f_j$ 的计算公式为 $$f_j=[A_{soc}\exp(\frac{r_{ij}-d_{ij}}{B_{soc}})+k\cdot g(r_{ij}-d_{ij})]\vec n_{ij} \\ + \kappa\cdot g(r_{ij}-d_{ij})\cdot(\Delta \vec \bm v_{ij}\cdot \vec\bm t_{ij})\vec\bm t_{ij} \tag{5}$$

解释：因为人总是有与他人保持一定距离的趋势，因此在agent之间不接触时， $r_{ij}-d_{ij}$ 小于0，并且两个agent相距越大 -> 该值越小 -> $f_j$由j指向i的力（即斥力）也就越小；最后一项表示在有身体接触的情况下，agent i 有被 j 的相对速度所影响的趋势；

#### obstacle force 的计算

obstacle的计算与social force的计算非常类似：$$f_{obs}=[A_{soc}\exp(\frac{r_{i}-d_{io}}{B_{soc}})+k\cdot g(r_{i}-d_{io})]\vec n_{io} \\ + \kappa\cdot g(r_{i}-d_{io})\cdot(-\vec \bm v_{i}\cdot \vec\bm t_{io})\vec\bm t_{io} \tag{6}$$

#### look ahead force 的计算


agent 会在行走的过程中，初略统计在其视野范围内的行人数量，并且有向着人数较少的一侧运动的趋势；

考虑 social force 中就具有类似的功能，故将 look ahead force 直接设置为0，另一方面也可以极大地降低计算量。

<!-- 假设行人的朝向为 $\vec e_v$，其视野范围为 -2.5rad ~ 2.5rad，其0.5 ~ 2.5范围视为左侧，统计得到人数$P_{l}$其-0.5 ~ -2.5范围视为右侧，统计得到人数$P_{r}$，$\vec e_t$表示$\vec e_v$的左侧法向量，为了避免除零错误，设置常数$P_{min}$=3，则 $f_{look}$ 的计算为：$$f_{look}=\frac{P_l-P_r}{min(P_l, P_r, P_{min})}\vec e_t$$ -->

### 信息更新

信息更新过程分为两步：1、更新行人位置；2、更新行人速度；


## 代码分析

### Tobstacle

该类用于表示仿真过程中的障碍物，模拟过程中的障碍物都为矩形，主要保存的是 A点`(ax, ay)` 和 B点`(bx, by)` ，由这两个点确定障碍物的大小与位置。方法 `closestPoint`的传入参数也是一个点P，该方法确定的是在线段 AB 上离 P 点最近的一个点 P'。

### Tagent

该类用于表示仿真过程中的行人，模拟过程中每个行人都被视为一个圆形，其中成员变量`agentRadius`为其半径。

成员变量`waypoints`表示该agent需要依次到达的目标位置（类似于旅行商），通过方法`addWaypoint`添加新的目标点。

成员变量`p`是`Tvector`类型的变量，用以表示agent的位置（在 version2.4 中，坐标`(x,y,z)`中只用了`(x,y)`）。

成员变量 `scene` 是 `Tscene` 类型的变量。

### Ttree

该类用于优化 agent 之间的距离计算，该类表示的是在scene中的一块区域，该区域主要有4个成员变量记录：`x`、`y`、`w`、`h`，其中`(x, y)` 表示该区域左上角点所在的位置，`w`表示该区域的宽度（x方向上的长度），`h`表示该区域的高度（y方向上的长度），如下图所示，4个区域分别代表了`Ttree`的4个成员变量：`tree1`、`tree2`、`tree3`、`tree4`，这4个变量也是`Ttree`类型。

![](/assets/tree.png)

如果 tree 是叶子节点，则其成员变量 `isleaf` 为True，所有的agent 直接被添加到成员变量 `agents` 下，但是作为叶子节点时，`treeX`都为`None`，但作为叶子节点其所拥有的`agents`长度不能大于8（加快搜索速度），一旦大于8个，就需要使用`addChildren` 函数将叶子节点分裂开，为`treeX`按照上图顺序进行赋值，并将其所拥有的所有`agent`重新分配到`treeX`中。

在 `Ttree` 有成员变量 `depth`，该变量用于记录当前 tree 所在的层数，`scene` 所拥有的 tree 的 `depth` 为 0。


### Tscene

该类用于管理仿真过程中用到的所有元素，整个仿真过程中有且仅有1个Tscene类型的实例。

主要的成员变量介绍：
- `outputwriters`: 用于记录仿真过程的类，记录方式可以是写入文件，也可以通过网络进行发送；
- `obstacles`: 用于记录在场景中所有存在的障碍物；
- `agents`: 用于记录在场景中所有存在的行人；
- `tree`: 用于记录scene的位置与大小，Tscene构造函数中的入参`(left, top, width, height)` 分别对于Ttree的`(x, y, w, h)`；

主要的方法介绍：
- `addAgent(self, a)`: 将本 scene 设置到 agent中，并将本 scene 添加到成员`agents`中；
- `moveAgents` 是 `libpedsim` 最重要的函数，通过该函数实现行人的运动仿真；

