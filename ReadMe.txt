本项目的默认算法选择是Fuzzy Adaptive STSMC + APF，如需更改成其他算法，请将~\ros_motion_planning-master\src\core\controller\pid_controller\src此地址中的pid_controller.cpp的内容替换为此文件夹的其他算法的pid_controller.cpp文件的内容，重新编译后使用。
本项目来源于GitHub上的开源项目 https://github.com/ai-winter/ros_motion_planning
本人对此项目做了改进逐步实现了以下核心功能与算法迭代：
1.基础轨迹跟踪复现：基于ros_motion_planning框架，复现了基于A*全局规划与PID局部控制的导航系统，分析了PID算法在理想环境下的性能基准。
2.复杂干扰环境构建：开发了force_disturbance.py干扰注入脚本，能够模拟随机强度的侧向风载及纵向阻力波动，构建了高保真的物理干扰测试环境。
3.鲁棒滑模控制（SMC）：针对PID抗扰性差的问题，设计了基于指数趋近律的滑模控制器。利用滑模变结构控制对匹配干扰的不变性，显著提升了机器人在强干扰下的轨迹保持能力。
4.SMC与人工势场（APF）融合：引入move_obstacle.py控制动态障碍物，并将APF的斥力场模型融入SMC控制律，实现了轨迹跟踪与动态避障的耦合控制，解决了静态路径规划无法应对动态威胁的问题。
5.模糊自适应增强（Fuzzy Adaptive）：针对SMC固有的“颤振”现象以及APF参数固定的局限性，设计了双重模糊逻辑控制器。一方面根据跟踪误差实时调节SMC切换增益，实现“大误差快趋近，小误差低抖动”；另一方面根据障碍物距离与角度动态调整斥力增益，平衡避障安全性与跟踪平滑性。
6.二阶滑模（STSMC）终极优化：为从理论根源上消除抖振并保证有限时间收敛，采用了超螺旋二阶滑模算法，结合模糊自适应机制，构建了Fuzzy Adaptive STSMC+APF控制器，实现了亚毫米级的跟踪精度与极高的动态响应特性。
运行此项目，需先按照原项目中的ReadMe文件，复现完成原项目后，再运行此项目。
