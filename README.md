# RbtEnergyTrjOpt

Windows下一个基于时间放缩的轨迹优化框架，使用QT C++实现

- 可以实现机器人**实时位置与功率采集**
- 可以实现机器人**六轴联动控制与单步控制**
- 可以计算出给定机器人轨迹后的**最优放缩轨迹**（能耗最优）
- 可以**生成轨迹代码**对机器人进行调试

# 编译环境

Qt Creator 6.0以上版本, MSVC2015以上版本

注意：要将Eigen矩阵库放到D盘

# 运行



https://github.com/Jackchen402/RbtEnergyTrjOpt/assets/62838442/f7e3fe28-4a64-46b9-afa8-995676e7b528



# 待优化的地方

- 轨迹优化后的平滑性
- 优化算法辨识的准确性
