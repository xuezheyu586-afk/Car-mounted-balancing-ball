# Car-mounted-balancing-ball
基于德州仪器的MSP-G3507的开发板和stm32f103c8t6

2026 电赛 H 题｜视觉闭环球平衡与运动控制系统

Vision-Based Ball Balancing & Motion Control System
2026 电子设计竞赛 H 题参赛项目

本项目面向 2026 年电子设计竞赛 H 题，实现了一套基于 机器视觉 + STM32 + PID 闭环控制 的球位置稳定与运动控制系统。

视觉端实时识别小球位置，并通过串口向 STM32 发送位置偏差及运动信息；STM32 根据视觉反馈运行 PID 控制算法，驱动执行机构调整球的位置。

在赛前实际调试中，系统能够将小球稳定控制在目标中心附近，稳态位置误差约为 2 cm。


