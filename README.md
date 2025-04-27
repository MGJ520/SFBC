<h1 align="center">
SFBC(SimpleFocBalanceCar)这是一个适合新手学习FOC的平衡车项目(下限低,上限高)
</h1>
<div align="center">
<i>喜欢这个项目吗？请考虑给 Star ⭐️ 以帮助改进！</i>
</div>

---

## 项目简介

SFBC(SimpleFocBalanceCar)是一个基于FOC（Field-Oriented Control）控制的平衡车项目，
旨在帮助新手学习FOC控制、PID算法等核心知识。
硬件设计参考了YeHarold/BalanceFocCar并进行了优化，
基于ESP32S3芯片和Arduino框架下的SimpleFOC库开发而成。

- **平衡车支持以下特性：**
- [x] 平衡保护
- [x] 提起识别
- [x] XBOX手柄遥控控制（蓝牙）
- [x] 网页遥控控制

- **项目特性：**
- [x] 方便二次开发，代码结构清晰，适合大型项目
- [x] 适配多个开发板


---

## 硬件配置

- **电机**：ZD2808
- **主控芯片**：ESP32S3-WROOM-U1-N16R8
- **传感器**：
    - MPU6050（I2C）
    - 编码器：AS5600（I2C）
- **驱动模块**：无刷电机驱动（DRV8300DRGER）
- **电流采样**：低端电流采样（INA181A2IDBVR）

---

## 软件架构

- **开发平台**：PlatformIO
- **嵌入式框架**：Arduino
- **功能扩展**：基于Arduino框架，增加了更多自定义功能

---

## 项目优势

- **适合新手**：从基础的FOC控制到复杂的功能实现，逐步引导学习。
- **功能丰富**：支持多种控制方式，满足不同场景需求。
- **开源友好**：基于开源硬件和软件框架，便于二次开发和扩展。

---

## 联系方式

交流和反馈:

---

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=SmallPond/D-BOT&type=Date)](https://www.star-history.com/#SmallPond/D-BOT&Date)
