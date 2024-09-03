# tc264 基于ccu6模块的foc设计

<img src="https://i.imgur.com/expgjKT.jpeg=86x192" width="300px">

- [角度环拟合速度环测试](https://www.bilibili.com/video/BV1TAWcePEq1/)
- [国赛预赛3m8/s演示视频](https://www.bilibili.com/video/BV1g7W3eeEPe/)
------
## 说两句
- 19届国赛一等奖认证方案可行性，实验室速度可达到4m/s不怎么发烫
- 板子就六根线连到fd6288预驱，感觉是个人就会画，不放了（雾）
- 实现很不优雅，用错模块了，以后再搞gtm
- 没开电流闭环，想开的可以看看代码里面夏理的那套思路
- 有uq限制，放开跑不怕烧
- 程序是三天肝完的，请大佬多多指教
- 都到这一步了还不会用就写issue或者直接问我吧

## 致谢
- SimpleFOC
- DengFOC
- STC FOC Lite v2 （感谢王佬的闭环思路，猛猛抄）
- 18届夏理单车越野
- SEEKFREE
- 西凉马戏团
- 智控协会会长大人
