# Measure-rotational-speed
使用STM32F1定时器与霍尔传感器对无人机/无人车的主轴转速进行测量
CSDN链接：https://blog.csdn.net/Sky777wdnmd/article/details/123552067

国际标准中，国际标准单位为RPM（Revolutions Per Minute，转/分）。

测量算法

  作者先后采用了两种方法对主轴转速进行了测量，第一种在规定时间（1秒）内对上升沿或下降沿的次数进行统计，在安装了三个霍尔传感器的基础上，每当发生第四次上升沿/下降沿的时候就代表完成了一圈的转动，此时圈数加一（或者用总的上升沿次数除以三得出转动圈数）。用一秒内的转动圈数乘以60即为标准转速RPM，简称为“计数法”；

　　第二种方法不是对跳变沿的个数进行采集，而是对两次跳变沿间的保持时间进行记录，每当判定完成一圈转动时，对时间进行记录，用60除以一圈的时间（换算为秒）即为实时的RPM，如果转速过快，可以对圈数和RPM进行累加，在main函数的循环进行显示时可以取平均值，然后清零，简称为“计时法”。

![image](https://github.com/XiaoWan-zi/Measure-rotational-speed/blob/main/image/%E4%BC%A0%E6%84%9F%E5%99%A8%E7%A4%BA%E6%84%8F%E5%9B%BE.png)

                                                          图1　霍尔测量法示意图

两种方法相比，前者：

　　当转速较慢时，例如RPM低于60，即1秒时间内转动时间小于1圈。此时计算出的转速一直为0；
  
　　该方法计时时间是一秒钟转的圈数，国际标准是以分钟为单位，在转换时需要进行乘60的操作，因此最终结果一定是60的倍数。可以通过拉长统计时间来提高精度，然而由于STM32内部定时器的上限以及实时显示频率的要求，注定表达不够精确；
结合实际实验数据表明，该方法存在较大的误差。

后者：

　　通过统计旋转一圈的时间，或对多圈时间进行记录和累加，不需受到固定时间的拘泥。
  
  
实验部分

　　实验过程：以优利德（UNI-T）的光学转速传感器结合白色布条为标准数据进行实验，可以进行RPM与count的实时显示，实物图如下2所示。

![image](https://github.com/XiaoWan-zi/Measure-rotational-speed/blob/main/image/%E4%BC%98%E5%88%A9%E5%BE%B7%E5%85%89%E5%AD%A6%E8%BD%AC%E9%80%9F%E4%BC%A0%E6%84%9F%E5%99%A8.jpg)

图２　优利德光学转速传感器

　　电路板由一块核心板和靠近主轴的监测板组成。核心板以STMF103作为主控，将监测板上霍尔传感器的输入通过一个电压比较器输出给STM32的引脚完成监测。两者通过排插连接。
  
![image](https://github.com/XiaoWan-zi/Measure-rotational-speed/blob/main/image/%E7%9B%91%E6%B5%8B%E7%94%B5%E8%B7%AF%E6%9D%BF.jpg)

图３　监测电路板

　　以电调为驱动，分别在不同PWM下记录转速数据，PWM为1400电机开始转动，上限为2200，其中的间隔为100。
实验结果如下：

PWM	参考转速（RPM）	霍尔转速（计时法）	霍尔转速（计数法)
1400	816	815	1100
1500	950	945	1120
1600	1090	1086	1200
1700	1225	1223	1260
1800	1350	1353	1400
1900	1500	1490	1500
2000	1630	1625	1500
2100	1765	1762	1620
2200	1853	1855	1860
根据平均误差计算公式

![image](https://github.com/XiaoWan-zi/Measure-rotational-speed/blob/main/image/%E5%B9%B3%E5%9D%87%E8%AF%AF%E5%B7%AE.jpg)

求得计时法的平均转速误差为3.8RPM。在参考数据存在一定误差的情况下，精度较高。

