---
title: SLAM——视觉里程计（二）对极几何
date: 2019-01-09 00:00:00
tags: [SLAM,geometry,computer vision]
categories: SLAM
mathjax: true
---        

上次介绍了特征，现在我们已经得到了特征匹配后的点对，现在我们需要用这些东西来估计相机运动。由于相机原理不同，估计的方法也不同。现在我们考虑的是单目相机，使用的方法为对极几何。  


<!--more-->


### [](about:blank#%E5%AF%B9%E6%9E%81%E7%BA%A6%E6%9D%9F "对极约束")对极约束

如下图：

![](https://evolution-video.oss-cn-beijing.aliyuncs.com/images/djjh1.png)

我们希望求得是两帧图像$I_1,I_2$之间的运动，设第一帧到第二帧的旋转为$R$，平移为$t$向量，两个相机的位置分别为$O_1,O_2$。假如$p_1,p_2$为一对匹配正确的特征点，那么他们是同一个空间点$P$在两个平面上的投影。$O_1p_1$与$O_2p_2$会相交于点$p$。这时候点$Q_1,Q_2,P$三个点可以确定一个平面，称为极平面（epipolar plane）。$O_1O_2$连线与像平面$I_1,I_2$的交点$e_1,e_2$为极点（epipoles），$O_1O_2$为基线（baseline）。

我们可以看到，如果只看第一帧$I_1$，射线$O_1p_1$上是任何一点都是这个像素点可能对应的空间位置，而$e_2p_2$是这个射线则是对应的可能的投影位置。因此，只有通过正确的特征匹配才能保证这个$p$点的确定位置，否则的话我们需要在这个极线上搜索。

现在我们从代数角度来看一下这其中的几何关系。假设在第一帧的相机坐标系下，P的坐标为：
$$
P_1 = [X_1,Y_1,Z_1]^T, P_2 = [X_2,Y_2,Z_2]^T
$$
上式中：
$$
P_2 = RP_1 + t
$$
根据之前介绍的针孔相机模型，我们可以得到两个像素点$p_1,p_2$的位置:
$$
p_1 = \frac 1 {Z_1}KP_1, p_2 = \frac 1 {Z_2 }KP_2 = \frac{1}{Z_2}(RP_1+t).
$$
这里$K$为相机内参矩阵。现在取$x_1 = K^{-1}p_1, x_2 = K^{-1}p_2$，则$x_1,x_2$分别为在空间的归一化坐标，即：
$$
x_1 = [\frac{X_1}{Z_1},\frac{Y_1}{Z_1},1],x_2 = [\frac{X_2}{Z_2},\frac{Y_2}{Z_2},1]
$$
我们知道：
$$
P_2 = RP_1 + t
$$
则：
$$
Z_2x_2 = RZ_1 x_1 + t
$$
在上式两侧左乘$t^{\hat {} }$，得到：
$$
Z_2t^{\hat{} } x_2 = Z_1t^{\hat{} } Rx_1
$$
如果再将两侧左乘$x_2^T$：
$$
Z_2x_2^T t^{\hat{} } x_2 =Z_1 x_2^T t^{\hat{} } R x_1
$$
由于$t^{\hat{} } x_2$与$t,x_2$都是垂直的，因此得到：
$$
Z_1x_2^T t^{\hat{} } R x_1 = 0.
$$
即：
$$
x_2^T t^{\hat{} } R x_1 = 0.
$$
重新代回$p_1,p_2$，得到：
$$
p_2^T(K^{-1})^Tt^{\hat{} }RK^{-1}p_1 = 0
$$
上式就是对极约束。对极约束中包含了平移和旋转，我们把中间部分记为两个矩阵：基础矩阵（Fundamental Matrix）F与本质矩阵（Essential Matrix）E，据说可以进一步简化对极约束：
$$
x_2^T E x_1 = p_2^T F p_1 = 0
$$
上式中：$E = t^{\hat{} }R,F = (K^{-1})^T E K^{-1}$。

对极约束简洁地给出了两个匹配点的空间位置关系，于是相机位姿的估计问题变成了下面两步：

1.  根据配对点像素求出E或者F
2.  根据E或者F求出R和t。

由于E和F只相差一个相机内参，而内参一般来说是已知的。实践中往往使用形式更简单的E。

### [](about:blank#%E6%9C%AC%E8%B4%A8%E7%9F%A9%E9%98%B5E "本质矩阵E")本质矩阵E

根据定义$E= t^{\hat{} }R$，它是一个$3\times 3$矩阵，有9个元素。它有下面几个性质：

*   本质矩阵是由对极约束定义的，由于对极约束是等式为0的约束，因此对$E$乘以任意非0常数，对极约束依然满足，这说明$E$在不同尺度下是等价的。
*   根据定义可以证明，$E$的奇异值必定是$[\sigma,\sigma,0]^T$的形式，这是本质矩阵的内在性质。
*   由于$R$和$t$各有3个自由度，故$t^{\hat{} }R$共有6个自由度，而由于尺度等价性，$E$实际上有5个自由度。
*   E的秩为2

由于$E$的自由度有5个，可以用5个匹配点就可以求出$E$，但是$E$的内在性质是一种非线性性质，在求解时候会带来麻烦。可以只考虑它的制度等价性，使用8对点来估计$E$，这就是8点算法。它只使用了$E$的线性性质，因此求解更加容易。通常来说，我们匹配点的个数不会少于8个。

#### [](about:blank#8%E7%82%B9%E7%AE%97%E6%B3%95 "8点算法")8点算法

考虑到一对匹配点，它们的归一化坐标分别为$x_1 = [u_1,v_1,1]^T,x_2 = [u_2,v_2,1]^T$。根据对极约束有：
$$
\begin{pmatrix} u_1&v_1&1 \end{pmatrix}\begin{pmatrix} e_1&e_2&e_3\\ e_4&e_5&e_6\\ e_7&e_8&e_9 \end{pmatrix}\begin{pmatrix} u_2\\ v_2\\ 1 \end{pmatrix}
$$
我们把矩阵$E$展开写成向量形式：
$$
e = [e_1,e_2,e_3,e_4,e_5,e_6,e_7,e_8,e_9]^T
$$
那么对极约束可以写成与$e$有关的线性形式：
$$
[u_1u_2,u_1v_2,u_1,v_1u_2,v_1v_2,v_1,u_2,v_2,1]\cdot e = 0
$$
同理，对其他的7对点也有这样类似的形式，于是我们就有一个线性方程组：
$$
\begin{pmatrix} u^1_1u^1_2&u^1_1v^1_2&u^1_1&v^1_1u^1_2&v^1_1v^1_2&v^1_1&u^1_2&v^1_2&1\\ u^2_1u^2_2&u^2_1v^2_2&u^2_1&v^2_1u^2_2&v^2_1v^2_2&v^2_1&u^2_2&v^2_2&1\\ \vdots&\vdots&\vdots&\vdots&\vdots&\vdots&\vdots&\vdots&\vdots\\ u^8_1u^8_2&u^8_1v^8_2&u^8_1&v^8_1u^8_2&v^8_1v^8_2&v^8_1&u^8_2&v^8_2&1 \end{pmatrix} \begin{pmatrix} e_1\\ e_2\\ e_3\\ e_4\\ e_5\\ e_6\\ e_7\\ e_8\\ e_9 \end{pmatrix} = 0
$$
如果这个系数矩阵是满秩的（秩为8），那么它的零空间维数为1,也就是e构成一条线，这和$e$的尺度等价性是一致的（不明白这段话在说什么）。总之如果是满秩的，我们可以通过上面的方程组解出$e$的值。它们和e共同构成了一组9维空间的基。

有了e的值，我们就得到了本质矩阵$E$。接下来的问题是如何根据$E$求得$R,t(E = t^{\hat {} }R)$。

这时候我们需要用到奇异值分解（SVD）。假如：
$$
E = U\Sigma V^T,
$$
由于$E$的内在性质，我们知道$\Sigma = \text{diag}(\sigma,\sigma,0)$。在SVD中，对于任何一个$E$，存在有两个可能的$t,R$与他们对应：
$$
t^{\hat{} }_1 = UR_Z(\frac{\pi}{2})\Sigma U^T, R_1 = UR_Z^T(\frac{\pi}{2})V^T\\ t^{\hat{} }_2 = UR_Z(-\frac{\pi}{2})\Sigma U^T, R_2 = UR_Z^T(-\frac{\pi}{2})V^T
$$
上式中$R_Z(\frac{\pi}{2})$表示沿Z轴旋转90度得到的旋转矩阵。为什么要这样算？首先我们知道，得到的$t^{\hat{} }$必须得是一个反对称矩阵，同时得到的$R$要是一个旋转矩阵，因此这说明求解$R,t$时候是有约束的。而$R_Z(\frac{\pi}{2})$得到的解则是满足上面的条件的。同时由于$-E$与$E$是等价的，因此对$t$取负号,也是有一样的结果。所以就会得到4个解。想看更多的内容，可以查看wiki：[determine R and t](https://en.wikipedia.org/wiki/Essential_matrix#Determining_R_and_t_from_E)。下图形象得展示了分解本质矩阵得到的4个解：

![](https://evolution-video.oss-cn-beijing.aliyuncs.com/images/djjh3.jpg)

可以从上图看出只有第一个是正确的，它的深度都是正值，因此排除其他3个解并不算困难。

由于$E$的内在性质，还有一种算法是五点算法，不过这个做法会更加复杂。由于我们每次匹配的个数一般都会成百上千，因此他们的区别不是很大。对于五点算法有兴趣的可以看着篇论文：[五点算法](http://www.ee.oulu.fi/research/imag/courses/Sturm/nister04.pdf)。

最后还有一个问题，根据线性方程组解得的$E$可能不满足$E$的性质，因为现实世界总是充满噪声的。这时候在做SVD时，我们会刻意将$\Sigma$矩阵调整成上面的样子，如$\Sigma=\text{diag}(\sigma_1,\sigma_2,\sigma_3)$，设$\sigma_1>\sigma_2>\sigma_3$，那么我们取：
$$
E = U\text{diag}(\frac{\sigma_1+\sigma_2}{2},\frac{\sigma_1+\sigma_2}{2},0)V^T.
$$
其实更简单的做法是取$\Sigma = (1,1,0)$，由于$E$具有出度等价性，这样做也是没什么错误的。

### [](about:blank#%E5%8D%95%E5%BA%94%E7%9F%A9%E9%98%B5%EF%BC%88Homography-Matrix%EF%BC%89 "单应矩阵（Homography Matrix）")单应矩阵（Homography Matrix）

除了基本矩阵与本质矩阵，还有一个单应矩阵$H$需要我们注意。若场景中的特征点都落在同一平面上，则可以通过单应性来进行运动估计。

单应矩阵通常描述处于共同平面上的一些点在两张图像之间的变换关系。考虑在图像$I_1,I_2$有一对匹配好的特征点$p_1$和$p_2$。这些特征点落在平面$P$上，设这个平面满足：
$$
n^TP + d = 0.
$$
稍加整理可以得到：
$$
-\frac{n^TP}{d} = 1
$$
我们知道：
$$
\begin{aligned} p_2 &= \frac{1}{Z_2}K(RP+t)\\ &= \frac{1}{Z_2}K(RP + t \cdot\left(-\frac{n^TP}{d})\right )\\ &= \frac{1}{Z_2}K\left(R - \frac{tn^T}{d}\right)P\\ &=\frac{Z_1}{Z_2}K\left(R - \frac{tn^T}{d}\right)K^{-1}p_1\\ &= \frac{Z_1}{Z_2}Hp_1 \end{aligned}
$$
当然，这个$\frac{Z_1}{Z_2}$我们也是不知道的。我们可以忽略它到$H$矩阵求解中。简单得到$p_2 = Hp_2$  
于是，我们得到了一个直接描述图像坐标$p_1,p_2$的变换$H$。它的定义与旋转，平移以及平面的参数有关。与基础矩阵$F$类似，单应矩阵$H$也是一个$3\times 3$的矩阵。根据匹配点，我们可以得到：
$$
\begin{pmatrix} u_2\\ v_2\\ 1 \end{pmatrix}= \begin{pmatrix} h_1&h_2&h_3\\ h_4&h_5&h_6\\ h_7&h_8&h_9 \end{pmatrix} \begin{pmatrix} u_1\\ v_1\\ 1 \end{pmatrix}
$$
对上式右侧侧展开后，一般我们是无法得到准确地等于1的，因此将第三项变为1,得到：
$$
u_2 = \frac{h_1u_1 +h_2v_1 + h_3}{h_7u_1+h_8v_1+h_9}\\ v_2 = \frac{h_4u_1+h_5v_1+h_6}{h_7u_1+h_8v_1+h_9}
$$
实际中我们会对$h_9$进行归一化，整理可以得到：
$$
h_1u_1 +h_2v_1+h_3-h_7u_1u_2-h_8v_1v_2 = u_2\\ h_4u_1+h_5v_1+h_6-h_7u_1v_2-h_8v_1v_2 = v_2
$$
这样一组匹配点对就可以构造出来两项约束，而$H$的自由度为8,我们可以通过4对匹配特征点算出（不能有3点共线的情况）。

![](https://evolution-video.oss-cn-beijing.aliyuncs.com/images/djjh2.jpg)

下面的做法把$H$矩阵看成向量，通过解该向量的线性方程来恢复$H$，又称直接线性变换法。求出单应矩阵以后需要进行分解才能得到对应的旋转矩阵$R$与平移矩阵$t$。分解的方法包括数值法与解析法。单应矩阵的分解同样得到四组解，依照深度信息可以排除两组，剩下的两组需要用先验信息来完成。如，如果场景平面与相机平面平行，法向来的理论值为$\mathbf{1}^T$。

### [](about:blank#%E4%B8%89%E8%A7%92%E6%B5%8B%E9%87%8F "三角测量")三角测量

我们可以根据$R,t$来得到深度信息。首先，由之前的定义可以得到：
$$
Z_1x_1 = Z_2Rx_2 +t
$$
上式左侧乘以$x_1^{\hat{} }$，得到：
$$
Z_1x_1^{\hat{} } = Z_2x_1^{\hat{} }Rx_2 +x_1^{\hat{} }t = 0
$$
因此我们可以解出来$Z_2$与$Z_1$。当然由于尺度不确定性，这个求得只是三维坐标的相对位置。我们并无法得知它们真实的距离。

### [](about:blank#Note "Note")Note

*   为什么需要单应矩阵？当特征点共面或者相机发生纯旋转或者平移的时候，基础矩阵的自由度下降，此时继续使用8点算法，对于的自由度主要由噪声决定，这就出现了退化。为了避免这种现象，通常会估计基础矩阵$F$和单应矩阵$H$，选择重投误差较小的哪个作为最终的运动估计矩阵。
    
*   单目相机具有尺度不确定性，我们求解出来的值并不能确定它的单位是多少，进行同比例的放大缩小结果都是一样的。同时，它需要进行初始化，再初始化时，不能进行纯旋转，如果平移为0,则无法根据8点算法来求解出旋转。之后的求解可以使用3D-2D来计算相机运动。
    
*   在使用8点算法时候，由于噪声存在，往往构成超定问题。这是很可以通过最小二乘法来优化（SVD可以用来解最小二乘问题），当然误匹配情况下，也可以使用RANSAC来解决这个问题，实际上后者用得更多一点。
    
*   在使用3角测量时，由于像素的不确定性（可以认为是相机分辨率不够对定位的误差），会造成深度的不确定性。想要增大深度估计的精度，我们需要增加相机分辨率，但这样会导致计算量以及需要的内存过大，或者是增加平移的距离。而平移距离加大会导致外观变化较大，使得特征提取和匹配变得困难。因此这就导致了三角化的矛盾：平移过小，深度精度不够，平移过大，匹配失败。如下图：
    

![](https://evolution-video.oss-cn-beijing.aliyuncs.com/images/djjh4.jpg)
