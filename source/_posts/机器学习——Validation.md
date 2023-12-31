---
title: 机器学习——Validation
date: 2018-09-28 20:35:19
tags: [machine learning,validation]
categories: 机器学习
mathjax: true
---
上次regularization最后留下了一个问题：$\lambda$的选择。其实仔细想想，从学习机器学习到现在，我们面临的选择，可不止一个$\lambda$.
<!--more-->

模型的选择（PLA，POKCET，Linear Regression，Logistic Regression），特征转换的方法（用什么样的多项式转换），Regularizer的选择等等，这些组合起来足够让人头大。而实际上，也没有一种永远都表现得很好的组合，对于不同的问题需要不同的方法，也就是做选择是必须的。

为了简化问题，我们就仅以不同的假说的选择为例。其他的选择也与这个类似。

假设我们目前有2个$H$，一个$H_2$,另一个$H_{10}$，应该选择哪一个？

一个简单的想法，是利用$E_{in}$去判断。但是这个想法太naive了。我们之前讲过过拟合了，如果用$E_{in}$去判断，那就不用想了，因为$H_{10}$一定比$H_2$要好，而且如果两个模型一个加了$regularization$，它的表现一定不如另一个.而且如果两个$H$没有交集，从两个$E_{in}$中选择一个好的，那么实际上是在两个$H$的并集中选择，这也就意味着增大了代价，更容易得到不好的$E_{out}$。

另一种简单的想法，用测试数据来判断。这是一个很好的办法。我们知道$E_{test}$与$E_{out}$是满足霍夫丁不等式的，因此可以得到：
$$
E_{out} \leq E_{test}+O(\sqrt {\frac{\log M}{N_{test} } })
$$

所以用测试集来从模型中选择一个最好的是可行的。但是测试集从哪里来？

一般来说，测试集是锁在老板的柜子中。测试集相当于考试试卷，用来评判最终的分数.我们无法得到测试集，这就像考试前你想让自己做到最好，你没法用考试的卷子来测试自己，这叫作弊。

但是我们可以自己测验自己。这就是validation。

从给到的训练集当中，我们随机挑出一部分（保证iid），用来当作val集，其余部分用来训练模型。然后通过val集来选出表现最好的g'.为什么不是g？因为毕竟它的训练集相对于之前要少了很多，所以加个标识以区分。

一般来说，得到g'以后，也就是选出了我们想要的那个$H$，然后我们要做的就是将验证集再次融合回去，用这个整体的训练集在该假说上训练。毕竟某种程度上来说，N越大，得到的模型是越好的。而且
$$
E_{out}(g_{H_{chosen} }) < E_{out}(g'_{H_{chosen} })
$$
上式是实际中的一个很常见的式子，但理论上要这么保证还需要一定的限制条件。

假设我们从N个训练样本中挑出K个来做验证集，当然这个K的大小是会影响结果的。如果K很大，那么很开心，$E_{val}$与$E_{out}$更有可能很接近，这对于选择g'来说是很好的，但是K过大意味着留下来的训练样本过少，g'与g差别很大，可能导致我们无法找到应该选择的那个$H$；另一方面，K很小，g与g'相差很小，但是$E_{val}$与$E_{out}$可能实际上差的很远，也不能得到好的结果。因此这又是一个难题。
一般来说有下图：
![](https://evolution-video.oss-cn-beijing.aliyuncs.com/images/%7ELN%7EMS5IB6G61%40DWBDM51OE.png)
可以看到如果K过大，导致训练g'的训练集很小，使得它的学习效果很差，甚至不如不用验证的情况。

为了解决这个问题，引入一种新的工具：交叉验证。

首先我们考虑一种很极端的情况：K=1.每次留出一个来做验证，对于单个样本来说它当然无法代表$E_{out}$.但是如果我们对这个过程进行N次，所有的样本都曾做过验证集，最后求出来$E_{val}$的平均值，可以证明它就能代表g'的$E_{out}$.上面的办法，叫做leave-one-out cross validation.假设它得到的错误我们称为$E_{looc}$，则有下面的证明：
![](https://evolution-video.oss-cn-beijing.aliyuncs.com/images/VQVY7%5B%60%5BQA%40AT%7BQ_S24LDQ4.png)
(上面的证明我是看不大懂的)

这似乎是个很好的方法，但是它有个很致命的缺陷：计算量（N）倍的力气去计算g'。

因此，实际中我们很少用leave one out cross，而使用V-Fold Cross。将样本分为10（或者其他数）份，然后留一份作为val集，像上面一样交叉验证。这样需要的力气就是10倍，可以接受，而且能得到比非交叉验证更好的结果。

对于其他情况的选择，也可以用这样的办法，因为我们最终目的是得到尽量好的$E_{out}$。

最后，validation依然是为了做选择，因此它的结果依然是比较optimistic，算法最终的衡量还是要通过测试集，而不是将最好的validation结果作为衡量标准，这是自欺欺人的表现。