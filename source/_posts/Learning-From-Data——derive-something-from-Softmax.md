---
title: Learning From Data——derive something from Softmax
date: 2018-12-05 13:58:52
tags: [LFD class, machine learning, softmax regression]
mathjax: true
categories: 数据学习课程
---
这周的数据学习课更不知道该起什么题目了。主要是加上一些假设，从Softmax函数开始推导，最后得到一个非常简单的形式，从而大大简化了算法。这次的derivation和上篇讲得东西还是有一些相关的。
<!--more-->
## Review ##

首先回顾一下上篇博客介绍的内容，从HGR maximal correlation开始推导。依然是离散变量$X$与$Y$。不过稍微做点拓展，我们在提取ｘ，ｙ的信息时，把他们映射到一个ｋ维度的向量，也就是：
$$f(x) \rightarrow \mathbb{R}^k,g(x) \rightarrow \mathbb{R}^k.$$

这时候，和之前一样，做一些推导吧。这时候的相关系数变成了相关矩阵：
$$
\max ρ_{XY}=\max 𝔼 p_{XY}[f(x) g(y)^T]
$$

我们的约束变成：
$$
\mathbb{E}[f (x)] = \mathbb{E}[ g(y)] = \mathbf{0}\\
\mathbb{E}[ f^2(x)] = \mathbb{E}[ g^2(y)] = I_{k \times k}
$$

问题描述变为：
$$
\begin{aligned}
\max \Psi^T B \Phi,s.t. &\langle\sqrt{P_X},\Phi\rangle = \langle\sqrt{P_Y},\Psi\rangle = \mathbf{0};\\
&\Phi^T \Phi  =  \Psi^T \Psi  = I_{k \times k}.
\end{aligned}
$$

其中：
$$
\Phi = \begin{bmatrix}
\phi(x_1),\phi(x_2),...,\phi(x_{|X|})
\end{bmatrix}^T_{|X|\times k},\\
\Psi = \begin{bmatrix}
\psi(y_1),\psi(y_2),...,\psi(y_{|Y|})
\end{bmatrix}^T_{|Y|\times k},\\
B_{y,x} =  \frac{p_{XY}(x,y)}{\sqrt{p_X(x)p_Y(y)} },B_{|Y| \times |X|}.
$$

而这时候的$\Phi$与$\Psi$实际上是由Ｂ的第$2，...,k+1$右左特征向量组成:
$$
\Phi = \begin{bmatrix}
\upsilon_2,...,\upsilon_{k+1}
\end{bmatrix}\\
\Psi = \begin{bmatrix}
\mu_2,...,\mu_{k+1}
\end{bmatrix}
$$

$f(x)=\frac{\phi(x)}{\sqrt{p_X(x)} } ,g(y) = \frac{\psi(y)}{\sqrt{p_Y(y)} }$.

## HGR & Softmax ##

假设$X,Y$是离散的，并且几乎独立(weakly dependent),也就是$p_{XY}(xy) - p_{X}(x)p_{Y}(y)$非常小。

还记得softmax function:
$$
Q_{Y|X}(y|X) = \frac{e^{X^TW_y + b_y)} }{\sum_{y' \in \mathcal{Y} } e^{X^TW_{y'}+b_{y'} } }
$$

在这里，我们把$X,Y$再次进行信息提取，分布为$f(X),g(Y)$. 由于$W_y$与Y值相关，我们可以将$W_y$看作是g(y)。因此写成更通用的形式：
$$
\begin{aligned}
Q_{Y|X}(y|x) &= \frac{e^{f^T(x)g(y) + b(y)} }{\sum_{y' \in \mathcal{Y} }e^{f^T(x)g(y')+b(y')} }\\
&= \frac{p_Y(y)e^{f^T(x)g(y) + b(y) - \log p_Y(y)} }{\sum_{y' \in \mathcal{Y} }p_Y(y')e^{f^T(x)g(y')+b(y') - \log p_Y(y)} }
\end{aligned}
$$

现在我们定义：$d(y) \triangleq b(y)-\log p_Y(y)$，则：
$$
Q_{Y|X}(y|x) = \frac{p_Y(y)e^{f^T(x)g(y) + d(y)} }{\sum_{y' \in \mathcal{Y} } p_Y(y')e^{f^T(x)g(y')+d(y')} }
$$
可以看到，如果$f = g = d =  0$，$Q_{Y|X}(y|x) = p_Y(y)$.

由于我们的假设可以知道,$p_Y(y) \approx Q_{Y|X}(y|x)$，则$f^T(x)g(y)+ d(y) \approx 0$,根据泰勒展开:
$$
\begin{align}
e^{f^T(x)g(y) + d(y)} \approx 1 + f^T(x)g(y)+ d(y)
\end{align}
$$

而:
$$
\begin{aligned}
\sum_{y' \in \mathcal{Y} } p_Y(y')e^{f^T(x)g(y')+d(y')} &\approx \sum_{y' \in \mathcal{Y} } p_Y(y')[ 1+f^T(x)g(y')+d(y')]\\
&= 1 + f^T(x)\sum_{y' \in \mathcal Y} p_Y(y')g(y') + \sum_{y' \in \mathcal Y}p_Y(y')d(y')\\
&= 1 +f^T(x)\mathbb{E}_Y[g(Y) ] + \mathbb{E}_Y[d(Y) ] 
\end{aligned}
$$
而由泰勒展开$\frac{1}{1+x} \approx  1-x$得到：
$$
\begin{align}
\frac{1}{\sum_{ y'\in \mathcal{Y} } p_Y(y' )e^{f^T(x)g(y' )+d(y' )} } \approx 1 -f^T(x)\mathbb{E}_Y[g(Y )] -\mathbb{E}_Y[d(Y )]
\end{align}
$$
结合上面的(1),(2)，我们得到：
$$
\begin{aligned}
Q_{Y|X}(y|x) &= \frac{p_Y(y)e^{f^T(x)g(y) + d(y)} }{\sum_{y' \in \mathcal{Y} } p_Y(y')e^{f^T(x)g(y')+d(y')} }\\
&\approx p_Y(y)(1 + f^T(x)g(y)+ d(y) )( 1 -f^T(x)\mathbb{E}_Y[g(Y )] -\mathbb{E}_Y[d(Y )])\\
& \approx p_Y(y)[1 + f^T(x)g(y) +d(y) - f^T(x)\mathbb{E}_Y[g(Y) ] - \mathbb{E}_Y[d(Y) ]]\\
&= p_Y(y)[1+f^T(x)(g(y)-\mathbb{E}_Y[g(Y) ]) + (d(y) - \mathbb{E}_Y[d(Y) ])]
\end{aligned}
$$

现在我们令$\tilde{g}(y) = g(y) - \mathbb{E}_Y[g(Y)]，s.t. \mathbb{E}_Y[\tilde {g}(Y)] = 0$.得到：
$$
Q_{Y|X}(y|x) = p_Y(y)[1+ f^T(x)\tilde g (y) + \tilde d (y)]
$$

现在我们利用这个式子构建$empirical risk$，实际上也就是$-\frac{1}{n} \sum_{i=1}^n \log Q_{Y|X}(y_i|x_i)$.最小化经验风险(empirical risk)实际上也就是最大化$\mathbb{E}_{p_{XY} } [ Q_{Y|X}(y|x)]$，也是极大似然估计。
$$
\begin{aligned}
\log Q_{Y|X}(y|x) &= \log p_Y(y) + \log (1+ f^T(x)\tilde{g}(y) + \tilde{d} (y))\\
& \approx \log p_Y(y) + f^T(x)\tilde{g}(y) + \tilde{d}(y) -  \frac{1}{2} [(f^T(x)\tilde{g}(y))^2 + \tilde{d^2}(y) + 2 f^T(x)\tilde{g}(y)\tilde{d}(y)]
\end{aligned}
$$
上述过程用到了泰勒展开：$\log(1+x) \approx x - \frac{x^2}{2}$.

$\mathbb{E}[\log Q_{Y|X}(Y|X)] = \mathbb{E}[\log p_Y(Y)] + \mathbb{E}[f^T(X)\tilde{g}(Y) ] + \mathbb{E}[\tilde{d}(Y) ] - \mathbb{E}[\frac{1}{2} [(f^T(X)\tilde{g}(Y))^2 + \tilde{d^2}(Y) + 2 f^T(X)\tilde{g}(Y)\tilde{d}(Y)]] $

现在，我们来说明一些必要的东西：由假设得到$p_{XY}(x,y) - p_X(x)p_Y(y) = \epsilon \cdot \square = o(\epsilon)$，$o(\epsilon)$表示$\epsilon$的无穷小量（这么说其实不准确，因为我们最后要最大化这种无穷小量，显然不合理，可以当作为衡量有多小的量级）。

因为$$Q_{Y|X}(y|x) = P_Y(y)[1+f^T(x)\tilde{g}(y) + \tilde{d}(y)] \approx p_{Y},$$
同理可以得到$f^T(x)\tilde g (y)  = o(\epsilon),\tilde d (y) = o(\epsilon)$，我们假设对所有的$f,\tilde g,\tilde d$都进行了normalize，也就是$\mathbb{E}f = \mathbb{E}\tilde{g} = \mathbb{E}\tilde{d} = 0$,则：
* $$
\begin{aligned}
\mathbb{E}[f^T(X)\tilde g(Y) ] &= \sum_{x,y} p_{XY}(x,y)f^T(x)\tilde g(y)\\
&= \sum_{x,y}(p_{X}(x)p_{Y}(y)f^T(x)\tilde g(y) + o(\epsilon)f^T(x)\tilde g(y))\\
&= \sum_{x}p_{X}(x)f^T(x) \sum_{y}p_{Y}(y)\tilde g(y) + o(\epsilon^2)\\
&= o(\epsilon^2).
\end{aligned}
$$
* $$
\begin{aligned}
\mathbb{E}[(f^T(x)g(y) )^2] &= \sum_{x,y}p_{X}(x)p_{Y}(y)(f^T(x)\tilde g(y))^2 + \sum_{x,y} o(\epsilon)(f^T(x)\tilde g(y))^2\\
&= \sum_{x,y}p_{XY}(x,y)(f^T(x)\tilde g(y))^2 + o(\epsilon^3)
\end{aligned}
$$
* $$
\begin{aligned}
\mathbb{E}[f^T(x) \tilde{g}(y)\tilde{d}(y)] &= \sum_{x,y}p_{X}(x)p_Y(y) f^T(x)\tilde g (y) \tilde d (y) + o(\epsilon)\sum_{x,y}f^T(x)\tilde g (y) \tilde d (y)\\
&= \sum_{x}p_{X}(x)f^T(x) \sum_{y}p_{Y}(y)\tilde g(y) \tilde d(y) + o(\epsilon^3)\\
&=  o(\epsilon^3)
\end{aligned}
$$

而我们知道$o(\epsilon^3)$在$o(\epsilon^2)$之前是可以被忽略的。因此最终：
$$
\mathbb{E}[\log Q_{Y|X}] = \mathbb{E}[\log p_Y(Y)] + \mathbb{E}[f^T(X) \tilde g (Y)] - \frac 1 2 \mathbb{E}[(f^T(X) \tilde g (Y))^2] - \frac{1}{2} \mathbb{E}[\tilde{d^2} (y)]
$$

上式中，第一项为常数，最后一项为非负值，且与前面几项没有约束关系，因此为了最大化上式只需简单令$\frac{1}{2} \mathbb{E}[\tilde{d^2} (y)]=0$，因此最终我们要做的是：
$$
\max_{f,\tilde g} (\underbrace{\mathbb{E}[f^T(x)\tilde g(y)] - \frac{1}{2}\mathbb{E} [(f^T(x)\tilde g(y))^2]}_{\Delta}) 
$$

如果我们将$ \mathbb{E}[f^T(x)\tilde g(y)] $对$f(x)$求导，可以得到：
$$
\frac{\partial \Delta}{\partial f(x)} = 0\\
f(x) = \land ^{-1}_{\tilde g (Y)} \mathbb{E}[\tilde g(Y)|X = x]
$$

其中$\land ^{-1}_{\tilde g (Y)}=(\mathbb{E}_{p_Y}[\tilde g(Y){\tilde g}^T(Y)])^{-1}$，也就是我们得到了最佳的$f,\tilde g$.

同理我们也得到：
$$
\tilde {g ^*}(y) = \land ^{-1}_{f(X)} \mathbb{E}[f(X)|Y = y]
$$

也就是如果我们向softmax函数中喂入$f(x)$（形式固定）,那么softmax尽量在学的东西，也就是$W$实际上是$g^*$，当然不一定能成功学到这样的形式。

同样的，利用神经网络进行softmax可以看作是在寻找Ｘ的特征，它找到的最佳形式应该是$f*$.

简直是头大。实际上我不能保证这篇博客的正确性。