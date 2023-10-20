---
title: SLAM——位姿图优化
date: 2019-02-16 00:00:00
tags: [SLAM]
categories: SLAM
mathjax: true
---    
    

SLAM中另外一个用到的最多的后端优化方法叫做位姿图（Pose Graph）优化。想象一下，对于路标的优化，可能进行几次之后就已经收敛了，这时候每次插入一个帧都再次进行一次BA仿佛有点用力过猛。而且实际中，路标的数量远远大于位姿数量，因此BA在大规模建图时，它的计算量可能会越来越大，使得实时计算变得困难。这里我们介绍的位姿图优化，就是省去了对路标的优化，仅仅调整位姿的一种做法。  

<!--more-->



我们将pose graph的优化转换成图的问题，那么图的节点就是一个个位姿，用$\\xi\_1,…,\\xi\_n$来表示，而边则是两个位姿之间相对运动的估计，这个估计可能来自与特征点法或者是直接法。比如$\\xi\_i,\\xi\_j$之间一个相对运动$\\Delta \\xi\_{ij}$，则：

\\Delta \\xi\_{ij} = \\xi\_i^{-1}\\circ \\xi\_j = \\ln(\\exp((-\\xi\_i)^{\\hat{} }) \\exp (\\xi\_j^{\\hat{} }))^{\\hat{} }

或者按照李群的写法：

T\_{ij} = T\_{i}^{-1} T\_j.

我们知道，实际中上式不会精确成立，因此我们需要设立最小二乘误差，然后讨论关于优化变量的导数。这里我们将上式的$T\_{ij}$移到右侧，为了让其满足误差最小为0的设定，加上一个$\\ln$:

e\_{ij} = \\ln(T\_{ij}^{-1}T\_i^{-1}T\_j)^{\\hat{} } = \\ln(\\exp((-\\xi\_{ij})^{\\hat{} })\\exp((-\\xi\_{i})^{\\hat{} })\\exp(\\xi\_j^{\\hat{} }))^{\\hat {} }

值得注意的是这里的优化变量有两个：$\\xi\_i,\\xi\_j$，因此我们需要求$e\_{ij}$关于这两个变量的导数。按照李代数的求导方式，给$\\xi\_i,\\xi\_j$各一个左扰动$\\delta \\xi\_i,\\delta \\xi\_j$，于是误差变为：

\\hat e\_{ij} = \\ln(T\_{ij}^{-1}T\_i^{-1}\\exp((-\\delta \\xi\_i)^{\\hat{} })\\exp(\\delta\\xi\_j)^{\\hat{} }T\_j)^{\\hat{} }

我们希望将扰动项移到左侧或者右侧，需要利用到一个伴随性质：

\\exp((Ad(T)\\xi)^{\\hat{} }) = T\\exp(\\xi^{\\hat{} })T^{-1}.

稍加改变，得到：

\\exp(\\xi^{\\hat{} })T = T\\exp((Ad(T^{-1})\\xi)^{\\hat{} }).

这说明通过引入一个伴随项，我们能够交换扰动项左右侧的$T$，利用它可以将扰动项移到最右，导出右乘形式的雅科比矩阵：

\\begin{aligned} \\hat e\_{ij} &=\\ln\\left (T\_{ij}^{-1}T\_i^{-1}\\exp((-\\delta\\xi\_i)^{\\hat{} })\\exp(\\delta\\xi\_j^{\\hat{} })T\_j\\right)^{\\hat{} }\\\\ &=\\ln\\left(T\_{ij}^{-1}T\_i^{-1}T\_j \\exp\\left((-Ad(A\_j^{-1})\\delta\\xi\_i)^{\\hat{} }\\right)\\exp\\left((Ad(T\_j^{-1})\\delta \\xi\_j)^{\\hat{} }\\right)\\right)\\\\ &\\approx \\ln(T\_{ij}^{-1}T\_i^{-1}T\_j\[I - (Ad(T\_j^{-1})\\delta \\xi\_i)^{\\hat{} } + (Ad(T\_j^{-1})\\delta\\xi\_j)^{\\hat{} }\])^{\\hat{} }\\\\ &\\approx e\_{ij}+\\frac{\\partial e\_{ij} }{\\partial \\delta \\xi\_i}\\delta \\xi\_i + \\frac{\\partial e\_{ij} }{\\partial \\delta \\xi\_j}\\delta \\xi\_j \\end{aligned}

因此，按照李代数上的求导规则，我们求出了误差关于两个位姿的雅科比矩阵。关于$T\_i$的：

\\frac{\\partial e\_{ij} }{\\partial \\delta \\xi\_i} = -\\mathcal{J}\_r^{-1}(e\_{ij})Ad(T\_j^{-1}).

关于$T\_j$的：

\\frac{\\partial e\_{ij} }{\\partial \\delta \\xi\_i} = \\mathcal{J}\_r^{-1}(e\_{ij})Ad(T\_j^{-1}).

这部分的理解有点困难，可以回顾之前的[李群李代数](https://wlsdzyzl.top/2018/11/09/SLAM%E2%80%94%E2%80%94%E6%9D%8E%E7%BE%A4%E5%92%8C%E6%9D%8E%E4%BB%A3%E6%95%B0/)。之前也说过，由于李群（se(3)）上的雅科比矩阵形式过于复杂，我们通常取近似。如果误差接近于0，可以取近似：

\\mathcal{J}\_r^{-1}(e\_{ij}) \\approx I + \\frac 1 2 \\begin{bmatrix} \\phi \_e ^{\\hat{} } & \\rho \_e^{\\hat{} }\\\\ 0 & \\phi \_e ^{\\hat{} } \\end{bmatrix}.

了解了雅科比计算之后，其余的部分就是普通的图优化了。记$\\varepsilon$为所有边的集合，总体的目标函数为：

\\min\_\\xi \\frac{1}{2} \\sum\_{i,j \\in \\varepsilon} e\_{ij}^T\\Sigma\_{ij}^{-1}e\_{ij}.

我们依然可以利用列文伯格或者高斯牛顿法来解决这个问题。

### [](about:blank#Note "Note")Note

伴随性质：

*   SO(3) R \\exp(p^{\\hat{} })R^T = \\exp((Rp)^{\\hat{} }).
*   SE(3) T\\exp(\\xi^{\\hat{} })T^{-1} = \\exp((Ad(T)\\xi)^{\\hat{} }).其中： Ad(T) = \\begin{bmatrix} R & t^{\\hat{} }R\\\\ 0 & R \\end{bmatrix}.
