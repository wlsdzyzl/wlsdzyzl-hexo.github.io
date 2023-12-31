---
title: Learning From Data——DQN
date: 2019-01-02 00:42:53
tags: [LFD class, machine learning, reinforcement learning]
categories: 数据学习课程
mathjax: true
---
之前的博客讲了[reinforcement learning](https://wlsdzyzl.github.io/2018/12/28/Learning-From-Data%E2%80%94%E2%80%94Reinforcement-Learning/)，但是上节课讲得更多的像是理论层面的东西，实际操作起来还是一脸懵逼。这次介绍一个非常有名的DQN（Deep Q-network），是神经网络和Q-learning结合起来的一个算法。并且在最后，我们会用它做一个有趣的事情。
<!--more-->
## Q-learning ##

之前我们其实提到了一下Q-learning，最重要的就是动作价值函数Q。它接受两个参数：state，以及action。在Q-learning中，我们会维护一个Q-table。然后根据Q-table来决定下一步的动作怎么选择。

之前的文章中，比较复杂的地方是在一个状态选择一个动作之后，下一个状态是什么还是不确定的，现在我们可以简化这个值，也就是每个状态做一个动作之后得到的下一个状态一定是确定的。这样，问题就会得到简化。
$$
Q(s,a) = R(s,a)+\gamma \cdot \max_{\tilde{a} }\{Q(\tilde{s},\tilde{a})\}
$$

当然，即使是不确定的，学习的道理也是一样的。只不过更复杂了，我们需要去计算期望值。

我们可以看一个简单的例子来理解Q-learning。假如现在有这样的一个房间：

![]()

模型化之后长这个样子：

![]()

这时候，我们可以得到一个R-table，他表示的是每个状态取每个动作之后的奖励是多少，这是我们不可控制的部分。

![]()

同时我们需要维护的就是Q-table。我们不知道Q-table到底该长什么样子。因此最开始全部初始化为0。

![]()

现在我们来尝试更新这个Q-table，Q-learning的学习过程如下：

随机选择一个状态s，假如我们在状态1，查找R表，发现可以走的是3和5.如果采取状态5，那么根据算法的过程：
$$
\begin{aligned}
Q(1,5) &=R(1,5)+\max\{Q(5,1),Q(5,4),Q(5,5)\}
&=100+ \max{0,0,0}\\
&= 100
\end{aligned}
$$

好了，更新Q-table中，$Q(1,5)$为100。

同样的道理，$Q(1,3)$更新为0。

好了这样一个episode就结束了，我们继续这个过程，最后可以得到这个Q-table长成了这个样子：

![]()

通过查找最大的价值，我们来决定下一步怎么走。

当然，上面的例子太简单了，仔细思考的话，我们会发现一些问题。这个Q表是慢慢更新的。有时候，一个Q表的下面几个动作的Q值，并不是最终的结果。但是我们按照最大值，就会永远选择那个最大的，可能实际上它并不是最大的。这可能造成Q-table永远得不到更新。实际上，Q-learning的步骤比上面的更复杂一些，如下：

> Initialize Q-table abitrarily
>
>Repeat (for each episode):
>
>Initialize $s$
>
>Repeat (for each step of episode):
>
>- Choose $a$ from $s$ using policy derived from Q(e.g. $\epsilon$-greedy)
>- Take action $a$, observe $r$, $s'$
>- $Q(s,a):= Q(s,a)+\alpha[r+\gamma \max Q(s',a') - Q(s,a)]$
>- s := s'
>
>until s is terminal

这里，$\alpha$为学习率，$\gamma$为折扣因子。这些一般都是经验值。Q表可以看作是agent的记忆，如果$\alpha$更大，我们更依赖于当前表的值，否则我们更倾向于更新的值。而$\gamma$则是我们是否有长远眼光的一个度量。

Q-learning还是挺强大的，但是之前我们提到了维度诅咒，如果这个维度过大，维护这个表的负担将是不可想象的。因此就有了很多别的方法来计算这个Q值，之前提到了有线性的，特征映射的线性，以及使用神经网络。当然神经网络的效果是好于其他两个的，这就是Deep Q-Network。

[](about:blank#Sarsa "Sarsa")Sarsa
----------------------------------

在介绍Deep Q-Network之前，我们再提一个简单的强化学习算法，叫Sarsa，它和Q-learning算法非常相似。

> Initialize Q-table abitrarily
> 
> Repeat (for each episode):
> 
> Initialize $s$
> 
> Choose $a$ from $s$ using policy derived from Q(e.g. $\\epsilon$-greedy)
> 
> Repeat (for each step of episode):
> 
> *   Take action $a$, observe $r$, $s’$
> *   Choose $a’$ from $s’$ using policy derived from Q(e.g. $\\epsilon$-greedy)
> *   $Q(s,a):= Q(s,a)+\\alpha\[r+\\gamma Q(s’,a’) - Q(s,a)\]$
> *   $s := s’,a := a’$
> 
> until $s$ is terminal

它和Q-learning的区别在于，Q-learning选择了最大的$Q(s’,a’)$，更新Q表以后，下一步并不一定会做$a’$的动作，而sarsa多了一个选择$a’$的步骤,它选的不一定是最大值，并且在下一步一定执行这个动作。

可以看到的sarsa是在线学习，它的探索一定要自己去做，而Q-learning是离线学习，它可以使用别人的经验。而实际上，sarsa也有一些别的扩展算法，如sarsa($\\lambda$)等，在这里就不细谈了。

[](about:blank#Deep-Q-Network "Deep Q-Network")Deep Q-Network
-------------------------------------------------------------

接下来就到了Deep Q-Network的内容了。之前提到了，Q-learning的问题在于，如果高维度连续的情况下，维护一个Q-table是不现实的。一个比较好的做法是把Q表的更新问题变成一个函数拟合的问题。比如输入状态$s$，动作$a$，再加上一个额外的参数$\\theta$，用来得到$Q’(s,a)$。

Q(s,a;\\theta) \\approx Q'(s,a)

而神经网络又可以自动提取复杂特征，所以用它来做这个事情是最合适不过了。

但是首先遇到的一个问题，神经网络需要标签，也就是$y$值，这个$y$值怎么得到呢？

上一篇博客提到了，使用物理法则，或者是模拟器等等,来创造这样的样本,产生经验元组，以供神经网络来训练。我们假设得到的目标样本为$Q\_{\\text{target} }$，则神经网络的Loss-function为：

L(\\theta) = \\mathbb{E}\[Q\_{\\text{target} } - Q(s,a,\\theta)^2\]

根据Q-learning得到：

Q\_{\\text{target} } = r + \\gamma\\max\_{a'}Q(s',a',\\theta)

此外，在神经网络与Q-learning的结合中，还诞生了一些新的概念，如经验池（experience replay）,以及目标网络。

### [](about:blank#experience-replay "experience replay")experience replay

> 经验池的功能主要是解决相关性及非静态分布问题。具体做法是把每个时间步agent与环境交互得到的转移样本$(s\_t,a\_t,r\_t,s\_{t+1})$储存到回放记忆单元，要训练时就随机拿出一些（minibatch）来训练。

在我看来经验池就是存储之前得到的样本。毕竟现在reward也不是之前可以用一张表就能描述的了。

### [](about:blank#TargetNet "TargetNet")TargetNet

后来提出来的改进中，有专门一个网络，用来生成目标Q值。也许你会想说，你怎么知道这个得到的目标就一定是正确的呢？实际上，即使在原来的Q-learning中，Q表也是慢慢更新的。每一次迭代并不一定得到就是正确的值，你用来更新使用的是之前的非正确值，但是多次episode之后，这个值就趋于了稳定，收敛到正确的值，这里也是一样的。是一个互相督促的过程，先用目标网络产生目标值，让神经网络去不断逼近这个目标值，然后用再用神经网络替换目标网络，生成样本，不断这么重复。为什么这样可以？我也不是很清楚这背后的数学关系。

下面是Deep Q-Network的算法：

![](https://evolution-video.oss-cn-beijing.aliyuncs.com/images/dqn1.png)

你可以查看这篇paper来了解关于DQN的更多信息：  
[Human-level control through deep reinforcement learning](https://www.nature.com/articles/nature14236)

最后，我们来实现一下上一篇强化学习博客中提到的cartpole。

cartpole算是python库gym中的一个小项目，除了这个，它还有很多别的小游戏可以去尝试。

我们要做的是让小车尽量保持平衡。而状态是个四维向量$(x,\\theta,\\dot x \\dot \\theta)$,动作只有两种，向左或者向右。

整个想法和Q-learning是非常相似的。我们怎么选择下一个动作？这个依然要用到$\\epsilon$-greedy，以$\\epsilon$的概率输入状态，以得到各个动作的Q值（action-value funtion），然后选取最大的Q值对应的动作作为下一个输入；另外就是随机探索了。

如果小杆子到了，说明游戏结束，这时候得到的直接就是reward，而不会有下一个状态，如果没有倒，则状态转到下一个。当然，这些经历（state,action,next\_state,reward,done）都要放到经验池里，然后我们根据经验池的数据来训练神经网络。

什么时候训练网络是一个经验值。在这里我们假设time%10=0的时候来训练，也就是进行了10的整数倍次数动作之后。什么时候更换目标网络也是一个经验值。

然后我们要面临的是学习的问题。我们通过上面的式子得到y，$\\theta$表示的是MainNet，而$\\hat \\theta$表示是TargetNet，通过他们来得到y值，并用MSE loss以及Adam优化器来完成对参数的更新。需要注意的是机器学习中好的参数非常重要，如果出现错误了首先想到算法是不是写错了，接下来就要考虑调参的问题。

在这个实验中，我们对$\\epsilon$的值也会进行控制，想法是越往后就让它有越大的可能进行贪心选择。

其实这个算法的实现并不算难，但是需要掌握一定的pytorch相关的知识，在这方面我还是比较薄弱的。实际上这个实验是数据学习课程的一个作业。而学长已经编好了大部分的框架，只需要我们实现网络的建立，网络学习过程以及动作的选择。因此大大简化了任务。下面是我的实现。采用4×80×80×2的网络，激活函数为ReLU。

<table><tbody><tr><td class="gutter"><pre><span class="line">1</span><br><span class="line">2</span><br><span class="line">3</span><br><span class="line">4</span><br><span class="line">5</span><br><span class="line">6</span><br><span class="line">7</span><br><span class="line">8</span><br><span class="line">9</span><br><span class="line">10</span><br><span class="line">11</span><br><span class="line">12</span><br><span class="line">13</span><br><span class="line">14</span><br><span class="line">15</span><br><span class="line">16</span><br><span class="line">17</span><br><span class="line">18</span><br><span class="line">19</span><br><span class="line">20</span><br><span class="line">21</span><br><span class="line">22</span><br><span class="line">23</span><br><span class="line">24</span><br><span class="line">25</span><br><span class="line">26</span><br><span class="line">27</span><br><span class="line">28</span><br><span class="line">29</span><br><span class="line">30</span><br><span class="line">31</span><br><span class="line">32</span><br><span class="line">33</span><br><span class="line">34</span><br><span class="line">35</span><br><span class="line">36</span><br><span class="line">37</span><br><span class="line">38</span><br><span class="line">39</span><br><span class="line">40</span><br><span class="line">41</span><br><span class="line">42</span><br><span class="line">43</span><br><span class="line">44</span><br><span class="line">45</span><br><span class="line">46</span><br><span class="line">47</span><br><span class="line">48</span><br><span class="line">49</span><br><span class="line">50</span><br><span class="line">51</span><br><span class="line">52</span><br><span class="line">53</span><br><span class="line">54</span><br><span class="line">55</span><br><span class="line">56</span><br><span class="line">57</span><br><span class="line">58</span><br><span class="line">59</span><br><span class="line">60</span><br><span class="line">61</span><br><span class="line">62</span><br><span class="line">63</span><br><span class="line">64</span><br><span class="line">65</span><br><span class="line">66</span><br><span class="line">67</span><br><span class="line">68</span><br><span class="line">69</span><br><span class="line">70</span><br><span class="line">71</span><br><span class="line">72</span><br><span class="line">73</span><br><span class="line">74</span><br><span class="line">75</span><br><span class="line">76</span><br><span class="line">77</span><br><span class="line">78</span><br><span class="line">79</span><br><span class="line">80</span><br><span class="line">81</span><br><span class="line">82</span><br><span class="line">83</span><br><span class="line">84</span><br><span class="line">85</span><br><span class="line">86</span><br><span class="line">87</span><br><span class="line">88</span><br><span class="line">89</span><br><span class="line">90</span><br><span class="line">91</span><br><span class="line">92</span><br><span class="line">93</span><br><span class="line">94</span><br><span class="line">95</span><br><span class="line">96</span><br><span class="line">97</span><br><span class="line">98</span><br><span class="line">99</span><br><span class="line">100</span><br><span class="line">101</span><br><span class="line">102</span><br><span class="line">103</span><br><span class="line">104</span><br><span class="line">105</span><br><span class="line">106</span><br><span class="line">107</span><br><span class="line">108</span><br><span class="line">109</span><br><span class="line">110</span><br><span class="line">111</span><br><span class="line">112</span><br><span class="line">113</span><br><span class="line">114</span><br><span class="line">115</span><br><span class="line">116</span><br><span class="line">117</span><br><span class="line">118</span><br><span class="line">119</span><br><span class="line">120</span><br><span class="line">121</span><br><span class="line">122</span><br><span class="line">123</span><br><span class="line">124</span><br><span class="line">125</span><br><span class="line">126</span><br><span class="line">127</span><br><span class="line">128</span><br><span class="line">129</span><br><span class="line">130</span><br><span class="line">131</span><br><span class="line">132</span><br><span class="line">133</span><br><span class="line">134</span><br><span class="line">135</span><br><span class="line">136</span><br><span class="line">137</span><br><span class="line">138</span><br><span class="line">139</span><br><span class="line">140</span><br><span class="line">141</span><br><span class="line">142</span><br></pre></td><td class="code"><pre><span class="line"><span class="comment"># -*- coding: utf-8 -*-</span></span><br><span class="line"><span class="string">"""</span></span><br><span class="line"><span class="string">Created on Thu Nov 29 13:05:14 2018</span></span><br><span class="line"><span class="string"></span></span><br><span class="line"><span class="string">@author: xtw+wlsdzyzl</span></span><br><span class="line"><span class="string">"""</span></span><br><span class="line"></span><br><span class="line"><span class="keyword">import</span> random</span><br><span class="line"><span class="keyword">import</span> gym</span><br><span class="line"><span class="keyword">import</span> numpy <span class="keyword">as</span> np</span><br><span class="line"><span class="keyword">import</span> torch</span><br><span class="line"><span class="keyword">import</span> torch.nn <span class="keyword">as</span> nn</span><br><span class="line"><span class="keyword">import</span> torch.optim <span class="keyword">as</span> optim</span><br><span class="line"><span class="keyword">from</span> collections <span class="keyword">import</span> deque</span><br><span class="line">torch.set_default_dtype(torch.float64)</span><br><span class="line">EPISODES = <span class="number">1000</span></span><br><span class="line"><span class="string">'''</span></span><br><span class="line"><span class="string">some api you may use:</span></span><br><span class="line"><span class="string">    torch.from_numpy()</span></span><br><span class="line"><span class="string">    torch.view()</span></span><br><span class="line"><span class="string">    torch.max()</span></span><br><span class="line"><span class="string">    </span></span><br><span class="line"><span class="string">'''</span></span><br><span class="line"></span><br><span class="line"></span><br><span class="line"><span class="class"><span class="keyword">class</span> <span class="title">net</span><span class="params">(nn.Module)</span>:</span>         <span class="comment"># build your net </span></span><br><span class="line">    <span class="function"><span class="keyword">def</span> <span class="title">__init__</span><span class="params">(self, state_size, action_size)</span>:</span></span><br><span class="line">        super(net, self).__init__()</span><br><span class="line">        <span class="string">'''</span></span><br><span class="line"><span class="string">        your code</span></span><br><span class="line"><span class="string">        '''</span></span><br><span class="line">        self.fc1 = nn.Linear(state_size,<span class="number">60</span>)</span><br><span class="line">        self.fc1.weight.data.normal_(<span class="number">0.0</span>,<span class="number">0.1</span>)</span><br><span class="line">        self.fc2 = nn.Linear(<span class="number">60</span>,<span class="number">60</span>)</span><br><span class="line">        self.fc2.weight.data.normal_(<span class="number">0.0</span>,<span class="number">0.1</span>)</span><br><span class="line">        self.out = nn.Linear(<span class="number">60</span>,action_size)</span><br><span class="line">        self.out.weight.data.normal_(<span class="number">0.0</span>,<span class="number">0.1</span>)</span><br><span class="line"></span><br><span class="line">    <span class="function"><span class="keyword">def</span> <span class="title">forward</span><span class="params">(self, x)</span>:</span></span><br><span class="line">        <span class="string">'''</span></span><br><span class="line"><span class="string">        your code</span></span><br><span class="line"><span class="string">        '''</span></span><br><span class="line">        x = self.fc1(x)</span><br><span class="line">        x = nn.functional.relu(x)</span><br><span class="line">        x = self.fc2(x)</span><br><span class="line">        x = nn.functional.relu(x)</span><br><span class="line">        <span class="keyword">return</span> self.out(x)</span><br><span class="line">        </span><br><span class="line">        </span><br><span class="line">        </span><br><span class="line"></span><br><span class="line"></span><br><span class="line"><span class="class"><span class="keyword">class</span> <span class="title">DQNAgent</span>:</span>         <span class="comment"># bulid DQNagent</span></span><br><span class="line">    <span class="function"><span class="keyword">def</span> <span class="title">__init__</span><span class="params">(self, state_size, action_size, q_model, t_model)</span>:</span></span><br><span class="line">        self.state_size = state_size            </span><br><span class="line">        self.action_size = action_size</span><br><span class="line">        self.memory = deque(maxlen=<span class="number">2000</span>)</span><br><span class="line">        self.gamma = <span class="number">0.95</span>    <span class="comment"># discount rate</span></span><br><span class="line">        self.epsilon = <span class="number">1.0</span>  <span class="comment"># exploration rate</span></span><br><span class="line">        self.epsilon_min = <span class="number">0.001</span></span><br><span class="line">        self.epsilon_decay = <span class="number">0.995</span></span><br><span class="line">        self.q_model = q_model        <span class="comment"># model</span></span><br><span class="line">        self.t_model = t_model</span><br><span class="line">        self.criterion = nn.MSELoss() <span class="comment"># define loss</span></span><br><span class="line">        self.optimiser = optim.Adam(self.q_model.parameters(),lr = <span class="number">0.001</span>) <span class="comment">#define optimiser</span></span><br><span class="line"></span><br><span class="line">    </span><br><span class="line"></span><br><span class="line">    <span class="function"><span class="keyword">def</span> <span class="title">remember</span><span class="params">(self, state, action, reward, next_state, done)</span>:</span>     <span class="comment"># save memory</span></span><br><span class="line">        self.memory.append((state, action, reward, next_state, done))</span><br><span class="line"></span><br><span class="line">    <span class="function"><span class="keyword">def</span> <span class="title">act</span><span class="params">(self, state)</span>:</span></span><br><span class="line">        <span class="string">'''</span></span><br><span class="line"><span class="string">        your code</span></span><br><span class="line"><span class="string">        '''</span></span><br><span class="line">        <span class="keyword">if</span> random.random() &lt; self.epsilon:</span><br><span class="line">            <span class="keyword">if</span> random.random()&lt;<span class="number">0.5</span>:</span><br><span class="line">                <span class="keyword">return</span> <span class="number">0</span></span><br><span class="line">            <span class="keyword">else</span>:</span><br><span class="line">                <span class="keyword">return</span> <span class="number">1</span></span><br><span class="line">        <span class="keyword">else</span>:</span><br><span class="line">            <span class="comment"># argmax Q</span></span><br><span class="line">            action_value =  self.q_model.forward(torch.from_numpy(state))</span><br><span class="line"></span><br><span class="line">            <span class="keyword">return</span> int(torch.argmax(action_value))</span><br><span class="line">        <span class="comment"># returns action</span></span><br><span class="line"></span><br><span class="line">    <span class="function"><span class="keyword">def</span> <span class="title">replay</span><span class="params">(self, batch_size)</span>:</span></span><br><span class="line">        minibatch = random.sample(self.memory, batch_size)</span><br><span class="line">        <span class="string">'''</span></span><br><span class="line"><span class="string">        your codes,</span></span><br><span class="line"><span class="string">        use data from memory to train you q_model</span></span><br><span class="line"><span class="string">        '''</span></span><br><span class="line">        <span class="keyword">for</span> state,action,reward,next_state,done <span class="keyword">in</span> minibatch:</span><br><span class="line">            <span class="keyword">if</span> done:</span><br><span class="line">                yj = reward</span><br><span class="line">            <span class="keyword">else</span>:</span><br><span class="line">                yj = reward + self.gamma*torch.max(self.t_model.forward(torch.from_numpy(next_state)))</span><br><span class="line">            loss = self.criterion(yj,self.q_model.forward(torch.from_numpy(state))[action])</span><br><span class="line">            self.optimiser.zero_grad()</span><br><span class="line">            loss.backward()</span><br><span class="line">            self.optimiser.step()</span><br><span class="line">        </span><br><span class="line">        <span class="keyword">if</span> self.epsilon &gt; self.epsilon_min:   <span class="comment"># epsilon decay after each training</span></span><br><span class="line">            self.epsilon *= self.epsilon_decay</span><br><span class="line">        </span><br><span class="line">        </span><br><span class="line">    <span class="function"><span class="keyword">def</span> <span class="title">update_t</span><span class="params">(self)</span>:</span>    <span class="comment"># update t_model weights</span></span><br><span class="line">        torch.save(self.q_model.state_dict(), <span class="string">'params.pkl'</span>)</span><br><span class="line">        self.t_model.load_state_dict(torch.load(<span class="string">'params.pkl'</span>))</span><br><span class="line"></span><br><span class="line"></span><br><span class="line"><span class="keyword">if</span> __name__ == <span class="string">"__main__"</span>:</span><br><span class="line">    env = gym.make(<span class="string">'CartPole-v1'</span>)</span><br><span class="line">    state_size = env.observation_space.shape[<span class="number">0</span>]</span><br><span class="line">    action_size = env.action_space.n</span><br><span class="line">    q_model = net(state_size, action_size)   <span class="comment"># generate nets and DQNagent model</span></span><br><span class="line">    t_model = net(state_size, action_size)</span><br><span class="line">    agent = DQNAgent(state_size, action_size, q_model, t_model)</span><br><span class="line">    done = <span class="keyword">False</span></span><br><span class="line">    replace_target_iter =  <span class="number">25</span>    </span><br><span class="line">    batch_size = <span class="number">100</span></span><br><span class="line"></span><br><span class="line">    <span class="keyword">for</span> e <span class="keyword">in</span> range(EPISODES):</span><br><span class="line">        state = env.reset()</span><br><span class="line">        </span><br><span class="line">        <span class="keyword">if</span> e % replace_target_iter == <span class="number">0</span>:  <span class="comment"># update t_model weights</span></span><br><span class="line">            agent.update_t()</span><br><span class="line">        <span class="keyword">for</span> time <span class="keyword">in</span> range(<span class="number">481</span>):</span><br><span class="line">            env.render()      <span class="comment"># show the amination</span></span><br><span class="line">            action = agent.act(state)     <span class="comment"># chose action</span></span><br><span class="line">            next_state, reward, done, _ = env.step(action) <span class="comment"># Interact with Environment</span></span><br><span class="line">            reward = reward <span class="keyword">if</span> <span class="keyword">not</span> done <span class="keyword">else</span> <span class="number">-10</span>  <span class="comment"># get -10 reward if fail</span></span><br><span class="line">            </span><br><span class="line">            agent.remember(state, action, reward, next_state, done) <span class="comment"># save memory</span></span><br><span class="line">            state = next_state</span><br><span class="line">            <span class="keyword">if</span> done:                </span><br><span class="line">                print(<span class="string">"episode: {}/{}, score: {}, e: {:.2}"</span></span><br><span class="line">                      .format(e, EPISODES, time, agent.epsilon))</span><br><span class="line">                <span class="keyword">break</span></span><br><span class="line">            <span class="keyword">if</span> len(agent.memory) &gt; batch_size <span class="keyword">and</span> time % <span class="number">10</span> == <span class="number">0</span>: <span class="comment"># train q_model</span></span><br><span class="line">                agent.replay(batch_size)</span><br></pre></td></tr></tbody></table>
