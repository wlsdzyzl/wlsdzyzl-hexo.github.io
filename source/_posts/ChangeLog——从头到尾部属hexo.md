---
title: ChangeLog——从头到尾部属hexo
date: 2023-10-21 06:03:52
tags:
---
文艺复兴，突发奇想想要重新把自己的博客搭起来。仔细研究一下，发现其实变化不大，但还是遇到了一些问题，所以在这里记录一下。
<!--more-->
## 安装node.js
我们知道hexo实际上是基于node.js的，因此我们需要安装node.js。
## 部属hexo
### 安装hexo
有了node.js，hexo的安装是非常简单的，只需要通过下面的命令即可：
### hexo server
### 在github page上发布我们的blog
比较理想的情况是，我们既可以用github保存或者说备份我们的源文件（主要是md文件），也可以用它来发布页面，这样我们就不需要一个额外的服务器了。具体的做法是我们可以创建一个库命名为xxx.github.io，然后用其主分支（main）来备份源文件，再创建一个额外的分支，用来存储hexo生成的页面，并且发布出去。幸运的是，这些功能其实已经部分集成在现有版本的hexo中了。
### 支持latex显示
注：有一些bug会引起hexo的崩溃。
### 设置相册

## 结束
可以开始愉快地记录了。希望这个重启的博客能让生活变得更好一些。