---
layout: single
title: Installation
titleShort: Installation
permalink: /tutorials/installation/
layout: single
classes: wide
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials
showTitle: " "

category: "installationTutorial"
date: "2021-02-14"
last_modified_at: "2021-02-14"

pageTag: "tutorials-installation"
excerpt: "Installation guide."
excerptShort: "Installation guide."

header:
  teaser: "/assets/img/favicon.png"
---
{{page.excerpt}}

***

DECENTER installation is straightforward, just follow this steps:
1. **Remove** any old version of DECENTER on your computer.
2. **Download** and **unzip** the <a target= 'blank' href= "/download/">latest release of DECENTER</a> ({% for release in site.data.latest-release %}{{release.tag}}{% break %}{% endfor %}) and place it where you would like to install it.
3. **Open** the directory ({% for release in site.data.latest-release %}{{release.directory}}{% break %}{% endfor %}) in MATLAB and **run**:
~~~ m
addpath(genpath(pwd));
savepath;
~~~
to add and save {% for release in site.data.latest-release %}{{release.directory}}{% break %}{% endfor %} in your MATLAB path. Alternatively, add it manually on **Set Path** under the MATLAB **Home** tab.
4. **Run**:
~~~ m
testDECENTER;
~~~
to test your installation.
