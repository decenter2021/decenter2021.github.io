---
title: Tutorials
permalink: /tutorials/
layout: tags
sidebar:
  title: "Tutorials"
  nav: sidebar-tutorials

showTitle: " "
excerpt: "Detailed simple examples for getting started with DECENTER."
---

{{page.excerpt}}

***

# Control
### LTI systems

### LTV systems


***
# Estimation
### LTI systems

### LTV systems


***
# Installation
{% for page in site.pages %}
{% if page.pageTag == "tutorials-installation" %}
##### [{{page.title}}]({{page.permalink}}) <!-- ATERAR NO GIT -->
<!--##### [{{page.title}}](/{{site.baseurl}}{{page.permalink}})-->
{{page.excerpt}}
{% endif %}
{% endfor %}
