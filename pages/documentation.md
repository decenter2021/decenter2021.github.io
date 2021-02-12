---
title: Documentation
permalink: /documentation/
layout: single
classes: wide
sidebar:
  title: "Documentation"
  nav: sidebar-documentation
---

# Control
#### LTI systems
{% for command in site.data.documentationControlLTI %}[{{command.name}}]({{command.url}}){% unless command.last %}, {% endunless %}{% endfor %}
#### LTV systems
{% for command in site.data.documentationControlLTV %}[{{command.name}}]({{command.url}}){% unless command.last %}, {% endunless %}{% endfor %}

***
# Estimation
#### LTI systems
{% for command in site.data.documentationEstimationLTI %}[{{command.name}}]({{command.url}}){% unless command.last %}, {% endunless %}{% endfor %}
#### LTV systems
{% for command in site.data.documentationEstimationLTV %}[{{command.name}}]({{command.url}}){% unless command.last %}, {% endunless %}{% endfor %}

***

# Installation
{% for command in site.data.documentationInstallation %}[{{command.name}}]({{command.url}}){% unless command.last %}, {% endunless %}{% endfor %}
