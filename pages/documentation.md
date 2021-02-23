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
{% assign isFirst = 1 %}{% assign sorted_posts = site.pages | sort:"title"%}{% for post in sorted_posts %}{% assign counter = 0 %}{% for tag in post.tags %}{% if tag == "documentation" or tag == "control" or tag == "lti"   %}{% assign counter = counter | plus:1 %}{% endif %}{% if counter == 3 %}{% if isFirst == 0 %}, {% endif %}{% assign isFirst = 0 %}[{{post.title}}]({{post.permalink}}){% endif %}{% endfor %}{% endfor %}

#### LTV systems
{% assign isFirst = 1 %}{% assign sorted_posts = site.pages | sort:"title"%}{% for post in sorted_posts %}{% assign counter = 0 %}{% for tag in post.tags %}{% if tag == "documentation" or tag == "control" or tag == "ltv"   %}{% assign counter = counter | plus:1 %}{% endif %}{% if counter == 3 %}{% if isFirst == 0 %}, {% endif %}{% assign isFirst = 0 %}[{{post.title}}]({{post.permalink}}){% endif %}{% endfor %}{% endfor %}

***
# Estimation
#### LTI systems
{% assign isFirst = 1 %}{% assign sorted_posts = site.pages | sort:"title"%}{% for post in sorted_posts %}{% assign counter = 0 %}{% for tag in post.tags %}{% if tag == "documentation" or tag == "estimation" or tag == "lti"   %}{% assign counter = counter | plus:1 %}{% endif %}{% if counter == 3 %}{% if isFirst == 0 %}, {% endif %}{% assign isFirst = 0 %}[{{post.title}}]({{post.permalink}}){% endif %}{% endfor %}{% endfor %}
#### LTV systems
{% assign isFirst = 1 %}{% assign sorted_posts = site.pages | sort:"title"%}{% for post in sorted_posts %}{% assign counter = 0 %}{% for tag in post.tags %}{% if tag == "documentation" or tag == "estimation" or tag == "ltv"   %}{% assign counter = counter | plus:1 %}{% endif %}{% if counter == 3 %}{% if isFirst == 0 %}, {% endif %}{% assign isFirst = 0 %}[{{post.title}}]({{post.permalink}}){% endif %}{% endfor %}{% endfor %}

***

# Solvers
{% assign isFirst = 1 %}{% assign sorted_posts = site.pages | sort:"title"%}{% for post in sorted_posts %}{% assign counter = 0 %}{% for tag in post.tags %}{% if tag == "documentation" or tag == "solver" %}{% assign counter = counter | plus:1 %}{% endif %}{% if counter == 2 %}{% if isFirst == 0 %}, {% endif %}{% assign isFirst = 0 %}[{{post.title}}]({{post.permalink}}){% endif %}{% endfor %}{% endfor %}

***

# Installation
{% assign isFirst = 1 %}{% assign sorted_posts = site.pages | sort:"title"%}{% for post in sorted_posts %}{% assign counter = 0 %}{% for tag in post.tags %}{% if tag == "documentation" or tag == "installation" %}{% assign counter = counter | plus:1 %}{% endif %}{% if counter == 2 %}{% if isFirst == 0 %}, {% endif %}{% assign isFirst = 0 %}[{{post.title}}]({{post.permalink}}){% endif %}{% endfor %}{% endfor %}
