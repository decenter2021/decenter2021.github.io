---
title: Tutorials
permalink: /tutorials/
layout: single
classes: wide
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
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
{% for post in sorted_posts %}
  {% assign counter = 0 %}
  {% for tag in post.tags %}
    {% if tag == "tutorial" or tag == "control" or tag == "lti"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}{% break %}
    {% endif %}
  {% endfor %}
{% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br><br><br><br>

### LTV systems
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
{% for post in sorted_posts %}
  {% assign counter = 0 %}
  {% for tag in post.tags %}
    {% if tag == "tutorial" or tag == "control" or tag == "ltv"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}{% break %}
    {% endif %}
  {% endfor %}
{% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br><br><br><br>

***

# Estimation
### LTI systems
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
{% for post in sorted_posts %}
  {% assign counter = 0 %}
  {% for tag in post.tags %}
    {% if tag == "tutorial" or tag == "estimation" or tag == "lti"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}{% break %}
    {% endif %}
  {% endfor %}
{% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br><br><br><br>
### LTV systems
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
{% for post in sorted_posts %}
  {% assign counter = 0 %}
  {% for tag in post.tags %}
    {% if tag == "tutorial" or tag == "estimation" or tag == "ltv"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}{% break %}
    {% endif %}
  {% endfor %}
{% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br><br><br><br>

***

# Installation
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
{% for post in sorted_posts %}
  {% assign counter = 0 %}
  {% for tag in post.tags %}
    {% if tag == "tutorial" or tag == "installation" %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 2 %}
      {% include archive-single.html type="grid" %}{% break %}
    {% endif %}
  {% endfor %}
{% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br><br><br><br>
