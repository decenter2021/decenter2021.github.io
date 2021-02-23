---
title: Examples
permalink: /examples/
layout: single
sidebar:
  title: "Examples"
  nav: sidebar-examples
classes: wide
showTitle: true
excerpt: "Meaningful examples for illustrating the power of DECENTER."
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
    {% if tag == "example" or tag == "control" or tag == "lti"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}
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
    {% if tag == "example" or tag == "control" or tag == "ltv"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}
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
    {% if tag == "example" or tag == "estimation" or tag == "lti"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}
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
    {% if tag == "example" or tag == "estimation" or tag == "ltv"   %}
      {% assign counter = counter | plus:1 %}
    {% endif %}
    {% if counter == 3 %}
      {% include archive-single.html type="grid" %}
    {% endif %}
  {% endfor %}
{% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br><br><br><br>
