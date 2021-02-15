---
title: Tutorials
permalink: /tutorials/
layout: tags
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
   {% if post.category == "controlLTITutorial" %}
    {% include archive-single.html type="grid" %}
    {% endif %}
  {% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br>
### LTV systems
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
  {% for post in sorted_posts %}
   {% if post.category == "controlLTVTutorial" %}
    {% include archive-single.html type="grid" %}
    {% endif %}
  {% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br>

***

# Estimation
### LTI systems
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
  {% for post in sorted_posts %}
   {% if post.category == "estimationLTITutorial" %}
    {% include archive-single.html type="grid" %}
    {% endif %}
  {% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br>
### LTV systems
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
  {% for post in sorted_posts %}
   {% if post.category == "estimationLTVTutorial" %}
    {% include archive-single.html type="grid" %}
    {% endif %}
  {% endfor %}
</div>
<br> <br> <br> <br> <br> <br> <br> <br><br>

***

# Installation
<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
  {% for post in sorted_posts %}
   {% if post.category == "installationTutorial" %}
    {% include archive-single.html type="grid" %}
    {% endif %}
  {% endfor %}
</div>
