---
layout: splash
title: "DECENTER"
entries_layout: grid
excerpt: "Distributed control and estimation toolbox for MATLAB"
header:
  overlay_image: assets/img/network-4556932.jpg
  overlay_filter: 0.1 # same as adding an opacity of 0.5 to a black background
  #caption: "Photo credit: [**Unsplash**](https://unsplash.com)"
  actions:
    - label: "Download"
      url: "https://github.com/decenter2021/decenter/archive/v1.0.0.zip"
---

<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
  {% for post in sorted_posts %}
   {% if post.frontPage == "true" %}
    {% include archive-single.html type="grid" %}
    {% endif %}
  {% endfor %}
</div>
