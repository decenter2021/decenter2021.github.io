---
layout: splash
title: "DECENTER"
entries_layout: grid
excerpt: "Decentralized control and estimation toolbox for MATLAB"
header:
  overlay_image: assets/img/network-4556932.jpg
  overlay_filter: 0.1 # same as adding an opacity of 0.5 to a black background
  #caption: "Photo credit: [**Unsplash**](https://unsplash.com)"
actions:
  - label: "Download"
    url: /download/
---

<div class="grid__wrapper">
{% assign sorted_posts = site.pages | sort:"date" | reverse %}
{% for post in sorted_posts %}
{% for tag in post.tags %}
  {% if tag == "tutorial" or tag == "example"  %}
    {% include archive-single.html type="grid" %}
    {% endif %}
{% endfor %}
{% endfor %}
</div>
