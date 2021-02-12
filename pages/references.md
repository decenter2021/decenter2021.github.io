---
title: References
permalink: /references/
layout: single
classes: wide
sidebar:
  title: "References"
  nav: sidebar-references
---

{% for paper in site.data.references%}
<a href="{{paper.url}}" target="_blank">{{paper.harvardCitation}}</a>
{: .text-justify}
{% endfor %}
