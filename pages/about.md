---
title: About
permalink: /about/
layout: single
classes: wide
sidebar:
  title: "About"
  nav: sidebar-about
---
Project
=====
DECENTER toolbox is being developed within <a target="_blank" href ="https://decenterproject.weebly.com">project DECENTER</a>.
<hr>
Authors
=====
{% for author in site.data.authors %} {{author.name}}<sup>{{author.affiliation}}</sup> | {% if author.website %}<a itemprop="sameAs" target="_blank" content="{{author.website}}" href="{{author.website}}" target="blank" rel="me noopener noreferrer" style="vertical-align:top;"><img src="/assets/img/emoji/link_1f517.png" style="width:1em;margin-right:.5em;" alt="Website"></a>{%endif%}{% if author.orcid %}<a itemprop="sameAs" target="_blank" content="{{author.orcid}}" href="{{author.orcid}}" target="orcid.widget" rel="me noopener noreferrer" style="vertical-align:top;"><img src="https://orcid.org/sites/default/files/images/orcid_16x16.png" style="width:1em;margin-right:.5em;" alt="ORCID iD icon"></a>{%endif%}{% if author.googlescholar %}<a target="_blank" itemprop="sameAs" href="{{author.googlescholar}}" style="vertical-align:top;"><img src="https://upload-icon.s3.us-east-2.amazonaws.com/uploads/icons/png/17520148421579517848-512.png" style="width:1em;margin-right:.5em;"></a>{%endif%}{% if author.github %}<a itemprop="sameAs" target="_blank" content="{{author.github}}" href="{{author.github}}" rel="me noopener noreferrer" style="vertical-align:top;"><img src="https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png" style="width:1em;margin-right:.5em;" alt="GitHub icon"></a>{%endif%}<br>{% endfor %}
{% for affiliation in site.data.author-affiliation %} <sup>{{affiliation.superscript}}</sup>{{affiliation.affiliation}}{%endfor%}


<hr>
Contact
=====
DECENTER is currently mantained by Leonardo Pedroso (<a href = "mailto: leonardo.pedroso@tecnico.ulisboa.pt">leonardo.pedroso@tecnico.ulisboa.pt</a>).

<hr>

Acknowledgments
=====
This work was supported by the Fundação para a Ciência e a Tecnologia (FCT) through LARSyS - FCT Project UIDB/50009/2020 and through the FCT project DECENTER [LISBOA-01-0145-FEDER-029605], funded by the Programa Operacional Regional de Lisboa 2020 and PIDDAC programs.
{: .text-justify}
