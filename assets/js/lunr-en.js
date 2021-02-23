---

---
var idx = lunr(function () {
  this.field('title', {boost: 10})
  this.field('excerpt')
  this.field('categories')
  this.field('tags')
  this.ref('id')
});

{% assign count = 0 %}
{% for doc in site.pages %}


    idx.add({
      title: {{ doc.title | jsonify }},
      excerpt: {{ doc.excerpt | strip_html | truncatewords: 20 | jsonify }},
      categories: {{ doc.categories | jsonify }},
      tags: {{ doc.tags | jsonify }},
      id: {{ count }}
    });
    {% assign count = count | plus: 1 %}

{% endfor %}

console.log( jQuery.type(idx) );

var store = [
  {% for doc in site.pages %}
    {% if forloop.last %}
      {% assign l = true %}
    {% endif %}
      {
        "title": {{ doc.title | jsonify }},
        "url": {{ doc.url | absolute_url | jsonify }},
        "excerpt": {{ doc.content | strip_html | truncatewords: 15 | jsonify }}
      }{% unless forloop.last and l %},{% endunless %}

  {% endfor %}]

$(document).ready(function() {
  $('input#search').on('keyup', function () {
    var resultdiv = $('#results');
    var query = $(this).val();
    var result = idx.search(query);
    resultdiv.empty();
    resultdiv.prepend('<p>'+result.length+' Result(s) found</p>');
    for (var item in result) {
      var ref = result[item].ref;

    	  var searchitem =
          '<div>'+
            '<article class="archive__item" itemscope itemtype="http://schema.org/CreativeWork">'+
              '<h4 class="archive__item-title" itemprop="headline">'+
                '<a href="'+store[ref].url+'" rel="permalink">'+store[ref].title+'</a>'+
              '</h4>'+
              '<p class="archive__item-excerpt" itemprop="description">'+store[ref].excerpt+'</p>'+
            '</article>'+
          '</div>';
      resultdiv.append(searchitem);
    }
  });
});
