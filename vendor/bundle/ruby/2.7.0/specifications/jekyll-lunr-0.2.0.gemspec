# -*- encoding: utf-8 -*-
# stub: jekyll-lunr 0.2.0 ruby lib

Gem::Specification.new do |s|
  s.name = "jekyll-lunr".freeze
  s.version = "0.2.0"

  s.required_rubygems_version = Gem::Requirement.new(">= 0".freeze) if s.respond_to? :required_rubygems_version=
  s.metadata = { "bug_tracker_uri" => "https://0xacab.org/sutty/jekyll/jekyll-lunr/issues", "changelog_uri" => "https://0xacab.org/sutty/jekyll/jekyll-lunr/-/blob/master/CHANGELOG.md", "documentation_uri" => "https://rubydoc.info/gems/jekyll-lunr", "homepage_uri" => "https://0xacab.org/sutty/jekyll/jekyll-lunr", "source_code_uri" => "https://0xacab.org/sutty/jekyll/jekyll-lunr" } if s.respond_to? :metadata=
  s.require_paths = ["lib".freeze]
  s.authors = ["f".freeze]
  s.date = "2020-10-15"
  s.description = "Builds the LunrJS index and data documents during site generation".freeze
  s.email = ["f@sutty.nl".freeze]
  s.extra_rdoc_files = ["README.md".freeze, "LICENSE.txt".freeze]
  s.files = ["LICENSE.txt".freeze, "README.md".freeze]
  s.homepage = "https://0xacab.org/sutty/jekyll/jekyll-lunr".freeze
  s.licenses = ["GPL-3.0".freeze]
  s.rdoc_options = ["--title".freeze, "jekyll-lunr - Lunr indexer for Jekyll".freeze, "--main".freeze, "README.md".freeze, "--line-numbers".freeze, "--inline-source".freeze, "--quiet".freeze]
  s.required_ruby_version = Gem::Requirement.new(">= 2.6.0".freeze)
  s.rubygems_version = "3.1.4".freeze
  s.summary = "Lunr indexer for Jekyll".freeze

  s.installed_by_version = "3.1.4" if s.respond_to? :installed_by_version

  if s.respond_to? :specification_version then
    s.specification_version = 4
  end

  if s.respond_to? :add_runtime_dependency then
    s.add_runtime_dependency(%q<loofah>.freeze, ["~> 2.4"])
    s.add_development_dependency(%q<bundler>.freeze, ["~> 2.0"])
    s.add_development_dependency(%q<rake>.freeze, ["~> 10.0"])
  else
    s.add_dependency(%q<loofah>.freeze, ["~> 2.4"])
    s.add_dependency(%q<bundler>.freeze, ["~> 2.0"])
    s.add_dependency(%q<rake>.freeze, ["~> 10.0"])
  end
end
