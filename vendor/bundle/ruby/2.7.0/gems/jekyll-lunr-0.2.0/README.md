# jekyll-lunr

[LunrJS](https://lunrjs.com/) is a search engine that can run in
browser.  This plugin generates a Lunr index during site generation and
allows you to download it from your search box, instead of generating it
at runtime and making visitors wait.

## Installation

Add this line to your application's Gemfile:

```ruby
group :jekyll_plugins do
  gem 'jekyll-lunr'
end
```

And then execute:

    $ bundle

Or install it yourself as:

    $ gem install jekyll-lunr

Also, add the node packages to your site source:

```bash
yarn add lunr lunr-languages
```

## Usage

Add the plugin to the plugins array in `_config.yml`:

```yaml
plugins:
- jekyll-lunr
```

Additionally, you can add extra fields for indexation.  The default
front matter fields are `title`, `description`, `url` and `year`.

**Important:** Currently the plugin is only able to index one level of
the front matter hash.

```yaml
jekyll-lunr:
  fields:
  - image
```

## Usage

Two files will be generated, `data.json` and `idx.json`.  You can
download them from your `search.js`.

## Development

After checking out the repo, run `bin/setup` to install dependencies.
Then, run `rake test` to run the tests. You can also run `bin/console`
for an interactive prompt that will allow you to experiment.

To install this gem onto your local machine, run `bundle exec rake
install`. To release a new version, update the version number in
`version.rb`, and then run `bundle exec rake release`, which will create
a git tag for the version, push git commits and tags, and push the
`.gem` file to [rubygems.org](https://rubygems.org).

## Contributing

Bug reports and pull requests are welcome on 0xacab.org at
<https://0xacab.org/sutty/jekyll/jekyll-lunr>. This
project is intended to be a safe, welcoming space for collaboration, and
contributors are expected to adhere to the [Sutty code of
conduct](https://sutty.nl/en/code-of-conduct/).

## License

The gem is available as free software under the terms of the GPL3
License.

## Code of Conduct

Everyone interacting in the jekyll-lunr project’s codebases, issue
trackers, chat rooms and mailing lists is expected to follow the [code
of conduct](https://sutty.nl/en/code-of-conduct/).
