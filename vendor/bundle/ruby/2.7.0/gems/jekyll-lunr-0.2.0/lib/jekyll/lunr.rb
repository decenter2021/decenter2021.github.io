# frozen_string_literal: true

require 'json'
require 'open3'
require 'loofah'

module Jekyll
  module Lunr
    # Error
    class Error < StandardError; end

    module IndexableSite
      def self.included(base)
        base.class_eval do
          def indexer
            @indexer ||= Jekyll::Lunr::Indexer.new self
          end
        end
      end
    end

    module IndexableDocument
      def self.included(base)
        base.class_eval do
          def to_data
            data
              .slice(*site.indexer.fields)
              .transform_values { |value| extract_data_recursively(value) }
              .merge('content' => Loofah.fragment(content).to_text,
                     'url' => url,
                     'id' => url,
                     'year' => date&.year)
          end

          def extract_data_recursively(value)
            case value
            when Array then value.map { |v| extract_data(v) }
            when Hash then value.transform_values { |v| extract_data(v) }
            when Jekyll::Document then extract_data(value)
            when Jekyll::Page then extract_data(value)
            else value
            end
          end

          def extract_data(value)
            value.respond_to?(:to_data) ? value.to_data : value
          end
        end
      end
    end

    # Generates a Lunr index from documents
    class Indexer
      attr_reader :site

      def initialize(site)
        @site = site
      end

      # The data is the register where Lunr looks for results.
      # TODO: Write a single data file per doc?
      def data
        @data ||= site.documents.map(&:to_data)
      end

      def data_file
        @data_file ||= Jekyll::StaticFile.new(site, site.source, '.', 'data.json')
      end

      # Convert data to strings since Lunr can't index objects
      def indexable_data
        @indexable_data ||= data.map do |d|
          d.transform_values do |v|
            case v
            when Array then v.join(', ')
            when Hash then v.values.join(', ')
            else v.to_s
            end
          end
        end
      end

      def write
        File.open(data_file.path, 'w') do |df|
          df.write data.to_json
        end

        site.static_files << data_file
      end

      def dir
        File.realpath(File.join([__dir__, '..', '..']))
      end

      def index_file
        @index_file ||= Jekyll::StaticFile.new(site, site.source, '.', 'idx.json')
      end

      def indexer
        @indexer ||= ['node', File.join(dir, 'lib', 'assets', 'javascript', 'indexer.js'), lang].freeze
      end

      def env
        @env ||= { 'NODE_PATH' => File.join(site.source, 'node_modules') }
      end

      def index
        Open3.popen2(env, *indexer) do |stdin, stdout, wait|
          indexable_data.each do |data|
            stdin.puts data.to_json
          end
          stdin.close

          File.open(index_file.path, 'w') do |idx|
            idx.write(stdout.read)
          end

          site.static_files << index_file

          wait.value
        end
      end

      # Site lang
      def lang
        @lang ||= site.config.dig('lang').freeze
      end

      # Indexable fields
      def fields
        @fields ||= Set.new((site.config.dig('jekyll-lunr', 'fields') || []) + %w[title description]).freeze
      end

      def free
        @data = nil
        @indexable_data = nil
      end
    end
  end
end

Jekyll::Site.include Jekyll::Lunr::IndexableSite
Jekyll::Document.include Jekyll::Lunr::IndexableDocument
Jekyll::Page.include Jekyll::Lunr::IndexableDocument
