require 'jekyll/lunr'

Jekyll::Hooks.register :site, :pre_render do |site|
  Jekyll.logger.info 'Generating data.json'
  site.indexer.write

  Jekyll.logger.info 'Generating idx.json'
  site.indexer.index

  Jekyll.logger.info 'Freeing memory'
  site.indexer.free
end
