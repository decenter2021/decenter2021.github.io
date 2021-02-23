// https://lunrjs.com/guides/index_prebuilding.html

process.stdin.resume()
process.stdin.setEncoding('utf8')

const lunr = require('lunr'),
      readline = require('readline'),
      lang = process.argv[2],
      rl = readline.createInterface({
        input: process.stdin,
        output: process.stdout,
        terminal: false
      })

// Load Lunr languages
if (lang !== 'en') {
  require("lunr-languages/lunr.stemmer.support")(lunr)
  require(`lunr-languages/lunr.${lang}`)(lunr)
}

let builder

// Process each document as a separate line
rl.on('line', line => {
  const doc = JSON.parse(line)

  // If the builder isn't initialized, create it
  if (!builder) {
    builder = new lunr.Builder
    builder.pipeline.add(
      lunr.trimmer,
      lunr.stopWordFilter,
      lunr.stemmer
    )

    builder.searchPipeline.add(lunr.stemmer)

    if (lang !== 'en') builder.use(lunr[lang])

    Object.keys(doc).forEach(field => builder.field(field))
  }

  builder.add(doc)
})

process.stdin.on('end', () => process.stdout.write(JSON.stringify(builder.build())))
