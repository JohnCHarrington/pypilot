const EventEmitter = require('events')
const canboatjs = require('@canboat/canboatjs')

const { buildPgns } = require('./commands')

function loadMapper() {
  try {
    const pkg = require('n2k-signalk')
    if (pkg && pkg.N2kMapper) {
      return pkg.N2kMapper
    }
  } catch (error) {
  }

  try {
    return require('n2k-signalk/dist/n2kMapper').N2kMapper
  } catch (error) {
    throw error
  }
}

const N2kMapper = loadMapper()

class WorkerDevice extends EventEmitter {
  constructor(config) {
    super()
    this.config = Object.assign({}, config)
    this.started = false
    this.simpleCan = null
    this.parser = null
    this.mapper = null
    this.mapperState = {}
    this.app = new EventEmitter()
    this.bindAppEvents()
  }

  bindAppEvents() {
    this.app.on('n2k-signalk-out', (pgn) => {
      this.sendPgn(pgn)
    })
  }

  getOptions() {
    const uniqueNumber = Number(this.config.uniqueNumber || 1731561)
    return {
      app: this.app,
      canDevice: this.config.canDevice || 'can0',
      preferredAddress: Number(this.config.preferredAddress || 25),
      disableDefaultTransmitPGNs: true,
      transmitPGNs: [126996, 126998, 127237, 127245, 127250, 127251, 127257, 128259, 129026, 129029, 129033, 129283, 129284, 130306],
      addressClaim: {
        'Unique Number': uniqueNumber,
        'Manufacturer Code': this.config.manufacturerCode || 'Fusion Electronics',
        'Device Function': Number(this.config.deviceFunction || 150),
        'Device Class': this.config.deviceClass || 'Navigation',
        'Device Instance Lower': Number(this.config.deviceInstanceLower || 0),
        'Device Instance Upper': Number(this.config.deviceInstanceUpper || 0),
        'System Instance': Number(this.config.systemInstance || 0),
        'Industry Group': this.config.industryGroup || 'Marine'
      },
      productInfo: {
        'NMEA 2000 Version': Number(this.config.nmea2000Version || 2100),
        'Product Code': Number(this.config.productCode || 246),
        'Model ID': this.config.modelId || 'pypilot',
        'Software Version Code': this.config.softwareVersionCode || '0.1.0',
        'Model Version': this.config.modelVersion || 'canboatjs',
        'Model Serial Code': this.config.modelSerialCode || String(uniqueNumber),
        'Certification Level': Number(this.config.certificationLevel || 1),
        'Load Equivalency': Number(this.config.loadEquivalency || 1)
      }
    }
  }

  start() {
    if (this.started) {
      return
    }

    this.parser = new canboatjs.FromPgn({
      useCamel: true,
      returnNulls: true,
      resolveEnums: true
    })

    this.mapper = new N2kMapper()
    if (this.mapper.n2kOutIsAvailable) {
      this.mapper.n2kOutIsAvailable(this.app, 'n2k-signalk-out')
    }

    this.mapper.on('n2kSourceMetadata', (n2k, meta) => {
      this.emit('metadata', { n2k, meta })
    })
    this.mapper.on('n2kSourceChanged', (src, oldCanName, newCanName) => {
      this.emit('source-changed', { src, oldCanName, newCanName })
    })
    this.mapper.on('n2kSourceMetadataTimeout', (pgn, src) => {
      this.emit('warning', { pgn, src, message: 'metadata timeout' })
    })

    this.parser.on('warning', (pgn, warning) => {
      this.emit('warning', { pgn: pgn && pgn.pgn, warning })
    })
    this.parser.on('error', (pgn, error) => {
      this.emit('error', { pgn: pgn && pgn.pgn, error: error && error.message ? error.message : String(error) })
    })

    this.simpleCan = new canboatjs.SimpleCan(this.getOptions(), (raw) => {
      this.onRawMessage(raw)
    })
    this.simpleCan.start()
    this.started = true
    this.emit('started', { canDevice: this.getOptions().canDevice })
  }

  stop() {
    if (!this.started) {
      return
    }

    try {
      if (this.simpleCan && this.simpleCan.channel && this.simpleCan.channel.stop) {
        this.simpleCan.channel.stop()
      }
    } catch (error) {
      this.emit('error', { error: error && error.message ? error.message : String(error) })
    }

    this.simpleCan = null
    this.parser = null
    this.mapper = null
    this.mapperState = {}
    this.started = false
    this.emit('stopped', {})
  }

  restart() {
    this.stop()
    this.start()
  }

  updateConfig(nextConfig) {
    const oldDevice = this.config.canDevice
    const oldAddress = this.config.preferredAddress
    this.config = Object.assign({}, this.config, nextConfig)
    if (this.started && (oldDevice !== this.config.canDevice || oldAddress !== this.config.preferredAddress)) {
      this.restart()
    }
  }

  onRawMessage(raw) {
    if (!this.parser) {
      return
    }

    try {
      const parsed = this.parser.parse(raw)
      if (!parsed || !this.mapper) {
        return
      }

      const delta = this.mapper.toDelta(parsed)
      if (delta && delta.updates && delta.updates.length) {
        const values = delta.updates[0].values || []
        if (values.length) {
          this.emit('delta', { parsed, delta })
        }
      }
    } catch (error) {
      this.emit('error', { error: error && error.message ? error.message : String(error), raw })
    }
  }

  sendPgn(pgn) {
    if (!this.simpleCan) {
      return
    }
    this.simpleCan.sendPGN(pgn)
    this.emit('tx', { pgn })
  }

  sendSemantic(command) {
    if (!this.started) {
      return
    }

    const sourceAddress = Number(this.config.preferredAddress || 25)
    const pgns = buildPgns(command, sourceAddress)
    pgns.forEach((pgn) => {
      this.sendPgn(pgn)
    })
  }
}

module.exports = {
  WorkerDevice
}
