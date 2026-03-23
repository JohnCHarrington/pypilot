#!/usr/bin/env node

const { WorkerDevice } = require('./lib/device')
const { createCommandReader, emit, emitError, emitStatus } = require('./lib/protocol')

const config = {
  canDevice: process.env.PYPILOT_N2K_CAN_DEVICE || 'can0',
  preferredAddress: Number(process.env.PYPILOT_N2K_ADDRESS || 25),
  modelId: process.env.PYPILOT_N2K_MODEL_ID || 'pypilot',
  modelVersion: process.env.PYPILOT_N2K_MODEL_VERSION || 'canboatjs',
  softwareVersionCode: process.env.PYPILOT_N2K_SOFTWARE_VERSION || '0.1.0'
}

const worker = new WorkerDevice(config)
let enabled = true

worker.on('started', (details) => emitStatus('connected', details))
worker.on('stopped', (details) => emitStatus('stopped', details))
worker.on('warning', (details) => emit({ type: 'warning', details }))
worker.on('metadata', (details) => emit({ type: 'metadata', details }))
worker.on('source-changed', (details) => emit({ type: 'source-changed', details }))
worker.on('tx', (details) => emit({ type: 'tx', details }))
worker.on('delta', (details) => emit({ type: 'delta', details }))
worker.on('error', (details) => emit({ type: 'worker-error', details }))

function setEnabled(nextEnabled) {
  enabled = !!nextEnabled
  if (enabled) {
    worker.start()
  } else {
    worker.stop()
  }
}

createCommandReader((command) => {
  switch (command.type) {
    case 'configure':
      worker.updateConfig(command.config || {})
      emitStatus('configured', { config: command.config || {} })
      break
    case 'set-enabled':
      setEnabled(command.enabled)
      emitStatus(enabled ? 'enabled' : 'disabled', {})
      break
    case 'send':
      worker.sendSemantic(command.payload || {})
      break
    case 'shutdown':
      worker.stop()
      emitStatus('shutdown', {})
      process.exit(0)
      break
    default:
      emitError(new Error('unsupported command type: ' + command.type), { command })
      break
  }
})

try {
  if (enabled) {
    worker.start()
  }
} catch (error) {
  emitError(error, { phase: 'startup' })
  process.exit(1)
}
