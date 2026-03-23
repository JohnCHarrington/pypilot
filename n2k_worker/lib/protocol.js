const readline = require('readline')

function emit(event) {
  process.stdout.write(JSON.stringify(event) + '\n')
}

function emitStatus(state, details) {
  emit({ type: 'status', state, details: details || {} })
}

function emitError(error, details) {
  emit({
    type: 'error',
    error: error && error.message ? error.message : String(error),
    details: details || {}
  })
}

function createCommandReader(onCommand) {
  const rl = readline.createInterface({
    input: process.stdin,
    terminal: false
  })

  rl.on('line', (line) => {
    const raw = line.trim()
    if (!raw) {
      return
    }

    let command
    try {
      command = JSON.parse(raw)
    } catch (error) {
      emitError(error, { raw, phase: 'parse-command' })
      return
    }

    try {
      onCommand(command)
    } catch (error) {
      emitError(error, { command, phase: 'handle-command' })
    }
  })

  rl.on('close', () => {
    onCommand({ type: 'shutdown' })
  })

  return rl
}

module.exports = {
  createCommandReader,
  emit,
  emitError,
  emitStatus
}
