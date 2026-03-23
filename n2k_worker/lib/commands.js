const radians = Math.PI / 180
const knotsToMetersPerSecond = 0.5144456333854638

function asNumber(value, fallback) {
  const number = Number(value)
  return Number.isFinite(number) ? number : fallback
}

function degreesToRadians(value) {
  return asNumber(value, 0) * radians
}

function knotsToMeters(value) {
  return asNumber(value, 0) * knotsToMetersPerSecond
}

function modeToControllerMode(mode) {
  switch (mode) {
    case 'nav':
      return 'track control'
    case 'gps':
      return 'course hold'
    case 'wind':
      return 'wind'
    case 'true wind':
      return 'true wind'
    default:
      return 'heading hold'
  }
}

function buildHeadingCommand(command, sourceAddress) {
  return [{
    pgn: 127250,
    src: sourceAddress,
    dst: 255,
    prio: 3,
    fields: {
      Heading: degreesToRadians(command.heading)
    }
  }]
}

function buildRateOfTurnCommand(command, sourceAddress) {
  return [{
    pgn: 127251,
    src: sourceAddress,
    dst: 255,
    prio: 3,
    fields: {
      'Rate of Turn': degreesToRadians(command.rate)
    }
  }]
}

function buildAttitudeCommand(command, sourceAddress) {
  return [{
    pgn: 127257,
    src: sourceAddress,
    dst: 255,
    prio: 3,
    fields: {
      Pitch: degreesToRadians(command.pitch),
      Roll: degreesToRadians(command.roll)
    }
  }]
}

function buildWindCommand(command, sourceAddress) {
  return [{
    pgn: 130306,
    src: sourceAddress,
    dst: 255,
    prio: 3,
    fields: {
      'Wind Angle': degreesToRadians(command.direction),
      'Wind Speed': knotsToMeters(command.speed),
      Reference: command.reference || 'Apparent'
    }
  }]
}

function buildRudderCommand(command, sourceAddress) {
  return [{
    pgn: 127245,
    src: sourceAddress,
    dst: 255,
    prio: 3,
    fields: {
      'Rudder Angle': -degreesToRadians(command.angle)
    }
  }]
}

function buildAutopilotCommand(command, sourceAddress) {
  const mode = command.mode || 'compass'
  const heading = degreesToRadians(command.heading_command)
  const fields = {
    'Autopilot Engagement Status': !!command.enabled,
    'Heading/Track Controller Mode': modeToControllerMode(mode),
    'Heading Commanded': heading
  }

  if (mode === 'nav') {
    fields['Course/Track'] = heading
  }

  return [{
    pgn: 127237,
    src: sourceAddress,
    dst: 255,
    prio: 3,
    fields
  }]
}

function buildGpsCommand(command, sourceAddress) {
  const messages = []
  const track = asNumber(command.track, 0)

  if (command.lat !== undefined && command.lon !== undefined) {
    messages.push({
      pgn: 129029,
      src: sourceAddress,
      dst: 255,
      prio: 3,
      fields: {
        Latitude: asNumber(command.lat, 0),
        Longitude: asNumber(command.lon, 0)
      }
    })
  }

  if (command.speed !== undefined || command.track !== undefined) {
    messages.push({
      pgn: 129026,
      src: sourceAddress,
      dst: 255,
      prio: 3,
      fields: {
        'Course Over Ground': degreesToRadians(track >= 0 ? track : 360 + track),
        'Speed Over Ground': knotsToMeters(command.speed)
      }
    })
  }

  if (command.timestamp !== undefined) {
    messages.push({
      pgn: 129033,
      src: sourceAddress,
      dst: 255,
      prio: 3,
      fields: {
        UTC: command.timestamp
      }
    })
  }

  return messages
}

function buildPgns(command, sourceAddress) {
  switch (command.topic) {
    case 'heading':
      return buildHeadingCommand(command, sourceAddress)
    case 'rate_of_turn':
      return buildRateOfTurnCommand(command, sourceAddress)
    case 'attitude':
      return buildAttitudeCommand(command, sourceAddress)
    case 'wind':
      return buildWindCommand(command, sourceAddress)
    case 'truewind':
      return buildWindCommand({
        direction: command.direction,
        speed: command.speed,
        reference: 'True'
      }, sourceAddress)
    case 'rudder':
      return buildRudderCommand(command, sourceAddress)
    case 'autopilot':
      return buildAutopilotCommand(command, sourceAddress)
    case 'gps':
      return buildGpsCommand(command, sourceAddress)
    case 'send-pgn':
      return [command.pgn]
    default:
      throw new Error('unsupported command topic: ' + command.topic)
  }
}

module.exports = {
  buildPgns
}
