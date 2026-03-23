# pypilot N2K Worker

This worker bridges pypilot to NMEA 2000 using:

- `@canboat/canboatjs` for SocketCAN access and PGN transmit support
- `n2k-signalk` for converting inbound N2K traffic into Signal K deltas

## Requirements

- Node.js 20 or newer
- A working SocketCAN device such as `can0`

## Install

From the repository root:

```sh
cd n2k_worker
npm install
```

## Run

```sh
npm start
```

The worker reads line-delimited JSON commands on stdin and emits line-delimited JSON events on stdout.

## Commands

- `{"type":"configure","config":{"canDevice":"can0","preferredAddress":25}}`
- `{"type":"set-enabled","enabled":true}`
- `{"type":"send","payload":{"topic":"heading","heading":123.4}}`
- `{"type":"shutdown"}`

## Events

- `status`
- `delta`
- `metadata`
- `warning`
- `tx`
- `worker-error`
