#!/usr/bin/env python

radians = 3.141592653589793/180
meters_s = 0.5144456333854638

# provide bi-directional translation of these keys
signalk_table = {
    'wind': {
        ('environment.wind.speedApparent', meters_s): 'speed',
        ('environment.wind.angleApparent', radians): 'direction'
    },
    'truewind': {
        ('environment.wind.speedTrue', meters_s): 'speed',
        ('environment.wind.angleTrue', radians): 'direction'
    },
    'gps': {
        ('navigation.courseOverGroundTrue', radians): 'track',
        ('navigation.speedOverGround', meters_s): 'speed',
        ('navigation.position', 1): {
            'latitude': 'lat',
            'longitude': 'lon'
        }
    },
    'rudder': {
        ('steering.rudderAngle', -radians): 'angle'
    },
    'apb': {
        ('steering.autopilot.target.headingTrue', radians): 'track'
    },
    'imu': {
        ('navigation.headingMagnetic', radians): 'heading_lowpass',
        ('navigation.attitude', radians): {
            'pitch': 'pitch',
            'roll': 'roll',
            'yaw': 'heading_lowpass'
        },
        ('navigation.rateOfTurn', radians): 'headingrate_lowpass'
    },
    'water': {
        ('navigation.speedThroughWater', meters_s): 'speed',
        ('navigation.leewayAngle', radians): 'leeway'
    }
}
