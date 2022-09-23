
"use strict";

let hedge_pos = require('./hedge_pos.js');
let hedge_pos_a = require('./hedge_pos_a.js');
let hedge_quality = require('./hedge_quality.js');
let hedge_telemetry = require('./hedge_telemetry.js');
let beacon_pos_a = require('./beacon_pos_a.js');
let hedge_imu_fusion = require('./hedge_imu_fusion.js');
let marvelmind_waypoint = require('./marvelmind_waypoint.js');
let hedge_imu_raw = require('./hedge_imu_raw.js');
let beacon_distance = require('./beacon_distance.js');
let hedge_pos_ang = require('./hedge_pos_ang.js');

module.exports = {
  hedge_pos: hedge_pos,
  hedge_pos_a: hedge_pos_a,
  hedge_quality: hedge_quality,
  hedge_telemetry: hedge_telemetry,
  beacon_pos_a: beacon_pos_a,
  hedge_imu_fusion: hedge_imu_fusion,
  marvelmind_waypoint: marvelmind_waypoint,
  hedge_imu_raw: hedge_imu_raw,
  beacon_distance: beacon_distance,
  hedge_pos_ang: hedge_pos_ang,
};
