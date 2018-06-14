
"use strict";

let GetThrusterEfficiency = require('./GetThrusterEfficiency.js')
let GetThrusterState = require('./GetThrusterState.js')
let SetThrusterState = require('./SetThrusterState.js')
let SetUseGlobalCurrentVel = require('./SetUseGlobalCurrentVel.js')
let SetThrusterEfficiency = require('./SetThrusterEfficiency.js')
let GetFloat = require('./GetFloat.js')
let GetModelProperties = require('./GetModelProperties.js')
let SetFloat = require('./SetFloat.js')

module.exports = {
  GetThrusterEfficiency: GetThrusterEfficiency,
  GetThrusterState: GetThrusterState,
  SetThrusterState: SetThrusterState,
  SetUseGlobalCurrentVel: SetUseGlobalCurrentVel,
  SetThrusterEfficiency: SetThrusterEfficiency,
  GetFloat: GetFloat,
  GetModelProperties: GetModelProperties,
  SetFloat: SetFloat,
};
