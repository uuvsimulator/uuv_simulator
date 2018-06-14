
"use strict";

let TransformFromSphericalCoord = require('./TransformFromSphericalCoord.js')
let TransformToSphericalCoord = require('./TransformToSphericalCoord.js')
let SetOriginSphericalCoord = require('./SetOriginSphericalCoord.js')
let GetOriginSphericalCoord = require('./GetOriginSphericalCoord.js')
let SetCurrentVelocity = require('./SetCurrentVelocity.js')
let GetCurrentModel = require('./GetCurrentModel.js')
let SetCurrentModel = require('./SetCurrentModel.js')
let SetCurrentDirection = require('./SetCurrentDirection.js')

module.exports = {
  TransformFromSphericalCoord: TransformFromSphericalCoord,
  TransformToSphericalCoord: TransformToSphericalCoord,
  SetOriginSphericalCoord: SetOriginSphericalCoord,
  GetOriginSphericalCoord: GetOriginSphericalCoord,
  SetCurrentVelocity: SetCurrentVelocity,
  GetCurrentModel: GetCurrentModel,
  SetCurrentModel: SetCurrentModel,
  SetCurrentDirection: SetCurrentDirection,
};
