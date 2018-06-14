
"use strict";

let InitWaypointSet = require('./InitWaypointSet.js')
let GetWaypoints = require('./GetWaypoints.js')
let GetMBSMControllerParams = require('./GetMBSMControllerParams.js')
let Hold = require('./Hold.js')
let InitWaypointsFromFile = require('./InitWaypointsFromFile.js')
let SetMBSMControllerParams = require('./SetMBSMControllerParams.js')
let ClearWaypoints = require('./ClearWaypoints.js')
let StartTrajectory = require('./StartTrajectory.js')
let GetPIDParams = require('./GetPIDParams.js')
let IsRunningTrajectory = require('./IsRunningTrajectory.js')
let GetSMControllerParams = require('./GetSMControllerParams.js')
let SetSMControllerParams = require('./SetSMControllerParams.js')
let InitHelicalTrajectory = require('./InitHelicalTrajectory.js')
let InitRectTrajectory = require('./InitRectTrajectory.js')
let AddWaypoint = require('./AddWaypoint.js')
let SwitchToAutomatic = require('./SwitchToAutomatic.js')
let SwitchToManual = require('./SwitchToManual.js')
let InitCircularTrajectory = require('./InitCircularTrajectory.js')
let GoToIncremental = require('./GoToIncremental.js')
let SetPIDParams = require('./SetPIDParams.js')
let ResetController = require('./ResetController.js')
let GoTo = require('./GoTo.js')

module.exports = {
  InitWaypointSet: InitWaypointSet,
  GetWaypoints: GetWaypoints,
  GetMBSMControllerParams: GetMBSMControllerParams,
  Hold: Hold,
  InitWaypointsFromFile: InitWaypointsFromFile,
  SetMBSMControllerParams: SetMBSMControllerParams,
  ClearWaypoints: ClearWaypoints,
  StartTrajectory: StartTrajectory,
  GetPIDParams: GetPIDParams,
  IsRunningTrajectory: IsRunningTrajectory,
  GetSMControllerParams: GetSMControllerParams,
  SetSMControllerParams: SetSMControllerParams,
  InitHelicalTrajectory: InitHelicalTrajectory,
  InitRectTrajectory: InitRectTrajectory,
  AddWaypoint: AddWaypoint,
  SwitchToAutomatic: SwitchToAutomatic,
  SwitchToManual: SwitchToManual,
  InitCircularTrajectory: InitCircularTrajectory,
  GoToIncremental: GoToIncremental,
  SetPIDParams: SetPIDParams,
  ResetController: ResetController,
  GoTo: GoTo,
};
