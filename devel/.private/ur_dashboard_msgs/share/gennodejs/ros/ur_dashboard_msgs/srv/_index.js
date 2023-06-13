
"use strict";

let GetProgramState = require('./GetProgramState.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let Load = require('./Load.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetRobotMode = require('./GetRobotMode.js')
let AddToLog = require('./AddToLog.js')
let Popup = require('./Popup.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let RawRequest = require('./RawRequest.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let IsProgramRunning = require('./IsProgramRunning.js')

module.exports = {
  GetProgramState: GetProgramState,
  IsProgramSaved: IsProgramSaved,
  Load: Load,
  GetLoadedProgram: GetLoadedProgram,
  GetRobotMode: GetRobotMode,
  AddToLog: AddToLog,
  Popup: Popup,
  GetSafetyMode: GetSafetyMode,
  RawRequest: RawRequest,
  IsInRemoteControl: IsInRemoteControl,
  IsProgramRunning: IsProgramRunning,
};
