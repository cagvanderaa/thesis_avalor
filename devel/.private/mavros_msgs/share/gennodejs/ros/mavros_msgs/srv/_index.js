
"use strict";

let LogRequestData = require('./LogRequestData.js')
let FileWrite = require('./FileWrite.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let FileOpen = require('./FileOpen.js')
let WaypointClear = require('./WaypointClear.js')
let ParamPull = require('./ParamPull.js')
let FileChecksum = require('./FileChecksum.js')
let ParamPush = require('./ParamPush.js')
let FileRemove = require('./FileRemove.js')
let WaypointPull = require('./WaypointPull.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandTOL = require('./CommandTOL.js')
let FileRename = require('./FileRename.js')
let ParamGet = require('./ParamGet.js')
let CommandBool = require('./CommandBool.js')
let CommandHome = require('./CommandHome.js')
let WaypointPush = require('./WaypointPush.js')
let FileMakeDir = require('./FileMakeDir.js')
let LogRequestList = require('./LogRequestList.js')
let SetMode = require('./SetMode.js')
let CommandLong = require('./CommandLong.js')
let FileTruncate = require('./FileTruncate.js')
let CommandInt = require('./CommandInt.js')
let FileClose = require('./FileClose.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileRead = require('./FileRead.js')
let ParamSet = require('./ParamSet.js')
let MessageInterval = require('./MessageInterval.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let CommandAck = require('./CommandAck.js')
let StreamRate = require('./StreamRate.js')
let FileList = require('./FileList.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let MountConfigure = require('./MountConfigure.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')

module.exports = {
  LogRequestData: LogRequestData,
  FileWrite: FileWrite,
  VehicleInfoGet: VehicleInfoGet,
  FileOpen: FileOpen,
  WaypointClear: WaypointClear,
  ParamPull: ParamPull,
  FileChecksum: FileChecksum,
  ParamPush: ParamPush,
  FileRemove: FileRemove,
  WaypointPull: WaypointPull,
  SetMavFrame: SetMavFrame,
  CommandTOL: CommandTOL,
  FileRename: FileRename,
  ParamGet: ParamGet,
  CommandBool: CommandBool,
  CommandHome: CommandHome,
  WaypointPush: WaypointPush,
  FileMakeDir: FileMakeDir,
  LogRequestList: LogRequestList,
  SetMode: SetMode,
  CommandLong: CommandLong,
  FileTruncate: FileTruncate,
  CommandInt: CommandInt,
  FileClose: FileClose,
  FileRemoveDir: FileRemoveDir,
  FileRead: FileRead,
  ParamSet: ParamSet,
  MessageInterval: MessageInterval,
  CommandTriggerInterval: CommandTriggerInterval,
  CommandAck: CommandAck,
  StreamRate: StreamRate,
  FileList: FileList,
  LogRequestEnd: LogRequestEnd,
  CommandVtolTransition: CommandVtolTransition,
  WaypointSetCurrent: WaypointSetCurrent,
  MountConfigure: MountConfigure,
  CommandTriggerControl: CommandTriggerControl,
};
