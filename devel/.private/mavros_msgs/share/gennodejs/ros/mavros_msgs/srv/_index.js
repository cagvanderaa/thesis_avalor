
"use strict";

let CommandTOL = require('./CommandTOL.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let LogRequestList = require('./LogRequestList.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileRename = require('./FileRename.js')
let FileRead = require('./FileRead.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileTruncate = require('./FileTruncate.js')
let FileList = require('./FileList.js')
let WaypointPush = require('./WaypointPush.js')
let CommandLong = require('./CommandLong.js')
let CommandAck = require('./CommandAck.js')
let FileOpen = require('./FileOpen.js')
let CommandHome = require('./CommandHome.js')
let FileWrite = require('./FileWrite.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let SetMode = require('./SetMode.js')
let CommandBool = require('./CommandBool.js')
let ParamGet = require('./ParamGet.js')
let LogRequestData = require('./LogRequestData.js')
let WaypointPull = require('./WaypointPull.js')
let FileChecksum = require('./FileChecksum.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileRemove = require('./FileRemove.js')
let FileMakeDir = require('./FileMakeDir.js')
let FileClose = require('./FileClose.js')
let WaypointClear = require('./WaypointClear.js')
let MessageInterval = require('./MessageInterval.js')
let ParamPush = require('./ParamPush.js')
let CommandInt = require('./CommandInt.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let StreamRate = require('./StreamRate.js')
let ParamPull = require('./ParamPull.js')
let MountConfigure = require('./MountConfigure.js')
let ParamSet = require('./ParamSet.js')

module.exports = {
  CommandTOL: CommandTOL,
  VehicleInfoGet: VehicleInfoGet,
  LogRequestList: LogRequestList,
  FileRemoveDir: FileRemoveDir,
  FileRename: FileRename,
  FileRead: FileRead,
  LogRequestEnd: LogRequestEnd,
  FileTruncate: FileTruncate,
  FileList: FileList,
  WaypointPush: WaypointPush,
  CommandLong: CommandLong,
  CommandAck: CommandAck,
  FileOpen: FileOpen,
  CommandHome: CommandHome,
  FileWrite: FileWrite,
  CommandVtolTransition: CommandVtolTransition,
  SetMode: SetMode,
  CommandBool: CommandBool,
  ParamGet: ParamGet,
  LogRequestData: LogRequestData,
  WaypointPull: WaypointPull,
  FileChecksum: FileChecksum,
  CommandTriggerInterval: CommandTriggerInterval,
  SetMavFrame: SetMavFrame,
  FileRemove: FileRemove,
  FileMakeDir: FileMakeDir,
  FileClose: FileClose,
  WaypointClear: WaypointClear,
  MessageInterval: MessageInterval,
  ParamPush: ParamPush,
  CommandInt: CommandInt,
  WaypointSetCurrent: WaypointSetCurrent,
  CommandTriggerControl: CommandTriggerControl,
  StreamRate: StreamRate,
  ParamPull: ParamPull,
  MountConfigure: MountConfigure,
  ParamSet: ParamSet,
};
