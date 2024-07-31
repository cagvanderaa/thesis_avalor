
"use strict";

let OverrideRCIn = require('./OverrideRCIn.js');
let ExtendedState = require('./ExtendedState.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let HilSensor = require('./HilSensor.js');
let ManualControl = require('./ManualControl.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let ActuatorControl = require('./ActuatorControl.js');
let Trajectory = require('./Trajectory.js');
let BatteryStatus = require('./BatteryStatus.js');
let StatusText = require('./StatusText.js');
let VehicleInfo = require('./VehicleInfo.js');
let Altitude = require('./Altitude.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let Vibration = require('./Vibration.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let HilControls = require('./HilControls.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let ESCStatus = require('./ESCStatus.js');
let GPSRAW = require('./GPSRAW.js');
let SysStatus = require('./SysStatus.js');
let ParamValue = require('./ParamValue.js');
let PositionTarget = require('./PositionTarget.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let HilGPS = require('./HilGPS.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let GPSRTK = require('./GPSRTK.js');
let RCIn = require('./RCIn.js');
let MountControl = require('./MountControl.js');
let HomePosition = require('./HomePosition.js');
let WaypointReached = require('./WaypointReached.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let ESCInfo = require('./ESCInfo.js');
let RCOut = require('./RCOut.js');
let LandingTarget = require('./LandingTarget.js');
let Param = require('./Param.js');
let FileEntry = require('./FileEntry.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let LogEntry = require('./LogEntry.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let DebugValue = require('./DebugValue.js');
let RTKBaseline = require('./RTKBaseline.js');
let TerrainReport = require('./TerrainReport.js');
let RadioStatus = require('./RadioStatus.js');
let Mavlink = require('./Mavlink.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let CommandCode = require('./CommandCode.js');
let GPSINPUT = require('./GPSINPUT.js');
let Tunnel = require('./Tunnel.js');
let Waypoint = require('./Waypoint.js');
let State = require('./State.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let VFR_HUD = require('./VFR_HUD.js');
let RTCM = require('./RTCM.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let Thrust = require('./Thrust.js');
let WaypointList = require('./WaypointList.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let CellularStatus = require('./CellularStatus.js');
let LogData = require('./LogData.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');

module.exports = {
  OverrideRCIn: OverrideRCIn,
  ExtendedState: ExtendedState,
  EstimatorStatus: EstimatorStatus,
  ADSBVehicle: ADSBVehicle,
  MagnetometerReporter: MagnetometerReporter,
  HilSensor: HilSensor,
  ManualControl: ManualControl,
  HilActuatorControls: HilActuatorControls,
  ActuatorControl: ActuatorControl,
  Trajectory: Trajectory,
  BatteryStatus: BatteryStatus,
  StatusText: StatusText,
  VehicleInfo: VehicleInfo,
  Altitude: Altitude,
  CamIMUStamp: CamIMUStamp,
  Vibration: Vibration,
  PlayTuneV2: PlayTuneV2,
  HilControls: HilControls,
  ESCTelemetryItem: ESCTelemetryItem,
  ESCStatusItem: ESCStatusItem,
  ESCStatus: ESCStatus,
  GPSRAW: GPSRAW,
  SysStatus: SysStatus,
  ParamValue: ParamValue,
  PositionTarget: PositionTarget,
  HilStateQuaternion: HilStateQuaternion,
  HilGPS: HilGPS,
  CameraImageCaptured: CameraImageCaptured,
  GPSRTK: GPSRTK,
  RCIn: RCIn,
  MountControl: MountControl,
  HomePosition: HomePosition,
  WaypointReached: WaypointReached,
  OnboardComputerStatus: OnboardComputerStatus,
  ESCInfo: ESCInfo,
  RCOut: RCOut,
  LandingTarget: LandingTarget,
  Param: Param,
  FileEntry: FileEntry,
  ESCTelemetry: ESCTelemetry,
  TimesyncStatus: TimesyncStatus,
  LogEntry: LogEntry,
  CompanionProcessStatus: CompanionProcessStatus,
  DebugValue: DebugValue,
  RTKBaseline: RTKBaseline,
  TerrainReport: TerrainReport,
  RadioStatus: RadioStatus,
  Mavlink: Mavlink,
  WheelOdomStamped: WheelOdomStamped,
  CommandCode: CommandCode,
  GPSINPUT: GPSINPUT,
  Tunnel: Tunnel,
  Waypoint: Waypoint,
  State: State,
  AttitudeTarget: AttitudeTarget,
  VFR_HUD: VFR_HUD,
  RTCM: RTCM,
  NavControllerOutput: NavControllerOutput,
  Thrust: Thrust,
  WaypointList: WaypointList,
  ESCInfoItem: ESCInfoItem,
  CellularStatus: CellularStatus,
  LogData: LogData,
  GlobalPositionTarget: GlobalPositionTarget,
  OpticalFlowRad: OpticalFlowRad,
};
