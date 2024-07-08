
"use strict";

let HilActuatorControls = require('./HilActuatorControls.js');
let Mavlink = require('./Mavlink.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let HomePosition = require('./HomePosition.js');
let MountControl = require('./MountControl.js');
let Thrust = require('./Thrust.js');
let GPSINPUT = require('./GPSINPUT.js');
let ParamValue = require('./ParamValue.js');
let CellularStatus = require('./CellularStatus.js');
let DebugValue = require('./DebugValue.js');
let HilControls = require('./HilControls.js');
let FileEntry = require('./FileEntry.js');
let RTKBaseline = require('./RTKBaseline.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let ActuatorControl = require('./ActuatorControl.js');
let Tunnel = require('./Tunnel.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let RCIn = require('./RCIn.js');
let ESCStatus = require('./ESCStatus.js');
let TerrainReport = require('./TerrainReport.js');
let PositionTarget = require('./PositionTarget.js');
let ESCInfo = require('./ESCInfo.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let LogEntry = require('./LogEntry.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let RCOut = require('./RCOut.js');
let Waypoint = require('./Waypoint.js');
let ExtendedState = require('./ExtendedState.js');
let Param = require('./Param.js');
let State = require('./State.js');
let Altitude = require('./Altitude.js');
let LandingTarget = require('./LandingTarget.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let CommandCode = require('./CommandCode.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let SysStatus = require('./SysStatus.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let Trajectory = require('./Trajectory.js');
let GPSRTK = require('./GPSRTK.js');
let ManualControl = require('./ManualControl.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let HilSensor = require('./HilSensor.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let WaypointList = require('./WaypointList.js');
let VehicleInfo = require('./VehicleInfo.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let Vibration = require('./Vibration.js');
let StatusText = require('./StatusText.js');
let BatteryStatus = require('./BatteryStatus.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let GPSRAW = require('./GPSRAW.js');
let VFR_HUD = require('./VFR_HUD.js');
let WaypointReached = require('./WaypointReached.js');
let RTCM = require('./RTCM.js');
let RadioStatus = require('./RadioStatus.js');
let HilGPS = require('./HilGPS.js');
let LogData = require('./LogData.js');

module.exports = {
  HilActuatorControls: HilActuatorControls,
  Mavlink: Mavlink,
  ADSBVehicle: ADSBVehicle,
  HilStateQuaternion: HilStateQuaternion,
  HomePosition: HomePosition,
  MountControl: MountControl,
  Thrust: Thrust,
  GPSINPUT: GPSINPUT,
  ParamValue: ParamValue,
  CellularStatus: CellularStatus,
  DebugValue: DebugValue,
  HilControls: HilControls,
  FileEntry: FileEntry,
  RTKBaseline: RTKBaseline,
  CameraImageCaptured: CameraImageCaptured,
  ActuatorControl: ActuatorControl,
  Tunnel: Tunnel,
  ESCTelemetry: ESCTelemetry,
  WheelOdomStamped: WheelOdomStamped,
  AttitudeTarget: AttitudeTarget,
  RCIn: RCIn,
  ESCStatus: ESCStatus,
  TerrainReport: TerrainReport,
  PositionTarget: PositionTarget,
  ESCInfo: ESCInfo,
  PlayTuneV2: PlayTuneV2,
  LogEntry: LogEntry,
  CompanionProcessStatus: CompanionProcessStatus,
  OnboardComputerStatus: OnboardComputerStatus,
  RCOut: RCOut,
  Waypoint: Waypoint,
  ExtendedState: ExtendedState,
  Param: Param,
  State: State,
  Altitude: Altitude,
  LandingTarget: LandingTarget,
  OverrideRCIn: OverrideRCIn,
  ESCTelemetryItem: ESCTelemetryItem,
  CommandCode: CommandCode,
  NavControllerOutput: NavControllerOutput,
  MagnetometerReporter: MagnetometerReporter,
  SysStatus: SysStatus,
  ESCInfoItem: ESCInfoItem,
  TimesyncStatus: TimesyncStatus,
  Trajectory: Trajectory,
  GPSRTK: GPSRTK,
  ManualControl: ManualControl,
  CamIMUStamp: CamIMUStamp,
  HilSensor: HilSensor,
  EstimatorStatus: EstimatorStatus,
  OpticalFlowRad: OpticalFlowRad,
  WaypointList: WaypointList,
  VehicleInfo: VehicleInfo,
  ESCStatusItem: ESCStatusItem,
  Vibration: Vibration,
  StatusText: StatusText,
  BatteryStatus: BatteryStatus,
  GlobalPositionTarget: GlobalPositionTarget,
  GPSRAW: GPSRAW,
  VFR_HUD: VFR_HUD,
  WaypointReached: WaypointReached,
  RTCM: RTCM,
  RadioStatus: RadioStatus,
  HilGPS: HilGPS,
  LogData: LogData,
};
