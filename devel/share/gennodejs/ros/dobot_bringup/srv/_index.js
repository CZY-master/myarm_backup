
"use strict";

let RobotMode = require('./RobotMode.js')
let StopScript = require('./StopScript.js')
let ClearError = require('./ClearError.js')
let EnableRobot = require('./EnableRobot.js')
let GetPathStartPose = require('./GetPathStartPose.js')
let Wait = require('./Wait.js')
let MovJExt = require('./MovJExt.js')
let GetAngle = require('./GetAngle.js')
let GetCoils = require('./GetCoils.js')
let GetTerminal485 = require('./GetTerminal485.js')
let RelMovJUser = require('./RelMovJUser.js')
let Sync = require('./Sync.js')
let DO = require('./DO.js')
let JointMovJ = require('./JointMovJ.js')
let AO = require('./AO.js')
let GetPose = require('./GetPose.js')
let GetTraceStartPose = require('./GetTraceStartPose.js')
let StartDrag = require('./StartDrag.js')
let PowerOn = require('./PowerOn.js')
let DIGroup = require('./DIGroup.js')
let RelJointMovJ = require('./RelJointMovJ.js')
let Continues = require('./Continues.js')
let DOExecute = require('./DOExecute.js')
let ServoJ = require('./ServoJ.js')
let GetErrorID = require('./GetErrorID.js')
let TcpDashboard = require('./TcpDashboard.js')
let User = require('./User.js')
let TcpRealData = require('./TcpRealData.js')
let LoadSwitch = require('./LoadSwitch.js')
let ToolDI = require('./ToolDI.js')
let MoveJog = require('./MoveJog.js')
let SetSafeSkin = require('./SetSafeSkin.js')
let GetInRegs = require('./GetInRegs.js')
let RelMovL = require('./RelMovL.js')
let SetCollisionLevel = require('./SetCollisionLevel.js')
let SetArmOrientation = require('./SetArmOrientation.js')
let AI = require('./AI.js')
let Arc = require('./Arc.js')
let SetTerminalKeys = require('./SetTerminalKeys.js')
let RelMovJ = require('./RelMovJ.js')
let SetPayload = require('./SetPayload.js')
let GetHoldRegs = require('./GetHoldRegs.js')
let AccJ = require('./AccJ.js')
let MovJ = require('./MovJ.js')
let MovJIO = require('./MovJIO.js')
let SetObstacleAvoid = require('./SetObstacleAvoid.js')
let StopDrag = require('./StopDrag.js')
let BrakeControl = require('./BrakeControl.js')
let LimZ = require('./LimZ.js')
let ToolDO = require('./ToolDO.js')
let EmergencyStop = require('./EmergencyStop.js')
let StopmoveJog = require('./StopmoveJog.js')
let GetSixForceData = require('./GetSixForceData.js')
let ModbusCreate = require('./ModbusCreate.js')
let CP = require('./CP.js')
let DI = require('./DI.js')
let RunScript = require('./RunScript.js')
let TCPSpeedEnd = require('./TCPSpeedEnd.js')
let RelMovLUser = require('./RelMovLUser.js')
let pause = require('./pause.js')
let SetCoils = require('./SetCoils.js')
let MovLIO = require('./MovLIO.js')
let AOExecute = require('./AOExecute.js')
let SpeedFactor = require('./SpeedFactor.js')
let Circle = require('./Circle.js')
let ServoP = require('./ServoP.js')
let SetCollideDrag = require('./SetCollideDrag.js')
let ContinueScript = require('./ContinueScript.js')
let ServoJParam = require('./ServoJParam.js')
let StartTrace = require('./StartTrace.js')
let MovL = require('./MovL.js')
let SpeedJ = require('./SpeedJ.js')
let HandleTrajPoints = require('./HandleTrajPoints.js')
let SpeedL = require('./SpeedL.js')
let RelMovLTool = require('./RelMovLTool.js')
let PayLoad = require('./PayLoad.js')
let InverseSolution = require('./InverseSolution.js')
let TCPSpeed = require('./TCPSpeed.js')
let PositiveSolution = require('./PositiveSolution.js')
let SyncAll = require('./SyncAll.js')
let ResetRobot = require('./ResetRobot.js')
let ToolAI = require('./ToolAI.js')
let SetTerminal485 = require('./SetTerminal485.js')
let SetHoldRegs = require('./SetHoldRegs.js')
let DOGroup = require('./DOGroup.js')
let PauseScript = require('./PauseScript.js')
let RelMovJTool = require('./RelMovJTool.js')
let ModbusClose = require('./ModbusClose.js')
let StartFCTrace = require('./StartFCTrace.js')
let Jump = require('./Jump.js')
let StartPath = require('./StartPath.js')
let Arch = require('./Arch.js')
let AccL = require('./AccL.js')
let DigitalOutputs = require('./DigitalOutputs.js')
let Tool = require('./Tool.js')
let ToolDOExecute = require('./ToolDOExecute.js')
let GetInBits = require('./GetInBits.js')
let DisableRobot = require('./DisableRobot.js')

module.exports = {
  RobotMode: RobotMode,
  StopScript: StopScript,
  ClearError: ClearError,
  EnableRobot: EnableRobot,
  GetPathStartPose: GetPathStartPose,
  Wait: Wait,
  MovJExt: MovJExt,
  GetAngle: GetAngle,
  GetCoils: GetCoils,
  GetTerminal485: GetTerminal485,
  RelMovJUser: RelMovJUser,
  Sync: Sync,
  DO: DO,
  JointMovJ: JointMovJ,
  AO: AO,
  GetPose: GetPose,
  GetTraceStartPose: GetTraceStartPose,
  StartDrag: StartDrag,
  PowerOn: PowerOn,
  DIGroup: DIGroup,
  RelJointMovJ: RelJointMovJ,
  Continues: Continues,
  DOExecute: DOExecute,
  ServoJ: ServoJ,
  GetErrorID: GetErrorID,
  TcpDashboard: TcpDashboard,
  User: User,
  TcpRealData: TcpRealData,
  LoadSwitch: LoadSwitch,
  ToolDI: ToolDI,
  MoveJog: MoveJog,
  SetSafeSkin: SetSafeSkin,
  GetInRegs: GetInRegs,
  RelMovL: RelMovL,
  SetCollisionLevel: SetCollisionLevel,
  SetArmOrientation: SetArmOrientation,
  AI: AI,
  Arc: Arc,
  SetTerminalKeys: SetTerminalKeys,
  RelMovJ: RelMovJ,
  SetPayload: SetPayload,
  GetHoldRegs: GetHoldRegs,
  AccJ: AccJ,
  MovJ: MovJ,
  MovJIO: MovJIO,
  SetObstacleAvoid: SetObstacleAvoid,
  StopDrag: StopDrag,
  BrakeControl: BrakeControl,
  LimZ: LimZ,
  ToolDO: ToolDO,
  EmergencyStop: EmergencyStop,
  StopmoveJog: StopmoveJog,
  GetSixForceData: GetSixForceData,
  ModbusCreate: ModbusCreate,
  CP: CP,
  DI: DI,
  RunScript: RunScript,
  TCPSpeedEnd: TCPSpeedEnd,
  RelMovLUser: RelMovLUser,
  pause: pause,
  SetCoils: SetCoils,
  MovLIO: MovLIO,
  AOExecute: AOExecute,
  SpeedFactor: SpeedFactor,
  Circle: Circle,
  ServoP: ServoP,
  SetCollideDrag: SetCollideDrag,
  ContinueScript: ContinueScript,
  ServoJParam: ServoJParam,
  StartTrace: StartTrace,
  MovL: MovL,
  SpeedJ: SpeedJ,
  HandleTrajPoints: HandleTrajPoints,
  SpeedL: SpeedL,
  RelMovLTool: RelMovLTool,
  PayLoad: PayLoad,
  InverseSolution: InverseSolution,
  TCPSpeed: TCPSpeed,
  PositiveSolution: PositiveSolution,
  SyncAll: SyncAll,
  ResetRobot: ResetRobot,
  ToolAI: ToolAI,
  SetTerminal485: SetTerminal485,
  SetHoldRegs: SetHoldRegs,
  DOGroup: DOGroup,
  PauseScript: PauseScript,
  RelMovJTool: RelMovJTool,
  ModbusClose: ModbusClose,
  StartFCTrace: StartFCTrace,
  Jump: Jump,
  StartPath: StartPath,
  Arch: Arch,
  AccL: AccL,
  DigitalOutputs: DigitalOutputs,
  Tool: Tool,
  ToolDOExecute: ToolDOExecute,
  GetInBits: GetInBits,
  DisableRobot: DisableRobot,
};
