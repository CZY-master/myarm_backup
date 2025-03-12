
"use strict";

let RobotMode = require('./RobotMode.js')
let DragSensivity = require('./DragSensivity.js')
let ClearError = require('./ClearError.js')
let EnableRobot = require('./EnableRobot.js')
let GetAngle = require('./GetAngle.js')
let SetOutputBool = require('./SetOutputBool.js')
let SetOutputFloat = require('./SetOutputFloat.js')
let ToolDOInstant = require('./ToolDOInstant.js')
let GetCoils = require('./GetCoils.js')
let RelMovJUser = require('./RelMovJUser.js')
let GetDO = require('./GetDO.js')
let DO = require('./DO.js')
let AO = require('./AO.js')
let AOInstant = require('./AOInstant.js')
let GetAO = require('./GetAO.js')
let GetPose = require('./GetPose.js')
let StartDrag = require('./StartDrag.js')
let PowerOn = require('./PowerOn.js')
let SetToolMode = require('./SetToolMode.js')
let DIGroup = require('./DIGroup.js')
let RelJointMovJ = require('./RelJointMovJ.js')
let StopMoveJog = require('./StopMoveJog.js')
let ServoJ = require('./ServoJ.js')
let GetErrorID = require('./GetErrorID.js')
let TcpDashboard = require('./TcpDashboard.js')
let VelL = require('./VelL.js')
let User = require('./User.js')
let DOInstant = require('./DOInstant.js')
let ToolDI = require('./ToolDI.js')
let MoveJog = require('./MoveJog.js')
let GetOutputBool = require('./GetOutputBool.js')
let SetSafeSkin = require('./SetSafeSkin.js')
let GetInRegs = require('./GetInRegs.js')
let SetCollisionLevel = require('./SetCollisionLevel.js')
let AI = require('./AI.js')
let Arc = require('./Arc.js')
let SetPayload = require('./SetPayload.js')
let SetPostCollisionMode = require('./SetPostCollisionMode.js')
let GetHoldRegs = require('./GetHoldRegs.js')
let AccJ = require('./AccJ.js')
let MovJ = require('./MovJ.js')
let Stop = require('./Stop.js')
let MovJIO = require('./MovJIO.js')
let StopDrag = require('./StopDrag.js')
let BrakeControl = require('./BrakeControl.js')
let GetOutputFloat = require('./GetOutputFloat.js')
let SetTool = require('./SetTool.js')
let Pause = require('./Pause.js')
let ToolDO = require('./ToolDO.js')
let Continue = require('./Continue.js')
let EmergencyStop = require('./EmergencyStop.js')
let SetToolPower = require('./SetToolPower.js')
let SetSafeWallEnable = require('./SetSafeWallEnable.js')
let GetInputBool = require('./GetInputBool.js')
let CalcTool = require('./CalcTool.js')
let InverseKin = require('./InverseKin.js')
let ModbusCreate = require('./ModbusCreate.js')
let CP = require('./CP.js')
let DI = require('./DI.js')
let RunScript = require('./RunScript.js')
let RelMovLUser = require('./RelMovLUser.js')
let SetCoils = require('./SetCoils.js')
let MovLIO = require('./MovLIO.js')
let SpeedFactor = require('./SpeedFactor.js')
let Circle = require('./Circle.js')
let ServoP = require('./ServoP.js')
let GetInputInt = require('./GetInputInt.js')
let EnableSafeSkin = require('./EnableSafeSkin.js')
let MovL = require('./MovL.js')
let GetInputFloat = require('./GetInputFloat.js')
let GetCurrentCommandId = require('./GetCurrentCommandId.js')
let VelJ = require('./VelJ.js')
let SetUser = require('./SetUser.js')
let RelMovLTool = require('./RelMovLTool.js')
let InverseSolution = require('./InverseSolution.js')
let PositiveKin = require('./PositiveKin.js')
let SetOutputInt = require('./SetOutputInt.js')
let ToolAI = require('./ToolAI.js')
let GetStartPose = require('./GetStartPose.js')
let SetHoldRegs = require('./SetHoldRegs.js')
let DOGroup = require('./DOGroup.js')
let SetBackDistance = require('./SetBackDistance.js')
let CalcUser = require('./CalcUser.js')
let RelMovJTool = require('./RelMovJTool.js')
let ModbusClose = require('./ModbusClose.js')
let GetOutputInt = require('./GetOutputInt.js')
let StartPath = require('./StartPath.js')
let ModbusRTUCreate = require('./ModbusRTUCreate.js')
let AccL = require('./AccL.js')
let Tool = require('./Tool.js')
let SetTool485 = require('./SetTool485.js')
let GetDOGroup = require('./GetDOGroup.js')
let GetInBits = require('./GetInBits.js')
let DisableRobot = require('./DisableRobot.js')

module.exports = {
  RobotMode: RobotMode,
  DragSensivity: DragSensivity,
  ClearError: ClearError,
  EnableRobot: EnableRobot,
  GetAngle: GetAngle,
  SetOutputBool: SetOutputBool,
  SetOutputFloat: SetOutputFloat,
  ToolDOInstant: ToolDOInstant,
  GetCoils: GetCoils,
  RelMovJUser: RelMovJUser,
  GetDO: GetDO,
  DO: DO,
  AO: AO,
  AOInstant: AOInstant,
  GetAO: GetAO,
  GetPose: GetPose,
  StartDrag: StartDrag,
  PowerOn: PowerOn,
  SetToolMode: SetToolMode,
  DIGroup: DIGroup,
  RelJointMovJ: RelJointMovJ,
  StopMoveJog: StopMoveJog,
  ServoJ: ServoJ,
  GetErrorID: GetErrorID,
  TcpDashboard: TcpDashboard,
  VelL: VelL,
  User: User,
  DOInstant: DOInstant,
  ToolDI: ToolDI,
  MoveJog: MoveJog,
  GetOutputBool: GetOutputBool,
  SetSafeSkin: SetSafeSkin,
  GetInRegs: GetInRegs,
  SetCollisionLevel: SetCollisionLevel,
  AI: AI,
  Arc: Arc,
  SetPayload: SetPayload,
  SetPostCollisionMode: SetPostCollisionMode,
  GetHoldRegs: GetHoldRegs,
  AccJ: AccJ,
  MovJ: MovJ,
  Stop: Stop,
  MovJIO: MovJIO,
  StopDrag: StopDrag,
  BrakeControl: BrakeControl,
  GetOutputFloat: GetOutputFloat,
  SetTool: SetTool,
  Pause: Pause,
  ToolDO: ToolDO,
  Continue: Continue,
  EmergencyStop: EmergencyStop,
  SetToolPower: SetToolPower,
  SetSafeWallEnable: SetSafeWallEnable,
  GetInputBool: GetInputBool,
  CalcTool: CalcTool,
  InverseKin: InverseKin,
  ModbusCreate: ModbusCreate,
  CP: CP,
  DI: DI,
  RunScript: RunScript,
  RelMovLUser: RelMovLUser,
  SetCoils: SetCoils,
  MovLIO: MovLIO,
  SpeedFactor: SpeedFactor,
  Circle: Circle,
  ServoP: ServoP,
  GetInputInt: GetInputInt,
  EnableSafeSkin: EnableSafeSkin,
  MovL: MovL,
  GetInputFloat: GetInputFloat,
  GetCurrentCommandId: GetCurrentCommandId,
  VelJ: VelJ,
  SetUser: SetUser,
  RelMovLTool: RelMovLTool,
  InverseSolution: InverseSolution,
  PositiveKin: PositiveKin,
  SetOutputInt: SetOutputInt,
  ToolAI: ToolAI,
  GetStartPose: GetStartPose,
  SetHoldRegs: SetHoldRegs,
  DOGroup: DOGroup,
  SetBackDistance: SetBackDistance,
  CalcUser: CalcUser,
  RelMovJTool: RelMovJTool,
  ModbusClose: ModbusClose,
  GetOutputInt: GetOutputInt,
  StartPath: StartPath,
  ModbusRTUCreate: ModbusRTUCreate,
  AccL: AccL,
  Tool: Tool,
  SetTool485: SetTool485,
  GetDOGroup: GetDOGroup,
  GetInBits: GetInBits,
  DisableRobot: DisableRobot,
};
