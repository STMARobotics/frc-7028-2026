package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_IDLE_RPS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_MAX_RPS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATUS_UPDATE_RATE_HZ;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.LEFT_FLYWHEEL_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_HOME_ANGLE;
import static frc.robot.Constants.ShooterConstants.PITCH_MAGNETIC_OFFSET;
import static frc.robot.Constants.ShooterConstants.PITCH_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ShooterConstants.PITCH_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_POSITION_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.PITCH_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ShooterConstants.PITCH_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.PITCH_SOFT_LIMIT_FORWARD;
import static frc.robot.Constants.ShooterConstants.PITCH_SOFT_LIMIT_REVERSE;
import static frc.robot.Constants.ShooterConstants.PITCH_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.PITCH_STATUS_UPDATE_RATE_HZ;
import static frc.robot.Constants.ShooterConstants.PITCH_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.RIGHT_FLYWHEEL_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.YAW_CONTINUOUS_WRAP;
import static frc.robot.Constants.ShooterConstants.YAW_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.YAW_MAGNETIC_OFFSET;
import static frc.robot.Constants.ShooterConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ShooterConstants.YAW_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.YAW_IDLE_CENTER;
import static frc.robot.Constants.ShooterConstants.YAW_IDLE_HALF_RANGE;
import static frc.robot.Constants.ShooterConstants.YAW_POSITION_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.YAW_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ShooterConstants.YAW_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.YAW_SOFT_LIMIT_FORWARD;
import static frc.robot.Constants.ShooterConstants.YAW_SOFT_LIMIT_REVERSE;
import static frc.robot.Constants.ShooterConstants.YAW_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.YAW_STATUS_UPDATE_RATE_HZ;
import static frc.robot.Constants.ShooterConstants.YAW_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * ShooterSubsystem: turret yaw + pitch (hood) + flywheel.
 */
@Logged
public class ShooterSubsystem extends SubsystemBase {
  // TODO: Figure out what the hard limits of the turret are.
  private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_ID, CANIVORE_BUS_NAME);
  private final CANcoder yawEncoder = new CANcoder(YAW_ENCODER_ID, CANIVORE_BUS_NAME);

  private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_ID, CANIVORE_BUS_NAME);
  private final CANcoder pitchEncoder = new CANcoder(PITCH_ENCODER_ID, CANIVORE_BUS_NAME);

  private final TalonFX rightFlywheelMotor = new TalonFX(RIGHT_FLYWHEEL_MOTOR_ID, CANIVORE_BUS_NAME);
  private final TalonFX leftFlywheelMotor = new TalonFX(LEFT_FLYWHEEL_MOTOR_ID, CANIVORE_BUS_NAME);

  private final MotionMagicVoltage yawPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rightFlywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC leftFlywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut flywheelCoastZeroVolts = new VoltageOut(0.0).withEnableFOC(true);

  private final VoltageOut sysIdYawVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdPitchVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdFlywheelVoltage = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> yawPosition = yawMotor.getPosition();
  private final StatusSignal<AngularVelocity> yawVelocity = yawMotor.getVelocity();
  private final StatusSignal<Angle> yawAbsolutePosition = yawEncoder.getAbsolutePosition();

  private final StatusSignal<Angle> pitchPosition = pitchMotor.getPosition();
  private final StatusSignal<AngularVelocity> pitchVelocity = pitchMotor.getVelocity();

  private final StatusSignal<AngularVelocity> rightFlywheelVelocity = rightFlywheelMotor.getVelocity();
  private final StatusSignal<AngularVelocity> leftFlywheelVelocity = leftFlywheelMotor.getVelocity();
  private final StatusSignal<AngularAcceleration> rightFlywheelAcceleration = rightFlywheelMotor.getAcceleration();
  private final StatusSignal<AngularAcceleration> leftFlywheelAcceleration = leftFlywheelMotor.getAcceleration();

  private final MutAngle yawAngle = Radians.mutable(0);
  private final MutAngle yawAngleContinuous = Radians.mutable(0);
  private final MutAngle pitchAngle = Radians.mutable(0);
  private final MutAngularVelocity flywheelRps = Rotations.per(Second).mutable(0);

  @Logged(name = "Yaw Sync OK")
  private boolean yawSyncOk = true;

  @Logged(name = "Yaw Sync Out Of Range")
  private boolean yawSyncOutOfRange = false;

  @Logged(name = "Yaw Abs Rot (0-1)")
  private double yawAbsoluteRotations = 0.0;

  private double yawSetpointRotations = 0.0;
  private double pitchSetpointRotations = 0.0;
  private double flywheelSetpointRps = 0.0;

  private static final double TWO_PI = 2.0 * Math.PI;

  // SysId routines
  private final SysIdRoutine yawSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Yaw Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> yawMotor.setControl(sysIdYawVoltage.withOutput(volts.in(Volts))),
          null,
          this));

  private final SysIdRoutine pitchSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Pitch Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> pitchMotor.setControl(sysIdPitchVoltage.withOutput(volts.in(Volts))),
          null,
          this));

  private final SysIdRoutine flywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Flywheel SysId", state.toString())),
      new SysIdRoutine.Mechanism(volts -> {
        rightFlywheelMotor.setControl(sysIdFlywheelVoltage.withOutput(volts.in(Volts)));
        leftFlywheelMotor.setControl(sysIdFlywheelVoltage.withOutput(volts.in(Volts)));
      }, null, this));

  public ShooterSubsystem() {
    configureYaw();
    configurePitch();
    configureFlywheels();
    configureStatusSignals();
    syncYawToAbsolute();
  }

  // Configuration
  private void configureYaw() {
    var yawCanCoderConfig = new CANcoderConfiguration();
    yawCanCoderConfig.MagnetSensor.withMagnetOffset(YAW_MAGNETIC_OFFSET.in(Rotations))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    yawEncoder.getConfigurator().apply(yawCanCoderConfig);

    var yawTalonConfig = new TalonFXConfiguration();

    yawTalonConfig.MotorOutput.withNeutralMode(Brake);
    yawTalonConfig.ClosedLoopGeneral.ContinuousWrap = YAW_CONTINUOUS_WRAP;

    yawTalonConfig.CurrentLimits.withSupplyCurrentLimit(YAW_SUPPLY_CURRENT_LIMIT.in(Amps))
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(YAW_STATOR_CURRENT_LIMIT.in(Amps))
        .withStatorCurrentLimitEnable(true);

    yawTalonConfig.Feedback.withRotorToSensorRatio(YAW_ROTOR_TO_SENSOR_RATIO)
        .withFeedbackRemoteSensorID(yawEncoder.getDeviceID())
        .withFeedbackSensorSource(FusedCANcoder);

    yawTalonConfig.withSlot0(Slot0Configs.from(YAW_SLOT_CONFIGS));
    yawTalonConfig.withMotionMagic(YAW_MOTION_MAGIC_CONFIGS);

    yawTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(YAW_SOFT_LIMIT_FORWARD.in(Rotations))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(YAW_SOFT_LIMIT_REVERSE.in(Rotations));

    yawMotor.getConfigurator().apply(yawTalonConfig);
  }

  private void configurePitch() {
    var pitchCanCoderConfig = new CANcoderConfiguration();
    pitchCanCoderConfig.MagnetSensor.withMagnetOffset(PITCH_MAGNETIC_OFFSET.in(Rotations))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    var pitchTalonConfig = new TalonFXConfiguration();

    pitchTalonConfig.MotorOutput.withNeutralMode(Brake);
    pitchTalonConfig.ClosedLoopGeneral.ContinuousWrap = false;

    pitchTalonConfig.CurrentLimits.withStatorCurrentLimit(PITCH_STATOR_CURRENT_LIMIT.in(Amps))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(PITCH_SUPPLY_CURRENT_LIMIT.in(Amps))
        .withSupplyCurrentLimitEnable(true);

    pitchTalonConfig.Feedback.withRotorToSensorRatio(PITCH_ROTOR_TO_SENSOR_RATIO)
        .withFeedbackRemoteSensorID(pitchEncoder.getDeviceID())
        .withFeedbackSensorSource(FusedCANcoder);

    pitchTalonConfig.withSlot0(Slot0Configs.from(PITCH_SLOT_CONFIGS));
    pitchTalonConfig.withMotionMagic(PITCH_MOTION_MAGIC_CONFIGS);

    pitchTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(PITCH_SOFT_LIMIT_FORWARD.in(Rotations))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(PITCH_SOFT_LIMIT_REVERSE.in(Rotations));

    pitchMotor.getConfigurator().apply(pitchTalonConfig);
  }

  private void configureFlywheels() {
    // Right flywheel
    var rightFlywheelTalonConfig = new TalonFXConfiguration();
    rightFlywheelTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightFlywheelTalonConfig.MotorOutput.withNeutralMode(Coast);

    rightFlywheelTalonConfig.CurrentLimits.withStatorCurrentLimit(FLYWHEEL_STATOR_CURRENT_LIMIT.in(Amps))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_CURRENT_LIMIT.in(Amps))
        .withSupplyCurrentLimitEnable(true);

    rightFlywheelTalonConfig.ClosedLoopGeneral.ContinuousWrap = false;
    rightFlywheelTalonConfig.withSlot0(Slot0Configs.from(FLYWHEEL_SLOT_CONFIGS));

    rightFlywheelMotor.getConfigurator().apply(rightFlywheelTalonConfig);

    // Left flywheel
    var leftFlywheelTalonConfig = new TalonFXConfiguration();
    leftFlywheelTalonConfig.MotorOutput.withNeutralMode(Coast);

    leftFlywheelTalonConfig.CurrentLimits.withStatorCurrentLimit(FLYWHEEL_STATOR_CURRENT_LIMIT.in(Amps))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_CURRENT_LIMIT.in(Amps))
        .withSupplyCurrentLimitEnable(true);

    leftFlywheelTalonConfig.ClosedLoopGeneral.ContinuousWrap = false;
    leftFlywheelTalonConfig.withSlot0(Slot0Configs.from(FLYWHEEL_SLOT_CONFIGS));

    leftFlywheelMotor.getConfigurator().apply(leftFlywheelTalonConfig);
  }

  private void configureStatusSignals() {
    yawPosition.setUpdateFrequency(YAW_STATUS_UPDATE_RATE_HZ);
    yawVelocity.setUpdateFrequency(YAW_STATUS_UPDATE_RATE_HZ);
    yawAbsolutePosition.setUpdateFrequency(YAW_STATUS_UPDATE_RATE_HZ);
    pitchPosition.setUpdateFrequency(PITCH_STATUS_UPDATE_RATE_HZ);
    pitchVelocity.setUpdateFrequency(PITCH_STATUS_UPDATE_RATE_HZ);

    rightFlywheelVelocity.setUpdateFrequency(FLYWHEEL_STATUS_UPDATE_RATE_HZ);
    leftFlywheelVelocity.setUpdateFrequency(FLYWHEEL_STATUS_UPDATE_RATE_HZ);
    rightFlywheelAcceleration.setUpdateFrequency(FLYWHEEL_STATUS_UPDATE_RATE_HZ);
    leftFlywheelAcceleration.setUpdateFrequency(FLYWHEEL_STATUS_UPDATE_RATE_HZ);

    // This asks Phoenix to pack signals efficiently on the CAN bus.
    yawMotor.optimizeBusUtilization();
    pitchMotor.optimizeBusUtilization();
    rightFlywheelMotor.optimizeBusUtilization();
    leftFlywheelMotor.optimizeBusUtilization();
  }

  // Sync / Calibration
  /**
   * Syncs the yaw motor's continuous position to the absolute CANcoder value, if in range.
   */
  public void syncYawToAbsolute() {
    yawAbsolutePosition.refresh();
    double absoluteRot = yawAbsolutePosition.getValue().in(Rotations);
    yawAbsoluteRotations = wrap0To1(absoluteRot);

    if (!isYawEquivalentWithinLimits(yawAbsoluteRotations)) {
      yawSyncOk = false;
      yawSyncOutOfRange = true;
      return;
    }

    double currentRot = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    double targetRot = chooseYawTargetContinuousRotations(yawAbsoluteRotations, currentRot);
    yawMotor.setPosition(targetRot);

    yawSetpointRotations = targetRot;
    yawSyncOk = true;
    yawSyncOutOfRange = false;
  }

  /**
   * Command to sync yaw to the absolute CANcoder value.
   * 
   * @return command that runs a one-time sync
   */
  public Command syncYawToAbsoluteCommand() {
    return runOnce(this::syncYawToAbsolute).withName("Sync yaw to absolute");
  }

  // SysId
  /**
   * Command to run yaw SysId routine in quasistatic mode.
   * 
   * @param direction
   * @return
   */
  public Command sysIdYawQuasistaticCommand(Direction direction) {
    return yawSysIdRoutine.quasistatic(direction).withName("SysId yaw quasi " + direction).finallyDo(this::stopYaw);
  }

  /**
   * Command to run yaw SysId routine in dynamic mode.
   * 
   * @param direction
   * @return
   */
  public Command sysIdYawDynamicCommand(Direction direction) {
    return yawSysIdRoutine.dynamic(direction).withName("SysId yaw dynamic " + direction).finallyDo(this::stopYaw);
  }

  /**
   * Command to run pitch SysId routine in quasistatic mode.
   * 
   * @param direction
   * @return
   */
  public Command sysIdPitchQuasistaticCommand(Direction direction) {
    return pitchSysIdRoutine.quasistatic(direction)
        .withName("SysId pitch quasi " + direction)
        .finallyDo(this::stopPitch);
  }

  /**
   * Command to run pitch SysId routine in dynamic mode.
   * 
   * @param direction
   * @return
   */
  public Command sysIdPitchDynamicCommand(Direction direction) {
    return pitchSysIdRoutine.dynamic(direction).withName("SysId pitch dynamic " + direction).finallyDo(this::stopPitch);
  }

  /**
   * Command to run flywheel SysId routine in quasistatic mode.
   * 
   * @param direction
   * @return
   */
  public Command sysIdFlywheelQuasistaticCommand(Direction direction) {
    return flywheelSysIdRoutine.quasistatic(direction)
        .withName("SysId flywheel quasi " + direction)
        .finallyDo(this::stopFlywheel);
  }

  /**
   * Command to run flywheel SysId routine in dynamic mode.
   * Runs both flywheel motors together.
   * 
   * @param direction
   * @return
   */
  public Command sysIdFlywheelDynamicCommand(Direction direction) {
    return flywheelSysIdRoutine.dynamic(direction)
        .withName("SysId flywheel dynamic " + direction)
        .finallyDo(this::stopFlywheel);
  }

  // Periodic / status signal refresh
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        yawPosition,
          yawVelocity,
          pitchPosition,
          pitchVelocity,
          rightFlywheelVelocity,
          leftFlywheelVelocity,
          rightFlywheelAcceleration,
          leftFlywheelAcceleration);
  }

  // Setpoints
  /**
   * Sets the turret's yaw angle in radians.
   * Expected range: [YAW_SOFT_LIMIT_REVERSE, YAW_SOFT_LIMIT_FORWARD] in rotations, converted from radians.
   * 
   * @param yawRadians The desired yaw angle in radians.
   */
  public void setYawAngleRadians(Angle yawRadians) {
    double currentRot = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    double desiredRot0To1 = wrap0To1(yawRadians.in(Rotations));
    double targetRot = chooseYawTargetContinuousRotations(desiredRot0To1, currentRot);
    yawSetpointRotations = targetRot;
    yawMotor.setControl(yawPositionRequest.withPosition(targetRot));
  }

  /**
   * Sets the turret's yaw angle in radians.
   * Convenience overload when the input is a raw radians value.
   * 
   * @param yawRadians The desired yaw angle in radians.
   */
  public void setYawAngleRadians(double yawRadians) {
    setYawAngleRadians(Radians.of(yawRadians));
  }

  /**
   * Sets the turret's yaw angle in continuous radians.
   * Expected range: [YAW_SOFT_LIMIT_REVERSE, YAW_SOFT_LIMIT_FORWARD] in rotations, converted from radians.
   * 
   * @param yawRadians The desired yaw angle in continuous radians.
   */
  public void setYawAngleContinuousRadians(Angle yawRadians) {
    double targetRot = clamp(
        yawRadians.in(Rotations),
          YAW_SOFT_LIMIT_REVERSE.in(Rotations),
          YAW_SOFT_LIMIT_FORWARD.in(Rotations));
    yawSetpointRotations = targetRot;
    yawMotor.setControl(yawPositionRequest.withPosition(targetRot));
  }

  /**
   * Sets the turret's yaw angle in continuous radians.
   * Convenience overload when the input is a raw radians value.
   * 
   * @param yawRadians The desired yaw angle in continuous radians.
   */
  public void setYawAngleContinuousRadians(double yawRadians) {
    setYawAngleContinuousRadians(Radians.of(yawRadians));
  }

  /**
   * Sets the hood's pitch angle in radians.
   * Expected range: [PITCH_SOFT_LIMIT_REVERSE, PITCH_SOFT_LIMIT_FORWARD] in rotations, converted from radians.
   * 
   * @param pitchRadians The desired pitch angle in radians.
   */
  public void setPitchAngleRadians(Angle pitchRadians) {
    double targetRot = clamp(
        pitchRadians.in(Rotations),
          PITCH_SOFT_LIMIT_REVERSE.in(Rotations),
          PITCH_SOFT_LIMIT_FORWARD.in(Rotations));
    pitchSetpointRotations = targetRot;
    pitchMotor.setControl(pitchPositionRequest.withPosition(targetRot));
  }

  /**
   * Sets the hood's pitch angle in radians.
   * Convenience overload when the input is a raw radians value.
   * 
   * @param pitchRadians The desired pitch angle in radians.
   */
  public void setPitchAngleRadians(double pitchRadians) {
    setPitchAngleRadians(Radians.of(pitchRadians));
  }

  /**
   * Sets the flywheel velocity (both motors) in rotations per second (RPS).
   * Expected range: [0, FLYWHEEL_MAX_RPS].
   * 
   * @param rps The desired flywheel velocity in RPS.
   */
  private void setFlywheelVelocity(AngularVelocity rps) {
    double targetRps = clamp(rps.in(Rotations.per(Second)), 0.0, FLYWHEEL_MAX_RPS.in(Rotations.per(Second)));
    flywheelSetpointRps = targetRps;
    rightFlywheelMotor.setControl(rightFlywheelVelocityRequest.withVelocity(Rotations.per(Second).of(targetRps)));
    leftFlywheelMotor.setControl(leftFlywheelVelocityRequest.withVelocity(Rotations.per(Second).of(targetRps)));
  }

  /**
   * Sets the flywheel velocity (both motors) in rotations per second (RPS).
   * 
   * @param rps The desired flywheel velocity in RPS.
   */
  public void setFlywheelRps(AngularVelocity rps) {
    setFlywheelVelocity(rps);
  }

  /**
   * Sets the flywheel velocity (both motors) in rotations per second (RPS).
   * Convenience overload when the input is a raw RPS value.
   * 
   * @param rps The desired flywheel velocity in RPS.
   */
  public void setFlywheelRps(double rps) {
    setFlywheelVelocity(Rotations.per(Second).of(rps));
  }

  // Idle / Stop
  /**
   * Moves yaw to the idle center.
   */
  public void idleYaw() {
    setYawAngleRadians(YAW_IDLE_CENTER);
  }

  /**
   * Moves pitch to the idle/home angle.
   */
  public void idlePitch() {
    setPitchAngleRadians(PITCH_HOME_ANGLE);
  }

  /**
   * Idles the flywheel motors.
   */
  public void idleFlywheel() {
    final double right = BaseStatusSignal.getLatencyCompensatedValue(rightFlywheelVelocity, rightFlywheelAcceleration)
        .in(Rotations.per(Second));
    final double left = BaseStatusSignal.getLatencyCompensatedValue(leftFlywheelVelocity, leftFlywheelAcceleration)
        .in(Rotations.per(Second));
    final double currentAvgRps = (right + left) / 2.0;

    final double idleRps = FLYWHEEL_IDLE_RPS.in(Rotations.per(Second));

    if (currentAvgRps > idleRps) {
      rightFlywheelMotor.setControl(flywheelCoastZeroVolts);
      leftFlywheelMotor.setControl(flywheelCoastZeroVolts);
      return;
    }

    setFlywheelVelocity(FLYWHEEL_IDLE_RPS);
  }

  // This is commented out because the SOTMvalues datatype doesnt exist in the branch yet.
  // /**
  // * Starts the auto-targeting routine.
  // *
  // * @param SOTMvalues Custom datatype containing values from the shoot on the move utility.
  // */
  // public void startAutoTargeting(SOTMvalues SOTMvalues) {
  // setYawAngleRadians(SOTMvalues.getYawAngle());
  // setPitchAngleRadians(SOTMvalues.getPitchAngle());
  // setFlywheelVelocity(SOTMvalues.getFlywheelRPS());
  // }

  public void stopAutoTargeting() {
    stopYaw();
    stopPitch();
    idleFlywheel();
  }

  /**
   * Stops the yaw motor.
   */
  public void stopYaw() {
    yawMotor.stopMotor();
  }

  /**
   * Stops the pitch motor.
   */
  public void stopPitch() {
    pitchMotor.stopMotor();
  }

  /**
   * Stops the flywheel motors.
   */
  public void stopFlywheel() {
    rightFlywheelMotor.stopMotor();
    leftFlywheelMotor.stopMotor();
    flywheelSetpointRps = 0.0;
  }

  /**
   * Stops all shooter mechanisms.
   */
  public void stopAll() {
    stopYaw();
    stopPitch();
    stopFlywheel();
  }

  // Telemetry / Getters
  /**
   * Gets the turret yaw angle in radians.
   * 
   * @return Returns wrapped heading [0, 2*pi) radians, not continuous yaw.
   */
  @Logged(name = "Turret Yaw Angle (rad)")
  public Angle getYawAngleRadians() {
    double rot = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    return yawAngle.mut_replace(rotationsToRadians(wrap0To1(rot)), Radians);
  }

  /**
   * Gets the turret yaw angle in continuous radians.
   * 
   * @return Returns continuous yaw in radians.
   */
  public Angle getYawContinuousRadians() {
    double rot = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    return yawAngleContinuous.mut_replace(rotationsToRadians(rot), Radians);
  }

  /**
   * Gets the turret yaw angle in continuous rotations.
   * 
   * @return Returns continuous yaw in rotations.
   */
  public double getYawContinuousRotations() {
    return BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
  }

  /**
   * Gets the hood pitch angle in radians.
   * 
   * @return The pitch angle in radians.
   */
  @Logged(name = "Turret Pitch Angle (rad)")
  public Angle getPitchAngleRadians() {
    double rot = BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity).in(Rotations);
    return pitchAngle.mut_replace(rotationsToRadians(rot), Radians);
  }

  /**
   * Gets the average flywheel velocity between both motors in rotations per second (RPS).
   * 
   * @return The flywheel velocity in RPS.
   */
  @Logged(name = "Flywheel Velocity (RPS)")
  public AngularVelocity getFlywheelRPS() {
    double right = BaseStatusSignal.getLatencyCompensatedValue(rightFlywheelVelocity, rightFlywheelAcceleration)
        .in(Rotations.per(Second));
    double left = BaseStatusSignal.getLatencyCompensatedValue(leftFlywheelVelocity, leftFlywheelAcceleration)
        .in(Rotations.per(Second));
    return flywheelRps.mut_replace((right + left) / 2.0, Rotations.per(Second));
  }

  // Setpoint Checks
  /**
   * Checks if the yaw is at its last commanded setpoint.
   * 
   * @return True if yaw is within tolerance.
   */
  public boolean isYawAtSetpoint() {
    double current = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    return Math.abs(current - yawSetpointRotations) <= YAW_POSITION_TOLERANCE.in(Rotations);
  }

  /**
   * Checks if the pitch is at its last commanded setpoint.
   * 
   * @return True if pitch is within tolerance.
   */
  public boolean isPitchAtSetpoint() {
    double current = BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity).in(Rotations);
    return Math.abs(current - pitchSetpointRotations) <= PITCH_POSITION_TOLERANCE.in(Rotations);
  }

  /**
   * Checks if the flywheel is at its last commanded speed.
   * 
   * @return True if flywheel is within tolerance.
   */
  public boolean isFlywheelAtSpeed() {
    double right = BaseStatusSignal.getLatencyCompensatedValue(rightFlywheelVelocity, rightFlywheelAcceleration)
        .in(Rotations.per(Second));
    double left = BaseStatusSignal.getLatencyCompensatedValue(leftFlywheelVelocity, leftFlywheelAcceleration)
        .in(Rotations.per(Second));
    double current = (right + left) / 2.0;
    return Math.abs(current - flywheelSetpointRps) <= FLYWHEEL_VELOCITY_TOLERANCE.in(Rotations.per(Second));
  }

  /**
   * Checks if yaw, pitch, and flywheel are at their setpoints.
   * 
   * @return True if all shooter setpoints are met.
   */
  public boolean isShooterReady() {
    return isYawAtSetpoint() && isPitchAtSetpoint() && isFlywheelAtSpeed();
  }

  // Helpers
  /**
   * Checks if the yaw is within the idle range.
   * Idle range is defined as a window around {@code YAW_IDLE_CENTER}.
   * 
   * @return True if yaw is within idle range.
   */
  private boolean isYawInIdleRange() {
    double current = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    double center = wrap0To1(YAW_IDLE_CENTER.in(Rotations));
    double delta = wrapToMinusHalfToHalf(current - center);
    return Math.abs(delta) <= YAW_IDLE_HALF_RANGE.in(Rotations);
  }

  /**
   * Chooses the best equivalent yaw target angle within soft limits, minimizing rotation.
   * Use this for >360 deg capable turrets. TODO: Confirm this, Osowski mentioned it was 390?
   * 
   * @param desiredHeading0To1Rot The desired heading in [0, 1) rotations.
   * @param currentYawContinuousRot The current yaw angle in continuous rotations.
   * @return The chosen yaw target angle.
   */
  private double chooseYawTargetContinuousRotations(double desiredHeading0To1Rot, double currentYawContinuousRot) {
    final double desired = wrap0To1(desiredHeading0To1Rot);
    final double current = currentYawContinuousRot;

    final double min = YAW_SOFT_LIMIT_REVERSE.in(Rotations);
    final double max = YAW_SOFT_LIMIT_FORWARD.in(Rotations);

    boolean found = false;
    double best = 0.0;
    double bestDist = Double.POSITIVE_INFINITY;

    for (int k = -2; k <= 2; k++) {
      double cand = desired + k;
      if (cand < min || cand > max) {
        continue;
      }
      double dist = Math.abs(cand - current);
      if (dist < bestDist) {
        bestDist = dist;
        best = cand;
        found = true;
      }
    }

    if (!found) {
      double nearest = desired + Math.rint(current - desired);
      best = clamp(nearest, min, max);
    }

    return best;
  }

  /**
   * Checks if any equivalent yaw target exists within soft limits.
   * 
   * @param desiredHeading0To1Rot The desired heading in [0, 1) rotations.
   * @return True if a valid equivalent exists within soft limits.
   */
  private boolean isYawEquivalentWithinLimits(double desiredHeading0To1Rot) {
    final double desired = wrap0To1(desiredHeading0To1Rot);
    final double min = YAW_SOFT_LIMIT_REVERSE.in(Rotations);
    final double max = YAW_SOFT_LIMIT_FORWARD.in(Rotations);

    for (int k = -2; k <= 2; k++) {
      double cand = desired + k;
      if (cand >= min && cand <= max) {
        return true;
      }
    }

    return false;
  }

  /**
   * Wraps a rotation value to the range [0, 1).
   * 
   * @param rotations The input rotation value.
   * @return The wrapped rotation value.
   */
  private static double wrap0To1(double rotations) {
    double r = rotations - Math.floor(rotations);
    if (r >= 1.0) {
      r = 0.0;
    }
    if (r < 0.0) {
      r += 1.0;
    }
    return r;
  }

  private static double wrapToMinusHalfToHalf(double rotations) {
    return rotations - Math.rint(rotations);
  }

  private static double rotationsToRadians(double rotations) {
    return rotations * TWO_PI;
  }

  /**
   * Clamps a value between min and max.
   * 
   * @param value The value to clamp.
   * @param min The minimum value.
   * @param max The maximum value.
   * @return The clamped value.
   */
  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
