package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_IDLE_SPEED;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_LEADER_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_MAX_SPEED;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATUS_UPDATE_RATE_HZ;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
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
import static frc.robot.Constants.ShooterConstants.YAW_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.YAW_HOME_ANGLE;
import static frc.robot.Constants.ShooterConstants.YAW_MAGNETIC_OFFSET;
import static frc.robot.Constants.ShooterConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ShooterConstants.YAW_MOTOR_ID;
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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/** Shooter subsystem: turret yaw + pitch + flywheel. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_ID, CANIVORE_BUS);
  private final CANcoder yawEncoder = new CANcoder(YAW_ENCODER_ID, CANIVORE_BUS);

  private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_ID, CANIVORE_BUS);
  private final CANcoder pitchEncoder = new CANcoder(PITCH_ENCODER_ID, CANIVORE_BUS);

  private final TalonFX flywheelLeaderMotor = new TalonFX(FLYWHEEL_LEADER_MOTOR_ID, CANIVORE_BUS);
  private final TalonFX flywheelFollowerMotor = new TalonFX(FLYWHEEL_FOLLOWER_MOTOR_ID, CANIVORE_BUS);

  private final MotionMagicVoltage yawPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final NeutralOut flywheelNeutral = new NeutralOut();

  private final VoltageOut sysIdYawVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdPitchVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC sysIdFlywheelCurrent = new TorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> yawPosition = yawMotor.getPosition();
  private final StatusSignal<AngularVelocity> yawVelocity = yawMotor.getVelocity();
  private final StatusSignal<Angle> pitchPosition = pitchMotor.getPosition();
  private final StatusSignal<AngularVelocity> pitchVelocity = pitchMotor.getVelocity();
  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelLeaderMotor.getVelocity();
  private final StatusSignal<AngularAcceleration> flywheelAcceleration = flywheelLeaderMotor.getAcceleration();

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
          Volts.of(10),
          Seconds.of(10),
          state -> SignalLogger.writeString("Flywheel SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          // SysIdRoutine uses a Voltage parameter, but here we intentionally interpret it as current (amps).
          amps -> flywheelLeaderMotor.setControl(sysIdFlywheelCurrent.withOutput(amps.in(Volts))),
          null,
          this));

  public ShooterSubsystem() {
    configureYaw();
    configurePitch();
    configureFlywheels();
    configureStatusSignals();
  }

  private void configureYaw() {
    CANcoderConfiguration yawCanCoderConfig = new CANcoderConfiguration();
    yawCanCoderConfig.MagnetSensor.withMagnetOffset(YAW_MAGNETIC_OFFSET)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    yawEncoder.getConfigurator().apply(yawCanCoderConfig);

    TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.withNeutralMode(Brake);

    yawTalonConfig.CurrentLimits.withSupplyCurrentLimit(YAW_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(YAW_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);

    yawTalonConfig.Feedback.withRotorToSensorRatio(YAW_ROTOR_TO_SENSOR_RATIO)
        .withFeedbackRemoteSensorID(yawEncoder.getDeviceID())
        .withFeedbackSensorSource(FusedCANcoder);

    yawTalonConfig.withSlot0(Slot0Configs.from(YAW_SLOT_CONFIGS));
    yawTalonConfig.withMotionMagic(YAW_MOTION_MAGIC_CONFIGS);

    yawTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(YAW_SOFT_LIMIT_FORWARD)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(YAW_SOFT_LIMIT_REVERSE);

    yawMotor.getConfigurator().apply(yawTalonConfig);
  }

  private void configurePitch() {
    CANcoderConfiguration pitchCanCoderConfig = new CANcoderConfiguration();
    pitchCanCoderConfig.MagnetSensor.withMagnetOffset(PITCH_MAGNETIC_OFFSET)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    TalonFXConfiguration pitchTalonConfig = new TalonFXConfiguration();
    pitchTalonConfig.MotorOutput.withNeutralMode(Brake);

    pitchTalonConfig.CurrentLimits.withStatorCurrentLimit(PITCH_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(PITCH_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);

    pitchTalonConfig.Feedback.withRotorToSensorRatio(PITCH_ROTOR_TO_SENSOR_RATIO)
        .withFeedbackRemoteSensorID(pitchEncoder.getDeviceID())
        .withFeedbackSensorSource(FusedCANcoder);

    pitchTalonConfig.withSlot0(Slot0Configs.from(PITCH_SLOT_CONFIGS));
    pitchTalonConfig.withMotionMagic(PITCH_MOTION_MAGIC_CONFIGS);

    pitchTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(PITCH_SOFT_LIMIT_FORWARD)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(PITCH_SOFT_LIMIT_REVERSE);

    pitchMotor.getConfigurator().apply(pitchTalonConfig);
  }

  private void configureFlywheels() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput.withNeutralMode(Coast);
    flywheelConfig.CurrentLimits.withStatorCurrentLimit(FLYWHEEL_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    flywheelConfig.withSlot0(Slot0Configs.from(FLYWHEEL_SLOT_CONFIGS));

    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelLeaderMotor.getConfigurator().apply(flywheelConfig);

    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelFollowerMotor.getConfigurator().apply(flywheelConfig);

    flywheelFollowerMotor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  private void configureStatusSignals() {
    yawPosition.setUpdateFrequency(YAW_STATUS_UPDATE_RATE_HZ);
    yawVelocity.setUpdateFrequency(YAW_STATUS_UPDATE_RATE_HZ);
    pitchPosition.setUpdateFrequency(PITCH_STATUS_UPDATE_RATE_HZ);
    pitchVelocity.setUpdateFrequency(PITCH_STATUS_UPDATE_RATE_HZ);
    flywheelVelocity.setUpdateFrequency(FLYWHEEL_STATUS_UPDATE_RATE_HZ);
    flywheelAcceleration.setUpdateFrequency(FLYWHEEL_STATUS_UPDATE_RATE_HZ);

    yawMotor.optimizeBusUtilization();
    pitchMotor.optimizeBusUtilization();
    flywheelLeaderMotor.optimizeBusUtilization();
    flywheelFollowerMotor.optimizeBusUtilization();
  }

  public Command sysIdYawQuasistaticCommand(Direction direction) {
    return yawSysIdRoutine.quasistatic(direction).withName("SysId yaw quasi " + direction).finallyDo(this::stopYaw);
  }

  public Command sysIdYawDynamicCommand(Direction direction) {
    return yawSysIdRoutine.dynamic(direction).withName("SysId yaw dynamic " + direction).finallyDo(this::stopYaw);
  }

  public Command sysIdPitchQuasistaticCommand(Direction direction) {
    return pitchSysIdRoutine.quasistatic(direction)
        .withName("SysId pitch quasi " + direction)
        .finallyDo(this::stopPitch);
  }

  public Command sysIdPitchDynamicCommand(Direction direction) {
    return pitchSysIdRoutine.dynamic(direction).withName("SysId pitch dynamic " + direction).finallyDo(this::stopPitch);
  }

  public Command sysIdFlywheelQuasistaticCommand(Direction direction) {
    return flywheelSysIdRoutine.quasistatic(direction)
        .withName("SysId flywheel quasi " + direction)
        .finallyDo(this::stopFlywheel);
  }

  public Command sysIdFlywheelDynamicCommand(Direction direction) {
    return flywheelSysIdRoutine.dynamic(direction)
        .withName("SysId flywheel dynamic " + direction)
        .finallyDo(this::stopFlywheel);
  }

  @Override
  public void periodic() {
    BaseStatusSignal
        .refreshAll(yawPosition, yawVelocity, pitchPosition, pitchVelocity, flywheelVelocity, flywheelAcceleration);
  }

  /**
   * Commands yaw to the nearest legal equivalent of the requested heading.
   *
   * <p>Input heading is treated as wrapped (0 to 1 rotation), but the commanded target remains
   * continuous so the turret can use multiple turns while obeying soft limits.
   *
   * @param targetYaw requested wrapped heading
   */
  public void setYawAngle(Angle targetYaw) {
    double currentRot = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    double wrappedDesiredRot = ShooterMath.wrapToUnitRotation(targetYaw.in(Rotations));
    double targetRot = ShooterMath.chooseYawShortestDistance(
        wrappedDesiredRot,
          currentRot,
          YAW_SOFT_LIMIT_REVERSE.in(Rotations),
          YAW_SOFT_LIMIT_FORWARD.in(Rotations));

    yawMotor.setControl(yawPositionRequest.withPosition(targetRot));
  }

  /**
   * Commands pitch angle clamped to configured software limits.
   *
   * @param targetPitch requested pitch angle
   */
  public void setPitchAngle(Angle targetPitch) {
    double targetRot = MathUtil.clamp(
        targetPitch.in(Rotations),
          PITCH_SOFT_LIMIT_REVERSE.in(Rotations),
          PITCH_SOFT_LIMIT_FORWARD.in(Rotations));

    pitchMotor.setControl(pitchPositionRequest.withPosition(targetRot));
  }

  /**
   * Commands flywheel speed on the leader motor, clamped to legal range.
   *
   * <p>The follower motor is configured once in follower mode and is not directly commanded here.
   *
   * @param targetSpeed desired flywheel speed
   */
  public void setFlywheelSpeed(AngularVelocity targetSpeed) {
    double targetSpeedRps = MathUtil
        .clamp(targetSpeed.in(RotationsPerSecond), 0.0, FLYWHEEL_MAX_SPEED.in(RotationsPerSecond));

    flywheelLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(targetSpeedRps));
  }

  /**
   * Applies yaw, pitch, and flywheel setpoints as a single command bundle.
   *
   * @param target grouped shooter setpoints
   */
  public void applySetpoints(ShooterTarget target) {
    setYawAngle(target.targetYaw());
    setPitchAngle(target.targetPitch());
    setFlywheelSpeed(target.targetFlywheelSpeed());
  }

  /** Parks yaw/pitch and then applies idle flywheel behavior. */
  public void stow() {
    setYawAngle(YAW_HOME_ANGLE);
    setPitchAngle(PITCH_HOME_ANGLE);
    idleFlywheel();
  }

  private void idleFlywheel() {
    AngularVelocity currentSpeed = BaseStatusSignal.getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
    if (currentSpeed.gt(FLYWHEEL_IDLE_SPEED)) {
      flywheelLeaderMotor.setControl(flywheelNeutral);
      return;
    }

    setFlywheelSpeed(FLYWHEEL_IDLE_SPEED);
  }

  public void stopYaw() {
    yawMotor.stopMotor();
  }

  public void stopPitch() {
    pitchMotor.stopMotor();
  }

  public void stopFlywheel() {
    flywheelLeaderMotor.stopMotor();
  }

  public void stopAll() {
    stopYaw();
    stopPitch();
    stopFlywheel();
  }

  @Logged(name = "Turret Yaw Angle")
  public Angle getYawAngle() {
    double rot = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity).in(Rotations);
    return Rotations.of(ShooterMath.wrapToUnitRotation(rot));
  }

  @Logged(name = "Turret Yaw Continuous Angle")
  public Angle getYawContinuousAngle() {
    return BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
  }

  @Logged(name = "Turret Pitch Angle")
  public Angle getPitchAngle() {
    return BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
  }

  @Logged(name = "Flywheel Speed")
  public AngularVelocity getFlywheelSpeed() {
    return BaseStatusSignal.getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
  }

  /** Returns whether current yaw is within tolerance of the active yaw request. */
  public boolean isYawAtSetpoint() {
    Angle currentYaw = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return currentYaw.isNear(Rotations.of(yawPositionRequest.Position), YAW_POSITION_TOLERANCE);
  }

  public boolean isPitchAtSetpoint() {
    Angle currentPitch = BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
    return currentPitch.isNear(Rotations.of(pitchPositionRequest.Position), PITCH_POSITION_TOLERANCE);
  }

  public boolean isFlywheelAtSpeed() {
    AngularVelocity currentSpeed = BaseStatusSignal.getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
    return currentSpeed.isNear(RotationsPerSecond.of(flywheelVelocityRequest.Velocity), FLYWHEEL_VELOCITY_TOLERANCE);
  }

  /** Returns a single-cycle readiness snapshot without any internal debouncing/filtering. */
  @Logged(name = "Shooter Ready Snapshot")
  public boolean isReadyToShoot() {
    return isYawAtSetpoint() && isPitchAtSetpoint() && isFlywheelAtSpeed();
  }

  /**
   * Input target bundle for the shooter subsystem.
   * `targetYaw` is expected as a wrapped heading in [0, 1) rotations.
   */
  public static record ShooterTarget(Angle targetYaw, Angle targetPitch, AngularVelocity targetFlywheelSpeed) {
    public ShooterTarget {
      if (targetYaw == null) {
        targetYaw = YAW_HOME_ANGLE;
      }
      if (targetPitch == null) {
        targetPitch = PITCH_HOME_ANGLE;
      }
      if (targetFlywheelSpeed == null) {
        targetFlywheelSpeed = FLYWHEEL_IDLE_SPEED;
      }
    }
  }
}
