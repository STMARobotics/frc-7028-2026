package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.LEFT_FLYWHEEL_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_LIMIT_FORWARD;
import static frc.robot.Constants.ShooterConstants.PITCH_LIMIT_REVERSE;
import static frc.robot.Constants.ShooterConstants.PITCH_MAGNETIC_OFFSET;
import static frc.robot.Constants.ShooterConstants.PITCH_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ShooterConstants.PITCH_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.PITCH_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ShooterConstants.PITCH_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.PITCH_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.PITCH_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.RIGHT_FLYWHEEL_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.YAW_ENCODER_ID;
import static frc.robot.Constants.ShooterConstants.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.ShooterConstants.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.ShooterConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ShooterConstants.YAW_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.YAW_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.YAW_STATOR_CURRENT_LIMIT;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * ShooterSubsystem: turret yaw + pitch (hood) + flywheel.
 */
@Logged
public class ShooterSubsystem extends SubsystemBase {
  // TODO: Tune all PID and motion magic values
  // TODO: Confirm all constants and configurations, and replace placeholders
  // TODO: StartAutoTargeting()
  // TODO: Fix comments
  // TODO: Figure out what is going to be public and private
  // TODO: Change yaw potentiometer to through bore encoder
  private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_ID, CANIVORE_BUS_NAME);
  private final CANcoder yawEncoder = new CANcoder(YAW_ENCODER_ID, CANIVORE_BUS_NAME);

  private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_ID, CANIVORE_BUS_NAME);
  private final CANcoder pitchEncoder = new CANcoder(PITCH_ENCODER_ID, CANIVORE_BUS_NAME);

  private final TalonFX rightFlywheelMotor = new TalonFX(RIGHT_FLYWHEEL_MOTOR_ID, CANIVORE_BUS_NAME);
  private final TalonFX leftFlywheelMotor = new TalonFX(LEFT_FLYWHEEL_MOTOR_ID, CANIVORE_BUS_NAME);

  private final MotionMagicVoltage yawPositionRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage pitchPositionRequest = new MotionMagicVoltage(0.0);
  private final VelocityTorqueCurrentFOC rightFlywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC leftFlywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private final VoltageOut sysIdYawVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdPitchVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdFlywheelVoltage = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> yawVelocity = yawMotor.getVelocity();
  private final StatusSignal<Angle> yawPosition = yawEncoder.getPosition();
  private final StatusSignal<Angle> pitchPosition = pitchEncoder.getPosition();
  private final StatusSignal<AngularVelocity> pitchVelocity = pitchEncoder.getVelocity();
  private final StatusSignal<AngularVelocity> rightFlywheelVelocity = rightFlywheelMotor.getVelocity();
  private final StatusSignal<AngularVelocity> leftFlywheelVelocity = leftFlywheelMotor.getVelocity();

  private final SysIdRoutine yawSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Yaw Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> yawMotor.setControl(sysIdYawVoltage.withOutput(voltage.in(Volts))),
          null,
          this));

  private final SysIdRoutine pitchSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Pitch Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> pitchMotor.setControl(sysIdPitchVoltage.withOutput(voltage.in(Volts))),
          null,
          this));

  private final SysIdRoutine rightFlywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Flywheel Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> rightFlywheelMotor.setControl(sysIdFlywheelVoltage.withOutput(voltage.in(Volts))),
          null,
          this));

  private final SysIdRoutine leftFlywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Flywheel Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> leftFlywheelMotor.setControl(sysIdFlywheelVoltage.withOutput(voltage.in(Volts))),
          null,
          this));

  public ShooterSubsystem() {
    // Configure sensor to actual turret ratio
    var yawEncoderConfig = new CANcoderConfiguration();
    yawEncoderConfig.MagnetSensor.withMagnetOffset(Angle.of(0.0, Rotations)) // placeholder
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    yawEncoder.getConfigurator().apply(yawEncoderConfig);

    var yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.withNeutralMode(Brake);
    yawTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
    yawTalonConfig.withSlot0(Slot0Configs.from(YAW_SLOT_CONFIGS));
    yawTalonConfig.CurrentLimits.withSupplyCurrentLimit(YAW_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(YAW_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    yawTalonConfig.withMotionMagic(YAW_MOTION_MAGIC_CONFIGS);
    yawTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(YAW_LIMIT_FORWARD.in(Rotations))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(YAW_LIMIT_REVERSE.in(Rotations));
    yawMotor.getConfigurator().apply(yawTalonConfig);

    var pitchCanCoderConfig = new CANcoderConfiguration();
    pitchCanCoderConfig.MagnetSensor.withMagnetOffset(PITCH_MAGNETIC_OFFSET.in(Rotations))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    var pitchTalonConfig = new TalonFXConfiguration();
    pitchTalonConfig.MotorOutput.withNeutralMode(Brake);
    pitchTalonConfig.CurrentLimits.withStatorCurrentLimit(PITCH_STATOR_CURRENT_LIMIT.in(Amps))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(PITCH_SUPPLY_CURRENT_LIMIT.in(Amps))
        .withSupplyCurrentLimitEnable(true);
    pitchTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
    pitchTalonConfig.Feedback.withRotorToSensorRatio(PITCH_ROTOR_TO_SENSOR_RATIO)
        .withFeedbackRemoteSensorID(pitchEncoder.getDeviceID())
        .withFeedbackSensorSource(FusedCANcoder);
    pitchTalonConfig.withSlot0(Slot0Configs.from(PITCH_SLOT_CONFIGS));
    pitchTalonConfig.withMotionMagic(PITCH_MOTION_MAGIC_CONFIGS);
    pitchTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(PITCH_LIMIT_FORWARD.in(Rotations))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(PITCH_LIMIT_REVERSE.in(Rotations));
    pitchMotor.getConfigurator().apply(pitchTalonConfig);

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

  /**
   * Sets the turret's yaw angle.
   *
   * @param angleDegrees The desired turret angle in degrees.
   *          -180 to 180, where 0 is forward, positive is clockwise looking from above.
   */
  public void setYawAngle(Angle yawAngle) {
    Angle normalized = normalizeYaw(yawAngle);
    yawMotor.setControl(yawPositionRequest.withPosition(normalized.in(Rotations)));
  }

  /**
   * Sets the turret hood pitch angle (the angle of the shot).
   *
   * @param angleDegrees The desired pitch angle in degrees.
   */
  public void setPitchAngle(Angle pitchAngle) {
    pitchMotor.setControl(pitchPositionRequest.withPosition(pitchAngle.in(Rotations)));
  }

  /**
   * Set flywheel speed in RPM.
   *
   * @param rps The desired flywheel speed in rotations per second.
   *          Phoenix 6 velocity units for TalonFX are typically rotations/second when using the units API.
   */
  public void setFlywheelRPM(double rps) {
    rightFlywheelMotor.setControl(rightFlywheelVelocityRequest.withVelocity(rps));
    leftFlywheelMotor.setControl(leftFlywheelVelocityRequest.withVelocity(rps));
  }

  /** Cuts power to the yaw motor (open-loop 0V). */
  public void stopYaw() {
    yawMotor.stopMotor();
  }

  /** Cuts power to the pitch motor (open-loop 0V). */
  public void stopPitch() {
    pitchMotor.stopMotor();
  }

  /** Cuts power to the flywheel motor (open-loop 0V). */
  public void stopFlywheel() {
    rightFlywheelMotor.stopMotor();
    leftFlywheelMotor.stopMotor();
  }

  /** Cuts power to all shooter subsystem motors. */
  public void stopAll() {
    stopYaw();
    stopPitch();
    stopFlywheel();
  }

  /**
   * Gets current turret yaw angle based on potentiometer.
   * 
   * @return The current absolute yaw angle (rotations, not degrees).
   */
  @Logged(name = "Turret Yaw Angle")
  public Angle getYawAngle() {
    BaseStatusSignal.refreshAll(yawMotor.getPosition());
    Angle rawAngle = Angle.of(yawMotor.getPosition().getValueAsDouble(), Rotations);
    return normalizeYaw(rawAngle);
  }

  /**
   * Gets current pitch angle with latency compensation from the CANcoder.
   *
   * @return The current pitch angle.
   */
  @Logged(name = "Turret Pitch Angle")
  public Measure<AngleUnit> getPitchAngle() {
    BaseStatusSignal.refreshAll(pitchPosition, pitchVelocity);
    return BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
  }

  /**
   * Gets current flywheel velocity (RPS).
   *
   * @return The current flywheel velocity in rotations per second.
   */
  @Logged(name = "Flywheel Velocity (RPS)")
  public double getFlywheelRPS() {
    BaseStatusSignal.refreshAll(flywheelVelocity);
    return flywheelVelocity.getValueAsDouble();
  }

  /**
   * Normalize an angle to (-0.5, 0.5] rotations which corresponds to [-180, 180] degrees.
   *
   * @param angle The angle to normalize.
   * @return The normalized angle.
   */
  private Angle normalizeYaw(Angle angle) {
    double rot = angle.in(Rotations);

    // IEEEremainder with modulus 1.0 wraps into (-0.5, 0.5]
    rot = Math.IEEEremainder(rot, 1.0);

    if (rot <= -0.5)
      rot += 1.0;
    if (rot > 0.5)
      rot -= 1.0;

    return Rotations.of(rot);
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
}
