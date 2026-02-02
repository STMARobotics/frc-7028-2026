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
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Subsystem for controlling the turret's yaw and pitch (hood/shot angle).
 */
@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_ID, CANIVORE_BUS_NAME);
  private final AnalogPotentiometer yawPotentiometer = new AnalogPotentiometer(0, 360.0, 0);

  private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_ID, CANIVORE_BUS_NAME);
  private final CANcoder pitchEncoder = new CANcoder(PITCH_ENCODER_ID, CANIVORE_BUS_NAME);

  private final TalonFX flywheelMotor = new TalonFX(FLYWHEEL_MOTOR_ID, CANIVORE_BUS_NAME);

  private final MotionMagicVoltage yawControl = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage pitchControl = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage flywheelControl = new MotionMagicVoltage(0.0);
  private final VoltageOut sysIdYawControl = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdPitchControl = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdFlywheelControl = new VoltageOut(0.0).withEnableFOC(true);

  // TODO: Fuse motor with cancoder

  // How do we get from a potentiometer to a StatusSignal inside of a constructor
  // private final StatusSignal<Angle> yawPosition;
  private final StatusSignal<AngularVelocity> yawVelocity = yawMotor.getVelocity();
  private final StatusSignal<Angle> pitchPosition = pitchEncoder.getPosition();
  private final StatusSignal<AngularVelocity> pitchVelocity = pitchEncoder.getVelocity();
  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelMotor.getVelocity();

  // Reusable object to reduce memory pressure from reallocation
  private final MutAngle yawAngle = Rotations.mutable(0);

  private final SysIdRoutine yawSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Yaw Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        yawMotor.setControl(sysIdYawControl.withOutput(voltage.in(Volts)));
      }, null, this));

  private final SysIdRoutine pitchSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Pitch Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        pitchMotor.setControl(sysIdPitchControl.withOutput(voltage.in(Volts)));
      }, null, this));

  private final SysIdRoutine flywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Flywheel Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        flywheelMotor.setControl(sysIdFlywheelControl.withOutput(voltage.in(Volts)));
      }, null, this));

  public ShooterSubsystem() {
    // Yaw motor configuration
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

    // Pitch encoder configuration
    var pitchCanCoderConfig = new CANcoderConfiguration();
    pitchCanCoderConfig.MagnetSensor.withMagnetOffset(PITCH_MAGNETIC_OFFSET.in(Rotations))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    // Pitch motor configuration
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

    // Flywheel motor configuration
    var flywheelTalonConfig = new TalonFXConfiguration();
    flywheelTalonConfig.MotorOutput.withNeutralMode(Coast);
    flywheelTalonConfig.CurrentLimits.withStatorCurrentLimit(FLYWHEEL_STATOR_CURRENT_LIMIT.in(Amps))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_CURRENT_LIMIT.in(Amps))
        .withSupplyCurrentLimitEnable(true);
    flywheelTalonConfig.ClosedLoopGeneral.ContinuousWrap = false;
    flywheelTalonConfig.withSlot0(Slot0Configs.from(FLYWHEEL_SLOT_CONFIGS));

    flywheelMotor.getConfigurator().apply(flywheelTalonConfig);
  }

  @Override
  public void periodic() {
    yawMotor.setControl(yawControl);
    pitchMotor.setControl(pitchControl);
  }

  /**
   * Sets the turret's yaw angle.
   *
   * @param angleDegrees The desired turret angle in degrees.
   *          -180 to 180, where 0 is forward, positive is clockwise looking from above.
   */
  public void setYawAngle(Angle yawDegrees) {
    yawMotor.setControl(yawControl.withPosition(normalizeYaw(yawDegrees)));
  }

  /**
   * Sets the turret's (hood's) pitch angle.
   *
   * @param angleDegrees The desired pitch angle in degrees.
   */
  public void setPitchAngle(Angle pitchAngle) {
    pitchMotor.setControl(pitchControl.withPosition(pitchAngle.in(Rotations)));
  }

  public void setFlywheelRPM(double rpm) {

  }

  public void stopYaw() {
    yawMotor.setControl(new VoltageOut(0.0).withEnableFOC(true));
  }

  public void stopPitch() {
    pitchMotor.setControl(new VoltageOut(0.0).withEnableFOC(true));
  }

  public void stopFlywheel() {
    flywheelMotor.setControl(new VoltageOut(0.0).withEnableFOC(true));
  }

  public void stopAll() {
    stopYaw();
    stopPitch();
    stopFlywheel();
  }

  /**
   * Starts auto-targeting mode for the turret.
   */
  public void startAutoTargeting() {
    // Uses some sort of helper class that returns a custom datatype that we can do something like:
    // LookupValues lookupValues = helperName.getLookupValues(...);
    // setTurretAngle(lookupValues.getDesiredYawAngle());
    // setPitchAngle(lookupValues.getDesiredPitchAngle());
  }

  /**
   * Gets the current turret yaw angle.
   *
   * @return The current turret angle in degrees.
   *         -180 to 180, where 0 is forward, positive is clockwise looking from above.
   */
  @Logged(name = "Turret Yaw Angle")
  public Angle getYawAngle() {
    double potPosition = yawPotentiometer.get(); // .get() returns position 0-1
    return Rotations
        .of(MathUtil.interpolate(YAW_LIMIT_REVERSE.in(Rotations), YAW_LIMIT_FORWARD.in(Rotations), potPosition));
  }

  /**
   * Gets the current pitch angle.
   *
   * @return The current pitch angle in degrees.
   */
  @Logged(name = "Turret Pitch Angle")
  private Measure<AngleUnit> getPitchAngle() {
    BaseStatusSignal.refreshAll(pitchPosition);
    return BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
  }

  /**
   * Normalizes the yaw angle to be within the -180 to 180 degree range.
   *
   * @param angle The angle to normalize.
   * @return The normalized angle.
   */
  private Angle normalizeYaw(Angle angle) {
    // I dont remember
  }

  // SysId Commands (keep at bottom)
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