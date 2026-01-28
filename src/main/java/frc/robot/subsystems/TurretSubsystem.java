package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.TurretConstants.PITCH_ENCODER_ID;
import static frc.robot.Constants.TurretConstants.PITCH_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.PITCH_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.PITCH_MAGNETIC_OFFSET;
import static frc.robot.Constants.TurretConstants.PITCH_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.TurretConstants.PITCH_MOTOR_ID;
import static frc.robot.Constants.TurretConstants.PITCH_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.TurretConstants.PITCH_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.PITCH_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.PITCH_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_MOTOR_ID;
import static frc.robot.Constants.TurretConstants.YAW_POT_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.YAW_POT_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.YAW_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.YAW_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Subsystem for controlling the turret's yaw and pitch (hood/shot angle).
 */
@Logged
public class TurretSubsystem extends SubsystemBase {

  private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_ID, CANIVORE_BUS_NAME);
  private final AnalogPotentiometer yawPotentiometer = new AnalogPotentiometer(0, 3600, 0);

  private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_ID, CANIVORE_BUS_NAME);
  private final CANcoder pitchEncoder = new CANcoder(PITCH_ENCODER_ID, CANIVORE_BUS_NAME);

  private final MotionMagicVoltage yawControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut sysIdYawControl = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdPitchControl = new VoltageOut(0.0).withEnableFOC(true);

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

  public TurretSubsystem() {
    // Yaw motor configuration
    var yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.withNeutralMode(Brake);
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
  public void setTurretAngle(double angleDegrees) {
    // Sets a variable that motion magic or something will use to move the turret in periodic()
  }

  /**
   * Stops both yaw and pitch motors, set in function just in case periodic() is not called (e-stop)
   */
  public void stop() {
    yawMotor.setControl(new VoltageOut(0.0).withEnableFOC(true));
    pitchMotor.setControl(new VoltageOut(0.0).withEnableFOC(true));
  }

  /**
   * Gets the current turret yaw angle.
   *
   * @return The current turret angle in degrees.
   *         -180 to 180, where 0 is forward, positive is clockwise looking from above.
   */
  public double getTurretAngle() {
    var potValue = yawPotentiometer.get();
    double t = (YAW_POT_LIMIT_FORWARD != YAW_POT_LIMIT_REVERSE)
        ? (potValue - YAW_POT_LIMIT_REVERSE) / (YAW_POT_LIMIT_FORWARD - YAW_POT_LIMIT_REVERSE)
        : 0.0;
    t = MathUtil.clamp(t, 0.0, 1.0);

    double minRot = YAW_LIMIT_REVERSE.in(Rotations);
    double maxRot = YAW_LIMIT_FORWARD.in(Rotations);
    double rot = MathUtil.interpolate(minRot, maxRot, t);

    return rot * 360.0;
  }

  /**
   * Starts auto-targeting mode for the turret.
   */
  public void startAutoTargeting() {
    // Uses some sort of custom data type that gives it all the info it needs use a math helper class
    // Updates a variable that motion magic will use in periodic()
  }

  public Command sysIdYawQuasistaticCommand(Direction direction) {
    return yawSysIdRoutine.quasistatic(direction).withName("SysId yaw quasi " + direction).finallyDo(this::stop);
  }

  public Command sysIdYawDynamicCommand(Direction direction) {
    return yawSysIdRoutine.dynamic(direction).withName("SysId yaw dynamic " + direction).finallyDo(this::stop);
  }

  public Command sysIdPitchQuasistaticCommand(Direction direction) {
    return pitchSysIdRoutine.quasistatic(direction).withName("SysId pitch quasi " + direction).finallyDo(this::stop);
  }

  public Command sysIdPitchDynamicCommand(Direction direction) {
    return pitchSysIdRoutine.dynamic(direction).withName("SysId pitch dynamic " + direction).finallyDo(this::stop);
  }
}
