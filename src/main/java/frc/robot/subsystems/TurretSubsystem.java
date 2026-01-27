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
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged(strategy = Strategy.OPT_IN)
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
    yawMotor.getConfigurator().apply(yawTalonConfig);
    // yawMotor.setControl(new Follower(yawMotor.getDeviceID(), false));

    var pitchCanCoderConfig = new CANcoderConfiguration();
    pitchCanCoderConfig.MagnetSensor.MagnetOffset = PITCH_MAGNETIC_OFFSET.in(Rotations);
    pitchCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    var pitchTalonConfig = new TalonFXConfiguration();
    pitchTalonConfig.MotorOutput.NeutralMode = Brake;
    pitchTalonConfig.CurrentLimits.StatorCurrentLimit = PITCH_STATOR_CURRENT_LIMIT.in(Amps);
    pitchTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pitchTalonConfig.CurrentLimits.SupplyCurrentLimit = PITCH_SUPPLY_CURRENT_LIMIT.in(Amps);
    pitchTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pitchTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
    pitchTalonConfig.Feedback.RotorToSensorRatio = PITCH_ROTOR_TO_SENSOR_RATIO;
    pitchTalonConfig.Feedback.FeedbackRemoteSensorID = pitchEncoder.getDeviceID();
    pitchTalonConfig.Feedback.FeedbackSensorSource = FusedCANcoder;
    pitchTalonConfig.Slot0 = Slot0Configs.from(PITCH_SLOT_CONFIGS);
    pitchTalonConfig.MotionMagic = PITCH_MOTION_MAGIC_CONFIGS;
    pitchTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PITCH_LIMIT_FORWARD.in(Rotations);
    pitchTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PITCH_LIMIT_REVERSE.in(Rotations);

    pitchMotor.getConfigurator().apply(pitchTalonConfig);
  }

  @Override
  public void periodic() {
  }

  public void setTurretAngle(double angleDegrees) {
  }

  public void stopTurret() {
  }

  public double getTurretAngle() {
    return 0.0;
  }

  public void startAutoTargeting() {
  }
}
