package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.TurretConstants.PITCH_MOTOR_ID;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_MOTOR_ID;
import static frc.robot.Constants.TurretConstants.YAW_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.TurretConstants.YAW_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.YAW_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class TurretSubsystem extends SubsystemBase {

  private final TalonFX yawMotor = new TalonFX(YAW_MOTOR_ID);
  // TODO: 10 turn pot (analog 0)

  private final TalonFX pitchMotor = new TalonFX(PITCH_MOTOR_ID);
  // TODO: Verify encoder type and ID

  private final MotionMagicVoltage yawControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  // TODO: SysIdRoutine for yaw and hood motors

  public TurretSubsystem() {
    // TODO: Configure motors and encoders

    var yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.NeutralMode = Brake;
    yawTalonConfig.CurrentLimits.StatorCurrentLimit = YAW_STATOR_CURRENT_LIMIT.in(Amps);
    yawTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    yawTalonConfig.CurrentLimits.SupplyCurrentLimit = YAW_SUPPLY_CURRENT_LIMIT.in(Amps);
    yawTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    yawTalonConfig.Feedback.RotorToSensorRatio = YAW_ROTOR_TO_SENSOR_RATIO;
    // yawTalonConfig.Feedback.FeedbackRemoteSensorID = yawEncoder.getDeviceID();
    yawTalonConfig.Slot0 = Slot0Configs.from(YAW_SLOT_CONFIGS);
    yawTalonConfig.MotionMagic = YAW_MOTION_MAGIC_CONFIGS;
    yawTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    yawTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = YAW_LIMIT_FORWARD.in(Rotations);
    yawTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    yawTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = YAW_LIMIT_REVERSE.in(Rotations);
    yawMotor.getConfigurator().apply(yawTalonConfig);
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
