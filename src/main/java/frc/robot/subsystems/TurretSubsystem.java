package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static frc.robot.Constants.TurretConstants.YAW_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.YAW_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class TurretSubsystem extends SubsystemBase {

  private final TalonFX yawMotor = new TalonFX(25); // TODO: Verify naming scheme
  // TODO: 10 turn pot (analog 0)

  private final TalonFX pitchMotor = new TalonFX(26); // TODO: Verify naming scheme
  // TODO: Verify encoder type and ID (heard CanCoder and in motor encoder)

  private final MotionMagicVoltage yawControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  // TODO: SysIdRoutine for yaw and hood motors

  public TurretSubsystem() {
    // TODO: Configure motors and encoders

    var yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.NeutralMode = Brake;
    yawTalonConfig.CurrentLimits.StatorCurrentLimit = YAW_STATOR_CURRENT_LIMIT; // TODO: .in(Amps);
    yawTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    yawTalonConfig.CurrentLimits.SupplyCurrentLimit = YAW_SUPPLY_CURRENT_LIMIT; // TODO: .in(Amps);
    yawTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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
}
