package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class TurretSubsystem extends SubsystemBase {

  private final TalonFX yawMotor = new TalonFX(10); // TODO: Verify ID and naming scheme
  // TODO: Add encoder type

  private final TalonFX hoodMotor = new TalonFX(12); // TODO: Verify ID and naming scheme
  // TODO: Add encoder type

  private final MotionMagicVoltage yawControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  // TODO: SysIdRoutine for yaw and hood motors

  public TurretSubsystem() {
    // TODO: Configure motors and encoders

    var yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.NeutralMode = Brake;
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
