package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import org.photonvision.*;

/**
 * The is the Subsystem for the Spindexer.
 * 
 */

public class SpindexerSubsystem extends SubsystemBase {
  private final TalonFX spindexerMotor = new TalonFX(Constants.SpindexerConstants.DEVICE_ID_SPINDEXER_MOTOR);
  private final VelocityTorqueCurrentFOC spindexerVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC spindexerTorqueControl = new TorqueCurrentFOC(0.0);
  private PhotonCamera hopperCam = new PhotonCamera("Hopper_cam");

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine spindexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("SysIdSpindexer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> spindexerMotor.setControl(spindexerTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  /**
   * Creates a new Subsystem for the Spindexer
   */
  public SpindexerSubsystem() {
    var spinTalonconfig = new TalonFXConfiguration();
    spinTalonconfig.withSlot0(Slot0Configs.from(Constants.SPINDEXER_SLOT_CONFIGS));
    spinTalonconfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    spinTalonconfig.TorqueCurrent
        .withPeakForwardTorqueCurrent(Constants.SpindexerConstants.SPINDEXER_TORQUE_CURRENT_LIMIT);
    spinTalonconfig.CurrentLimits.withStatorCurrentLimit(Constants.SpindexerConstants.SPINDEXER_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.SpindexerConstants.SPINDEXER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    spinTalonconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    spindexerMotor.getConfigurator().apply(spinTalonconfig);
  }

  public Command sysIdSpindexerCommand(Direction direction) {
    return spindexerSysIdRoutine.dynamic(direction).withName("Spindexer dynam " + direction).finallyDo(this::stop);
  }

  public Command sysidSpindexerQuasistaticCommand(Direction direction) {
    return spindexerSysIdRoutine.quasistatic(direction).withName("Spindexer quasi " + direction).finallyDo(this::stop);
  }

  // Spins the spindexer forward to feed the shooter
  public void feedShooter() {
    spindexerMotor
        .setControl(spindexerVelocityTorque.withVelocity(Constants.SpindexerConstants.SPINDEXER_FEED_VELOCITY));
  }

  // Spins the spindexer backward well intakeing fuel
  public void intake() {
    spindexerMotor
        .setControl(spindexerVelocityTorque.withVelocity(Constants.SpindexerConstants.SPINDEXER_INTAKE_VELOCITY));
  }

  // Agitates the spindexer to prevent jams
  public void agitate() {
    spindexerMotor
        .setControl(spindexerVelocityTorque.withVelocity(Constants.SpindexerConstants.SPINDEXER_AGITATE_VELOCITY));
  }

  // Stops the spindexer
  public void stop() {
    spindexerMotor.stopMotor();
  }

  public boolean isEmpty() {
    return !hopperCam.getLatestResult().hasTargets();
  }

  public boolean isFull() {
    if (isEmpty()) {
      return false;
    }
    double accumulatedArea = 0;
    for (var target : hopperCam.getLatestResult().getTargets()) {
      accumulatedArea += target.getArea();
      if (accumulatedArea >= Constants.SpindexerConstants.HOPPER_FULL_THRESHOLD) {
        return true;
      }
    }

    return false;
  }
}