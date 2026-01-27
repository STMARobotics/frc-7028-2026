package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {

  private final TalonFX spindexerMotor = new TalonFX(Constants.SpindexerConstants.DEVICE_ID_SPINDEXER_MOTOR);
  private final VelocityTorqueCurrentFOC spindexerVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC spindexerTorqueControl = new TorqueCurrentFOC(0.0);

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

  public SpindexerSubsystem() {
    var spinTalonconfig = new TalonFXConfiguration();
    spinTalonconfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    spinTalonconfig.TorqueCurrent
        .withPeakForwardTorqueCurrent(Constants.SpindexerConstants.SPINDEXER_TORQUE_CURRENT_LIMIT);
    spinTalonconfig.CurrentLimits.withStatorCurrentLimit(Constants.SpindexerConstants.SPINDEXER_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.SpindexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    spindexerMotor.getConfigurator().apply(spinTalonconfig);
  }

  public Command sysIdSpindexerCommand(Direction direction) {
    return spindexerSysIdRoutine.dynamic(direction).withName("Spindexer dynam " + direction).finallyDo(this::stop);
  }

  public Command sysidSpindexerQuasistaticCommand(Direction direction) {
    return spindexerSysIdRoutine.quasistatic(direction).withName("Spindexer quasi " + direction).finallyDo(this::stop);
  }

  public void feedShooter() {
    spindexerVelocityTorque.withVelocity(0.0);
    spindexerMotor.setControl(spindexerVelocityTorque);
  }

  public void intake() {
    spindexerVelocityTorque.withVelocity(-0.0);
    spindexerMotor.setControl(spindexerVelocityTorque);
  }

  public void agitate() {
    spindexerVelocityTorque.withVelocity(0.0);
    spindexerMotor.setControl(spindexerVelocityTorque);
  }

  public void stop() {
    spindexerVelocityTorque.withVelocity(0.0);
    spindexerMotor.setControl(spindexerVelocityTorque);
  }

  public boolean isEmpty() {
    // Placeholder for sensor logic to detect if spindexer is empty
    throw new UnsupportedOperationException("isEmpty() not implemented yet");
  }

  public boolean isFull() {
    // Placeholder for sensor logic to detect if spindexer is full
    throw new UnsupportedOperationException("isFull() not implemented yet");
  }
}