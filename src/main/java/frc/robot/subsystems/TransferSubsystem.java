package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.TransferConstants.DEVICE_ID_TRANSFER_CANRANGE;
import static frc.robot.Constants.TransferConstants.DEVICE_ID_TRANSFER_MOTOR;
import static frc.robot.Constants.TransferConstants.TRANSFER_FEED_VELOCITY;
import static frc.robot.Constants.TransferConstants.TRANSFER_SLOT_CONFIGS;
import static frc.robot.Constants.TransferConstants.TRANSFER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.TransferConstants.TRANSFER_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.TransferConstants.TRANSFER_TORQUE_CURRENT_LIMIT;
import static frc.robot.Constants.TransferConstants.TRANSFER_UNJAM_VELOCITY;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Subsystem for the Transfer.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class TransferSubsystem extends SubsystemBase {
  private final TalonFX transferMotor = new TalonFX(DEVICE_ID_TRANSFER_MOTOR, CANIVORE_BUS);
  private final CANrange transferCanRange = new CANrange(DEVICE_ID_TRANSFER_CANRANGE, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC transferVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC transferTorqueControl = new TorqueCurrentFOC(0.0);

  private final StatusSignal<Boolean> transferBallSignal = transferCanRange.getIsDetected();

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine transferSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("Transfer SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> transferMotor.setControl(transferTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  public TransferSubsystem() {
    var transferTalonconfig = new TalonFXConfiguration();
    transferTalonconfig.withSlot0(Slot0Configs.from(TRANSFER_SLOT_CONFIGS));
    transferTalonconfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    transferTalonconfig.TorqueCurrent.withPeakForwardTorqueCurrent(TRANSFER_TORQUE_CURRENT_LIMIT);
    transferTalonconfig.CurrentLimits.withStatorCurrentLimit(TRANSFER_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(TRANSFER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    transferTalonconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    transferMotor.getConfigurator().apply(transferTalonconfig);
  }

  public Command sysIdTransferDynamicCommand(Direction direction) {
    return transferSysIdRoutine.dynamic(direction)
        .withName("SysId transfer dynamic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdTransferQuasistaticCommand(Direction direction) {
    return transferSysIdRoutine.quasistatic(direction)
        .withName("SysId transfer quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Spins the transfer to feed the shooter
   */
  public void feedShooter() {
    transferMotor.setControl(transferVelocityTorque.withVelocity(TRANSFER_FEED_VELOCITY));
  }

  /**
   * Spins the transfer backward to unjam the transfer
   */
  public void unjam() {
    transferMotor.setControl(transferVelocityTorque.withVelocity(TRANSFER_UNJAM_VELOCITY));
  }

  /**
   * Stops the transfer motor
   */
  public void stop() {
    transferMotor.stopMotor();
  }

  /**
   * Returns true if the transfer has a ball in it
   */
  @Logged
  public boolean isFull() {
    return transferBallSignal.refresh().getValue();
  }

  /**
   * Returns true if the transfer is empty
   */
  @Logged
  public boolean isEmpty() {
    return !isFull();
  }
}
