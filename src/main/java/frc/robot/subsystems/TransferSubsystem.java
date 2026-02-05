package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {
  private final TalonFX transferMotor = new TalonFX(Constants.TransferConstants.DEVICE_ID_TRANSFER_MOTOR);
  private final VelocityTorqueCurrentFOC transferVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC transferTorqueControl = new TorqueCurrentFOC(0.0);
  private final CANrange transferCaNrange = new CANrange(
      Constants.TransferConstants.DEVICE_ID_TRANSFER_CANRANGE,
      CANIVORE_BUS_NAME);

  private final StatusSignal<Boolean> transferBallSignal = transferCaNrange.getIsDetected();

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine transferSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("SysIdTransfer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> transferMotor.setControl(transferTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  public TransferSubsystem() {
    var transferTalonconfig = new TalonFXConfiguration();
    transferTalonconfig.withSlot0(Slot0Configs.from(Constants.TransferConstants.TRANSFER_SLOT_CONFIGS));
    transferTalonconfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    transferTalonconfig.TorqueCurrent
        .withPeakForwardTorqueCurrent(Constants.TransferConstants.TRANSFER_TORQUE_CURRENT_LIMIT);
    transferTalonconfig.CurrentLimits.withStatorCurrentLimit(Constants.TransferConstants.TRANSFER_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.TransferConstants.TRANSFER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    transferTalonconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    transferMotor.getConfigurator().apply(transferTalonconfig);
  }

  public Command sysIdTransferCommand(Direction direction) {
    return transferSysIdRoutine.dynamic(direction).withName("Transfer sysid " + direction).finallyDo(this::stop);
  }

  public Command sysIdTransferQuasistaticCommand(Direction direction) {
    return transferSysIdRoutine.quasistatic(direction).withName("Transfer quasi " + direction).finallyDo(this::stop);
  }

  public void feedShooter() {
    transferMotor.setControl(transferVelocityTorque.withVelocity(Constants.TransferConstants.TRANSFER_FEED_VELOCITY));
  }

  public void unjam() {
    transferMotor.setControl(transferVelocityTorque.withVelocity(Constants.TransferConstants.TRANSFER_UNJAM_VELOCITY));
  }

  public void stop() {
    transferMotor.stopMotor();
  }

  public boolean isFull() {
    return transferBallSignal.refresh().getValue();
  }

  public boolean isEmpty() {
    return !isFull();
  }
}
