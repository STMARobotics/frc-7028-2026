//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.PEAK_FORWARD_CURRENT;
import static frc.robot.Constants.IntakeConstants.PEAK_REVERSE_CURRENT;
import static frc.robot.Constants.IntakeConstants.REVERSE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.IntakeConstants.SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

//Im using the 2024 Cresendo Intake
public class IntakeSubsytem extends SubsystemBase {

  public static final int DEVICE_ID = 0;
  public static final String CANIVORE_BUS_NAME = null;
  private static final String Rotations = null;
  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID, CANIVORE_BUS_NAME);

  // Motor request objects

  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);

  // SysId routines
  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(5).per(Second),
          Volts.of(30),
          null,
          state -> SignalLogger.writeString("Indexer SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (amps) -> rollerMotor.setControl(sysIdControl.withOutput(amps.in(Volts))),
          null,
          this));

  public IntakeSubsytem() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
    rollerConfig.Slot0 = Slot0Configs.from(SLOT_CONFIGS);
    rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_FORWARD_CURRENT.in(Amps);
    rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = PEAK_REVERSE_CURRENT.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("SysId intake dynam " + direction).finallyDo(this::stop);
  }

  public Command sysIdRollerQuasistatiCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("SysId intake dynam " + direction).finallyDo(this::stop);
  }

  public void intake() {
    runRollers(INTAKE_VELOCITY);
  }

  public void reverse() {
    runRollers(REVERSE_VELOCITY);
  }

  public void stop() {
    rollerMotor.stopMotor();
  }

  public void runRollers(AngularVelocity velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

}
