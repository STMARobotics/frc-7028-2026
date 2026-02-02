//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITIONS_RETRACTED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_DEPLOYED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_POSITION;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER;
import static frc.robot.Constants.IntakeConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.PEAK_DEPLOYMENT_CURRENT;
import static frc.robot.Constants.IntakeConstants.PEAK_RETRACTED_CURRENT;
import static frc.robot.Constants.IntakeConstants.REVERSE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.IntakeConstants.SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/*
 * Subsytem for the intake. 
 */

public class IntakeSubsytem extends SubsystemBase {

  public static final String canivore = null;
  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLER, canivore);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_POSITION, canivore);
  private final CANcoder deployCANcoder = new CANcoder(DEVICE_ID_ROLLER);

  // Motor request objects
  private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final StatusSignal<Angle> deployPositionSignal = deployMotor.getPosition();
  private final StatusSignal<AngularVelocity> deployVelocitySignal = deployMotor.getVelocity();

  // SysId routines
  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(5).per(Second),
          Volts.of(30),
          null,
          state -> SignalLogger.writeString("Intake SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (amps) -> rollerMotor.setControl(sysIdControl.withOutput(amps.in(Volts))),
          null,
          this));

  private final SysIdRoutine deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(5).per(Second),
          Volts.of(30),
          null,
          state -> SignalLogger.writeString("Deploy SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (amps) -> deployMotor.setControl(sysIdControl.withOutput(amps.in(Volts))),
          null,
          this));

  // Configure the roller motor
  public IntakeSubsytem() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.Feedback.RotorToSensorRatio = ROLLER_ROTOR_TO_SENSOR_RATIO;
    rollerConfig.Slot0 = Slot0Configs.from(SLOT_CONFIGS);
    rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_DEPLOYMENT_CURRENT.in(Amps);
    rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = PEAK_RETRACTED_CURRENT.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    deployConfig.Feedback.RotorToSensorRatio = DEPLOY_ROTOR_TO_SENSOR_RATIO;
    deployConfig.MotionMagic = DEPLOY_MOTION_MAGIC_CONFIGS;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITIONS_RETRACTED.in(Rotations);
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DEPLOY_POSITION_DEPLOYED.in(Rotations);
    deployConfig.Feedback.withFusedCANcoder(deployCANcoder);
    deployMotor.getConfigurator().apply(deployConfig);

  }

  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("SysId intake dynam " + direction).finallyDo(this::stop);
  }

  public Command sysIdRollerQuasistatiCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("SysId intake static " + direction).finallyDo(this::stop);
  }

  public Command sysIdDeployDynamicCommand(Direction direction) {
    return deploySysIdRoutine.dynamic(direction).withName("SysId deploy dynam " + direction).finallyDo(this::stop);
  }

  public Command sysIdDeployQuasistatiCommand(Direction direction) {
    return deploySysIdRoutine.quasistatic(direction).withName("SysID deply static " + direction).finallyDo(this::stop);
  }

  // This is running the intake rollers
  public void intake() {
    runRollers(INTAKE_VELOCITY);
  }

  // This is have the rollers reverse for the intake
  public void reverse() {
    runRollers(REVERSE_VELOCITY);
  }

  // Stops the inatakes rolllers
  public void stop() {
    rollerMotor.stopMotor();
  }

  public void runRollers(AngularVelocity velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity));
  }

  // Has the intake extend to a certain angle
  public boolean isIntakeDeployed() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return deployPosition.isNear(DEPLOY_POSITION_DEPLOYED, DEPLOY_TOLERANCE);
  }

  // Rettacts the inatake bask to its resting positioin.
  public boolean isIntakeRetacted() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return deployPosition.isNear(DEPLOY_POSITIONS_RETRACTED, DEPLOY_TOLERANCE);
  }
}
