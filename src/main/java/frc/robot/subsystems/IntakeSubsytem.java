//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_PEAK_FORWORD_CURRENT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_PEAK_REVERSE_CURRENT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITIONS_RETRACTED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_DEPLOYED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SUPPLY_LIMIT;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_POSITION;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER;
import static frc.robot.Constants.IntakeConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.PEAK_FORWARD_ROLLER_CURRENT;
import static frc.robot.Constants.IntakeConstants.PEAK_REVERSE_ROLLER_CURRENT;
import static frc.robot.Constants.IntakeConstants.REVERSE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.ROLLER_SUPPLY_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This is the intake subsystem
 * 
 */

public class IntakeSubsytem extends SubsystemBase {

  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLER, CANIVORE_BUS);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_POSITION, CANIVORE_BUS);
  private final CANcoder deployCANcoder = new CANcoder(DEVICE_ID_ROLLER, CANIVORE_BUS);

  // Motor request objects
  private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut sysIdDeployControl = new VoltageOut(0.0);
  private final VoltageOut sysIdRollerControl = new VoltageOut(0.0);
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
          (volts) -> rollerMotor.setControl(sysIdRollerControl.withOutput(volts.in(Volts))),
          null,
          this));

  private final SysIdRoutine deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(5).per(Second),
          Volts.of(30),
          null,
          state -> SignalLogger.writeString("Deploy SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> deployMotor.setControl(sysIdDeployControl.withOutput(volts.in(Volts))),
          null,
          this));

  /**
   * Creates a new substyem for the intake
   */
  // Configure the roller motor
  public IntakeSubsytem() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.Slot0 = Slot0Configs.from(ROLLER_SLOT_CONFIGS);
    rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_FORWARD_ROLLER_CURRENT.in(Amps);
    rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = PEAK_REVERSE_ROLLER_CURRENT.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_LIMIT.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    deployConfig.Feedback.RotorToSensorRatio = DEPLOY_ROTOR_TO_SENSOR_RATIO;
    deployConfig.TorqueCurrent.PeakForwardTorqueCurrent = DEPLOY_PEAK_FORWORD_CURRENT.in(Amps);
    deployConfig.TorqueCurrent.PeakReverseTorqueCurrent = DEPLOY_PEAK_REVERSE_CURRENT.in(Amps);
    deployConfig.CurrentLimits.SupplyCurrentLimit = DEPLOY_SUPPLY_LIMIT.in(Amps);
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.MotionMagic = DEPLOY_MOTION_MAGIC_CONFIGS;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITIONS_RETRACTED.in(Rotations);
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DEPLOY_POSITION_DEPLOYED.in(Rotations);
    deployConfig.Feedback.withFusedCANcoder(deployCANcoder);
    deployConfig.Slot0 = Slot0Configs.from(DEPLOY_SLOT_CONFIGS);
    deployMotor.getConfigurator().apply(deployConfig);

  }

  /**
   * Command to run the Rollers SysID routine in dynamic mode
   * 
   * @param direction The direction to run the roller motor for dynamic mode
   * @return The SysID output for dynamic mode
   */
  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction)
        .withName("SysId intake dynam " + direction)
        .finallyDo(this::stopRollers);
  }

  /**
   * Command to run the Rollers SysID routine in quasistatic mode
   * 
   * @param direction The direcion to run the roller motor for quasistatic mode
   * @return The SysID output data for quasistaitc mode
   */
  public Command sysIdRollerQuasistatiCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction)
        .withName("SysId intake static " + direction)
        .finallyDo(this::stopRollers);
  }

  /**
   * Command to run the Deploy SysID routine in dynamic mode
   * 
   * @param direction The direction to run the deploy motor for dynmaic mode
   * @return The SysID output for dynmaic mode
   */
  public Command sysIdDeployDynamicCommand(Direction direction) {
    return deploySysIdRoutine.dynamic(direction)
        .withName("SysId deploy dynam " + direction)
        .finallyDo(this::stopDeploy);
  }

  /**
   * Command to run the Deploy SysID routine in quasistatic mode
   * 
   * @param direction The direction to run the deploy motor for quasistatic mode
   * @return The SysID output for quasistatic mode
   */
  public Command sysIdDeployQuasistatiCommand(Direction direction) {
    return deploySysIdRoutine.quasistatic(direction)
        .withName("SysID deply static  " + direction)
        .finallyDo(this::stopDeploy);
  }

  /**
   * Allows the rollers to intake fuel
   */
  public void intakeRollers() {
    runRollers(INTAKE_VELOCITY);
  }

  /**
  * 
  */
  public void ejectRollers() {
    runRollers(REVERSE_VELOCITY);
  }

  /**
   * Deploys the intake
   */
  public void deploy() {
    runDepoly(DEPLOY_POSITION_DEPLOYED);
  }

  /**
   * Retracts the intake
   */
  public void retract() {
    runDepoly(DEPLOY_POSITIONS_RETRACTED);
  }

  /**
   * Stops the intake from extending.
   */
  public void stopDeploy() {
    deployMotor.stopMotor();
  }

  /**
   * Stops the intake from collecting fuel
   */
  public void stopRollers() {
    rollerMotor.stopMotor();
  }

  private void runRollers(AngularVelocity velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity));
  }

  private void runDepoly(Measure<AngleUnit> deployPositionDeployed) {
    deployMotor.setControl(deployControl);
  }

  /**
   * Sets a limit for how far the intake can be deployed
   * 
   * @return Is the intake deployed
   */
  public boolean isDeployed() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return deployPosition.isNear(DEPLOY_POSITION_DEPLOYED, DEPLOY_TOLERANCE);
  }

  /**
   * Sets a limit for how far the intake can be retracted back
   * 
   * @return Is the intake deployed
   */
  public boolean isRetacted() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return deployPosition.isNear(DEPLOY_POSITIONS_RETRACTED, DEPLOY_TOLERANCE);
  }
}
