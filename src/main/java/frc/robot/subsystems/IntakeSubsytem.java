//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITIONS_RETRACTED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_DEPLOYED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.PEAK_FORWARD_CURRENT;
import static frc.robot.Constants.IntakeConstants.PEAK_REVERSE_CURRENT;
import static frc.robot.Constants.IntakeConstants.REVERSE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.IntakeConstants.SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/*
 * Using the code from 2024 
 * Subsytem for the intake. its using only 1 TalonFX motor to run.
 */

public class IntakeSubsytem extends SubsystemBase {

  public static final int DEVICE_ID = 0;
  public static final String CANIVORE_BUS_NAME = null;
  private static final AngleUnit Rotations = null;
  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID, CANIVORE_BUS_NAME);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID, CANIVORE_BUS_NAME);

  // Motor request objects
  private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0).withIgnoreSoftwareLimits(true);

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

  // Configure the roller motor
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
    rollerConfig.MotionMagic = DEPLOY_MOTION_MAGIC_CONFIGS;
    rollerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rollerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITIONS_RETRACTED.in(Rotations);
    rollerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rollerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DEPLOY_POSITION_DEPLOYED.in(Rotations);
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    deployConfig.Feedback.SensorToMechanismRatio = DEPLOY_SENSOR_TO_MECHANISM_RATIO;
    deployConfig.MotionMagic = DEPLOY_MOTION_MAGIC_CONFIGS;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITIONS_RETRACTED.in(Rotations);
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DEPLOY_POSITION_DEPLOYED.in(Rotations);
    deployMotor.getConfigurator().apply(deployConfig);

    StatusSignal<Angle> deployPositionSignal = deployMotor.getPosition();

  }

  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("SysId intake dynam " + direction).finallyDo(this::stop);
  }

  public Command sysIdRollerQuasistatiCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("SysId intake dynam " + direction).finallyDo(this::stop);
  }

  // This is running the intake rollers
  public void intake() {
    runRollers(INTAKE_VELOCITY);
  }
  // This is have the rollers reverse for the intake
  public void reverse() {
    runRollers(REVERSE_VELOCITY);
  }

  // Stops the intake
  public void stop() {
    rollerMotor.stopMotor();
  }
  
  public void runRollers(AngularVelocity velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

  // Has the intake extend to a certain angle
  public boolean IntakeDeployed() {
    return deployControl.getPositionMeasure().isNear(DEPLOY_POSITION_DEPLOYED, DEPLOY_TOLERANCE);
  }

  //Rettacts the inatake bask to its resting positioin.
  public boolean IntakeRetacted() {
    return deployControl.getPositionMeasure().isNear(DEPLOY_POSITION_DEPLOYED, DEPLOY_TOLERANCE);
  }

  
  }
