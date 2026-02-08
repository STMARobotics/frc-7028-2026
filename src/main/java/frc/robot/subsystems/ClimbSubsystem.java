package frc.robot.subsystems;

import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.ClimbConstants.CLIMB_FORWARD_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_OUTPUT_FORWARD_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_OUTPUT_REVERSE_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_REVERSE_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CANDI_CLIMB_LIMITS;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CANDI_CLIMB_STOW;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_FOLLOWER_MOTOR;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_LEADER_MOTOR;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class ClimbSubsystem extends SubsystemBase {
  /*
   * climb motor 1 = #30
   * climb motor 2 = #31
   * CANdi #30 S1 forward, S2 reverse
   * #31 CANdi stow sensor (S1)
   */

  // keep track of states
  private enum ClimbState {
    IDLE,
    PREPARETOL1,
    L1,
    PREPARETOL2,
    L2,
    PREPARETOL3,
    L3,
    PREPARETOSTOW,
    MOVETOSTOW,
    GOTOFLOOR
  }

  private ClimbState currentClimbState = ClimbState.IDLE;

  private final TalonFX climbMotorLeader = new TalonFX(DEVICE_ID_CLIMB_LEADER_MOTOR, CANIVORE_BUS);
  private final TalonFX climbMotorFollower = new TalonFX(DEVICE_ID_CLIMB_FOLLOWER_MOTOR, CANIVORE_BUS);
  private final CANdi limmitCANdi = new CANdi(DEVICE_ID_CANDI_CLIMB_LIMITS, CANIVORE_BUS);
  private final CANdi stowCANdi = new CANdi(DEVICE_ID_CANDI_CLIMB_STOW, CANIVORE_BUS);

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final StatusSignal<Boolean> topLimitSwitchSignal = limmitCANdi.getS1Closed();
  private final StatusSignal<Boolean> bottomLimitSwitchSignal = limmitCANdi.getS2Closed();
  private final StatusSignal<Boolean> stowSensorSignal = stowCANdi.getS1Closed();

  /**
   * subsystem for the climb
   */
  public ClimbSubsystem() {
    // create CANdi config
    CANdiConfiguration climbCANdiLimmitsConfig = new CANdiConfiguration();
    climbCANdiLimmitsConfig.DigitalInputs.withS1CloseState(S1CloseStateValue.CloseWhenLow)
        .withS2CloseState(S2CloseStateValue.CloseWhenLow);

    // apply CANdi config
    limmitCANdi.getConfigurator().apply(climbCANdiLimmitsConfig);

    // create motor config
    TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
    climbMotorConfig
        .withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs().withForwardLimitRemoteCANdiS1(limmitCANdi)
                .withForwardLimitEnable(true)
                .withForwardLimitAutosetPositionValue(CLIMB_FORWARD_LIMIT)
                .withForwardLimitAutosetPositionEnable(true)
                .withReverseLimitRemoteCANdiS2(limmitCANdi)
                .withReverseLimitEnable(true)
                .withReverseLimitAutosetPositionValue(CLIMB_REVERSE_LIMIT)
                .withReverseLimitAutosetPositionEnable(true))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(CLIMB_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(CLIMB_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));

    // apply motor config
    climbMotorLeader.getConfigurator().apply(climbMotorConfig);
    climbMotorFollower.getConfigurator().apply(climbMotorConfig);
    climbMotorFollower.setControl(new Follower(DEVICE_ID_CLIMB_LEADER_MOTOR, MotorAlignmentValue.Aligned));
  }

  @Logged
  public boolean isAtTopLimit() {
    return topLimitSwitchSignal.refresh().getValue();
  }

  @Logged
  public boolean isAtBottomLimit() {
    return bottomLimitSwitchSignal.refresh().getValue();
  }

  @Logged
  public boolean isStowed() {
    return stowSensorSignal.refresh().getValue();
  }

  /**
   * moves the upper hook up and the lower hook down
   */
  private void expand() {
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_FORWARD_VOLTAGE));
  }

  /*
   * moves the upper hook down and the lower hook up
   */
  private void contract() {
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_REVERSE_VOLTAGE));
  }

  /**
   * moves the upper hook up
   */
  public void prepareToL1() {
    currentClimbState = ClimbState.PREPARETOL1;
    expand();
  }

  /**
   * moves the upper hook down
   */
  public void L1() {
    currentClimbState = ClimbState.L1;
    contract();
  }

  /**
   * moves the upper hook up
   */
  public void prepareToL2() {
    currentClimbState = ClimbState.PREPARETOL2;
    expand();
  }

  /**
   * moves the upper hook down
   */
  public void L2() {
    currentClimbState = ClimbState.L2;
    contract();
  }

  /**
   * moves the upper hook up
   */
  public void prepareToL3() {
    currentClimbState = ClimbState.PREPARETOL3;
    expand();
  }

  /**
   * moves the upper hook down
   */
  public void L3() {
    currentClimbState = ClimbState.L3;
    contract();
  }

  /**
   * moves the lower hook to the bottom passed the latch
   */
  public void prepareToStow() {
    currentClimbState = ClimbState.PREPARETOSTOW;
    expand();
  }

  /**
   * moves the lower hook to the stow position
   * IT WILL NOT STOP IT AT THE STOW POSITION
   */
  public void moveToStow() {
    currentClimbState = ClimbState.MOVETOSTOW;
    contract();
  }

  /**
   * moves the upper hook up
   */
  public void goToFloor() {
    currentClimbState = ClimbState.GOTOFLOOR;
    expand();
  }

  /**
   * returns true if the current requrest was completed
   * 
   * @return true/false
   */
  public boolean isActionComplete() {
    switch (currentClimbState) {
      case PREPARETOL1:
      case PREPARETOL2:
      case PREPARETOL3:
      case PREPARETOSTOW:
        return bottomLimitSwitchSignal.refresh().getValue();
      case L1:
      case L2:
      case L3:
        return topLimitSwitchSignal.refresh().getValue();
      case MOVETOSTOW:
        return stowSensorSignal.refresh().getValue();
      default:
        return false;
    }
  }

  /**
   * stops the climb if it is moving
   */
  public void stop() {
    currentClimbState = ClimbState.IDLE;
    climbMotorLeader.stopMotor();
  }

}
