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
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
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

/**
 * Subsystem for the climb
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ClimbSubsystem extends SubsystemBase {

  /**
   * Enum for different climb actions
   */
  public enum ClimbAction {
    /** Idle state, no action being performed */
    IDLE,
    /** Prepare to climb to Level 1 */
    PREPARE_TO_L1,
    /** Climb to Level 1 */
    L1,
    /** L1 hang position, not all the way handing off to L1 */
    L1_HANG,
    /** Prepare to climb to Level 2 */
    PREPARE_TO_L2,
    /** Climb to Level 2 */
    L2,
    /** Prepare to climb to Level 3 */
    PREPARE_TO_L3,
    /** Climb to Level 3 */
    L3,
    /** Prepare to stow the climb */
    PREPARE_TO_STOW,
    /** Move to the stow position */
    MOVE_TO_STOW,
    /** Move to the floor position */
    GO_TO_FLOOR
  }

  private final TalonFX climbMotorLeader = new TalonFX(DEVICE_ID_CLIMB_LEADER_MOTOR, CANIVORE_BUS);
  private final TalonFX climbMotorFollower = new TalonFX(DEVICE_ID_CLIMB_FOLLOWER_MOTOR, CANIVORE_BUS);
  private final CANdi limitCANdi = new CANdi(DEVICE_ID_CANDI_CLIMB_LIMITS, CANIVORE_BUS);
  private final CANdi stowCANdi = new CANdi(DEVICE_ID_CANDI_CLIMB_STOW, CANIVORE_BUS);

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final StatusSignal<Boolean> topLimitSwitchSignal = limitCANdi.getS1Closed();
  private final StatusSignal<Boolean> bottomLimitSwitchSignal = limitCANdi.getS2Closed();
  private final StatusSignal<Boolean> stowSensorSignal = stowCANdi.getS1Closed();

  private ClimbAction currentClimbAction = ClimbAction.IDLE;

  /**
   * Creates a new climb subsystem
   */
  public ClimbSubsystem() {
    // create CANdi config
    CANdiConfiguration climbCANdiLimmitsConfig = new CANdiConfiguration();
    climbCANdiLimmitsConfig.withDigitalInputs(
        new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow)
            .withS2CloseState(S2CloseStateValue.CloseWhenLow));

    // apply CANdi config
    limitCANdi.getConfigurator().apply(climbCANdiLimmitsConfig);

    // create motor config
    TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
    climbMotorConfig
        .withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs().withForwardLimitRemoteCANdiS1(limitCANdi)
                .withForwardLimitEnable(true)
                .withForwardLimitAutosetPositionValue(CLIMB_FORWARD_LIMIT)
                .withForwardLimitAutosetPositionEnable(true)
                .withReverseLimitRemoteCANdiS2(limitCANdi)
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

  /**
   * Returns True if at the top limit
   * 
   * @return True if at the top limit
   */
  @Logged
  public boolean isAtTopLimit() {
    return topLimitSwitchSignal.refresh().getValue();
  }

  /**
   * Returns True if at the bottom limit
   * 
   * @return True if at the bottom limit
   */
  @Logged
  public boolean isAtBottomLimit() {
    return bottomLimitSwitchSignal.refresh().getValue();
  }

  /**
   * Returns True if the climb is stowed
   * 
   * @return True if the climb is stowed
   */
  @Logged
  public boolean isStowed() {
    return stowSensorSignal.refresh().getValue();
  }

  /**
   * Moves the upper hook up and the lower hook down
   */
  public void expand() {
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_FORWARD_VOLTAGE));
  }

  /*
   * Moves the upper hook down and the lower hook up
   */
  public void contract() {
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_REVERSE_VOLTAGE));
  }

  /**
   * Reaches for L1 from floor
   */
  public void prepareToL1() {
    currentClimbAction = ClimbAction.PREPARE_TO_L1;
    expand();
  }

  /**
   * Moves robot from floor to L1 given robot is at prepareToL1
   */
  public void L1() {
    currentClimbAction = ClimbAction.L1;
    contract();
  }

  /**
   * Moves to the L1 hang position, which is not fully hanging on L1 so the robot can descend.
   */
  public void L1Hang() {
    currentClimbAction = ClimbAction.L1_HANG;
    // periodic() will handle moving to the stow/L1 hang position
  }

  /**
   * Reaches for L2 from L1
   */
  public void prepareToL2() {
    currentClimbAction = ClimbAction.PREPARE_TO_L2;
    expand();
  }

  /**
   * Moves robot from L1 to L2 given robot is at prepareToL2
   */
  public void L2() {
    currentClimbAction = ClimbAction.L2;
    contract();
  }

  /**
   * Reaches for L3 from L2
   */
  public void prepareToL3() {
    currentClimbAction = ClimbAction.PREPARE_TO_L3;
    expand();
  }

  /**
   * Moves robot from L2 to L3 given robot is at prepareToL3
   */
  public void L3() {
    currentClimbAction = ClimbAction.L3;
    contract();
  }

  /**
   * Moves the lower hook to the bottom past the latch
   */
  public void prepareToStow() {
    currentClimbAction = ClimbAction.PREPARE_TO_STOW;
    expand();
  }

  /**
   * Moves the lower hook to the stow position
   * <p>
   * The lower hook has to be below the latch in order to work
   */
  public void moveToStow() {
    currentClimbAction = ClimbAction.MOVE_TO_STOW;
  }

  /**
   * Moves the upper hook up
   */
  public void goToFloor() {
    currentClimbAction = ClimbAction.GO_TO_FLOOR;
    expand();
  }

  /**
   * Returns true if the current requrest was completed
   * 
   * @return True if the current action has completed
   */
  public boolean isActionComplete() {
    switch (currentClimbAction) {
      case PREPARE_TO_L1:
      case PREPARE_TO_L2:
      case PREPARE_TO_L3:
      case PREPARE_TO_STOW:
        return bottomLimitSwitchSignal.refresh().getValue();
      case L1:
      case L2:
      case L3:
        return topLimitSwitchSignal.refresh().getValue();
      case L1_HANG:
      case MOVE_TO_STOW:
        return stowSensorSignal.refresh().getValue();
      case IDLE:
        return false;
      default:
        return true;
    }
  }

  /**
   * Returns the current state of the climb
   * 
   * @return Current state of the climb
   */
  public ClimbAction getCurrentState() {
    return currentClimbAction;
  }

  /**
   * Stops the climb if it is moving
   */
  public void stop() {
    currentClimbAction = ClimbAction.IDLE;
    climbMotorLeader.stopMotor();
  }

  @Override
  public void periodic() {
    if (currentClimbAction == ClimbAction.PREPARE_TO_STOW || currentClimbAction == ClimbAction.L1_HANG && !isStowed()) {
      contract();
    } else if (currentClimbAction == ClimbAction.PREPARE_TO_STOW
        || currentClimbAction == ClimbAction.L1_HANG && isStowed()) {
      stop();
    }
  }

}
