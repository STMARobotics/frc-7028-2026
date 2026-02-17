package frc.robot.subsystems;

import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.ClimbConstants.CLIMB_FORWARD_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_OUTPUT_FORWARD_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_OUTPUT_REVERSE_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_REVERSE_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CANDI_CLIMB_BOTTOM;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CANDI_CLIMB_TOP;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_FOLLOWER_MOTOR;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_LEADER_MOTOR;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

  // keep track of states
  public enum ClimbState {
    IDLE,
    PREPARE_TO_L1,
    L1,
    PREPARE_TO_L2,
    L2,
    PREPARE_TO_L3,
    L3,
    PREPARE_TO_STOW,
    MOVE_TO_STOW,
    GO_TO_FLOOR
  }

  private final TalonFX climbMotorLeader = new TalonFX(DEVICE_ID_CLIMB_LEADER_MOTOR, CANIVORE_BUS);
  private final TalonFX climbMotorFollower = new TalonFX(DEVICE_ID_CLIMB_FOLLOWER_MOTOR, CANIVORE_BUS);
  private final CANdi topCanDi = new CANdi(DEVICE_ID_CANDI_CLIMB_TOP, CANIVORE_BUS);
  private final CANdi bottomCanDi = new CANdi(DEVICE_ID_CANDI_CLIMB_BOTTOM, CANIVORE_BUS);

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final StatusSignal<Boolean> topLimitSwitchSignal = topCanDi.getS1Closed();
  private final StatusSignal<Boolean> stowSensorSignal = topCanDi.getS2Closed();
  private final StatusSignal<Boolean> bottomLimitSwitchSignal = bottomCanDi.getS2Closed();

  private ClimbState currentClimbState = ClimbState.IDLE;

  /**
   * Creates a new climb subsystem
   */
  public ClimbSubsystem() {
    // create CANdi config
    CANdiConfiguration climbCANdiLimmitsConfig = new CANdiConfiguration();
    climbCANdiLimmitsConfig.withDigitalInputs(
        new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow)
            .withS2CloseState(S2CloseStateValue.CloseWhenLow)
            .withS1CloseState(S1CloseStateValue.CloseWhenLow));

    // apply CANdi config
    topCanDi.getConfigurator().apply(climbCANdiLimmitsConfig);
    bottomCanDi.getConfigurator().apply(climbCANdiLimmitsConfig);

    // create motor config
    TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
    climbMotorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
        .withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs().withForwardLimitRemoteCANdiS1(topCanDi)
                .withForwardLimitEnable(true)
                .withForwardLimitAutosetPositionValue(CLIMB_FORWARD_LIMIT)
                .withForwardLimitAutosetPositionEnable(true)
                .withReverseLimitRemoteCANdiS2(bottomCanDi)
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
    currentClimbState = ClimbState.PREPARE_TO_L1;
    expand();
  }

  /**
   * Moves robot from floor to L1 given robot is at prepareToL1
   */
  public void L1() {
    currentClimbState = ClimbState.L1;
    contract();
  }

  /**
   * Reaches for L2 from L1
   */
  public void prepareToL2() {
    currentClimbState = ClimbState.PREPARE_TO_L2;
    expand();
  }

  /**
   * Moves robot from L1 to L2 given robot is at prepareToL2
   */
  public void L2() {
    currentClimbState = ClimbState.L2;
    contract();
  }

  /**
   * Reaches for L3 from L2
   */
  public void prepareToL3() {
    currentClimbState = ClimbState.PREPARE_TO_L3;
    expand();
  }

  /**
   * Moves robot from L2 to L3 given robot is at prepareToL3
   */
  public void L3() {
    currentClimbState = ClimbState.L3;
    contract();
  }

  /**
   * Moves the lower hook to the bottom past the latch
   */
  public void prepareToStow() {
    currentClimbState = ClimbState.PREPARE_TO_STOW;
    expand();
  }

  /**
   * Moves the lower hook to the stow position
   * <p>
   * The lower hook has to be below the latch in order to work
   */
  public void moveToStow() {
    currentClimbState = ClimbState.MOVE_TO_STOW;
  }

  /**
   * Moves the upper hook up
   */
  public void goToFloor() {
    currentClimbState = ClimbState.GO_TO_FLOOR;
    expand();
  }

  /**
   * Returns true if the current requrest was completed
   * 
   * @return True if the current action has completed
   */
  public boolean isActionComplete() {
    switch (currentClimbState) {
      case PREPARE_TO_L1:
      case PREPARE_TO_L2:
      case PREPARE_TO_L3:
      case PREPARE_TO_STOW:
        return bottomLimitSwitchSignal.refresh().getValue();
      case L1:
      case L2:
      case L3:
        return topLimitSwitchSignal.refresh().getValue();
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
  public ClimbState getCurrentState() {
    return currentClimbState;
  }

  /**
   * Stops the climb if it is moving
   */
  public void stop() {
    currentClimbState = ClimbState.IDLE;
    climbMotorLeader.stopMotor();
  }

  @Override
  public void periodic() {
    if (currentClimbState == ClimbState.PREPARE_TO_STOW && !isStowed()) {
      contract();
    } else if (currentClimbState == ClimbState.PREPARE_TO_STOW && isStowed()) {
      stop();
    }
  }

}
