package frc.robot.subsystems;

import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.ClimbConstants.CLIMB_FORWARD_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_OUTPUT_FORWARD_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_OUTPUT_REVERSE_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_REVERSE_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_STOW_FORWARD_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_STOW_POSITION;
import static frc.robot.Constants.ClimbConstants.CLIMB_STOW_REVERSE_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CANDI_CLIMB_LIMITS;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CANDI_CLIMB_STOW;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_FOLLOWER_MOTOR;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_LEADER_MOTOR;

import com.ctre.phoenix6.BaseStatusSignal;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * Subsystem for the Climb mechanism.
 * 
 * <p>
 * Climbing sequence:
 * <ul>
 * <li>Auto: Climb to L1, then descend to ground for teleop</li>
 * <li>Endgame: Climb to L1 → handoff → L2 → handoff → L3 → extend to clear L2 bar</li>
 * </ul>
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ClimbSubsystem extends SubsystemBase {

  /**
   * Action of the climb mechanism.
   */
  public enum ClimbAction {
    /** Idle state, no action in progress */
    IDLE,
    /** Extend to climb to L1 */
    PREPARE_L1,
    /** L1 position */
    L1,
    /** L1 hang position, not all the way handing off to L1 */
    L1_HANG,
    /** Extend to climb to L2 */
    PREPARE_L2,
    /** L2 position */
    L2,
    /** Extend to climb to L3 */
    PREPARE_L3,
    /** L3 position */
    L3,
    /** Stow position */
    STOW
  }

  private final TalonFX climbMotorLeader = new TalonFX(DEVICE_ID_CLIMB_LEADER_MOTOR, CANIVORE_BUS);
  private final TalonFX climbMotorFollower = new TalonFX(DEVICE_ID_CLIMB_FOLLOWER_MOTOR, CANIVORE_BUS);
  private final CANdi limitCanDi = new CANdi(DEVICE_ID_CANDI_CLIMB_LIMITS, CANIVORE_BUS);
  private final CANdi stowCanDi = new CANdi(DEVICE_ID_CANDI_CLIMB_STOW, CANIVORE_BUS);

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final StatusSignal<Angle> climbPositionSignal = climbMotorLeader.getPosition();
  private final StatusSignal<Boolean> stowSensorSignal = stowCanDi.getS1Closed();
  private final StatusSignal<Boolean> topLimitSwitchSignal = limitCanDi.getS1Closed();
  private final StatusSignal<Boolean> bottomLimitSwitchSignal = limitCanDi.getS2Closed();

  // The currently requested action
  private ClimbAction currentAction = ClimbAction.IDLE;

  /**
   * Creates a new ClimbSubsystem.
   */
  public ClimbSubsystem() {
    // CANdi configuration for mag switches
    CANdiConfiguration limitCanDiConfig = new CANdiConfiguration().withDigitalInputs(
        new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow)
            .withS2CloseState(S2CloseStateValue.CloseWhenLow));
    limitCanDi.getConfigurator().apply(limitCanDiConfig);

    CANdiConfiguration stowCanDiConfig = new CANdiConfiguration()
        .withDigitalInputs(new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow));
    stowCanDi.getConfigurator().apply(stowCanDiConfig);

    // Motor configuration
    TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(CLIMB_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(CLIMB_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true))
        .withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs().withForwardLimitRemoteCANdiS1(limitCanDi)
                .withForwardLimitEnable(true)
                .withForwardLimitAutosetPositionEnable(true)
                .withForwardLimitAutosetPositionValue(CLIMB_FORWARD_LIMIT)
                .withReverseLimitRemoteCANdiS2(limitCanDi)
                .withReverseLimitEnable(true)
                .withReverseLimitAutosetPositionEnable(true)
                .withReverseLimitAutosetPositionValue(CLIMB_REVERSE_LIMIT))
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));

    climbMotorLeader.getConfigurator().apply(climbMotorConfig);
    climbMotorFollower.getConfigurator().apply(climbMotorConfig);
    climbMotorFollower.setControl(new Follower(DEVICE_ID_CLIMB_LEADER_MOTOR, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {
    // Stow and L1 hang are the same position
    if (ClimbAction.STOW == currentAction || ClimbAction.L1_HANG == currentAction) {
      BaseStatusSignal.refreshAll(stowSensorSignal, climbPositionSignal);
      Angle climbPosition = climbPositionSignal.getValue();
      if (stowSensorSignal.getValue()) {
        // Climb is stowed. Reset encoder position, just in case it hasn't been set yet or drifted, and stop.
        stop();
        climbMotorLeader.setPosition(CLIMB_STOW_POSITION);
      } else if (climbPosition.gt(ClimbConstants.CLIMB_STOW_POSITION)) {
        // Climb is above the stow position, slowly lower it to stow position
        climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_STOW_REVERSE_VOLTAGE));
      } else if (climbPosition.lt(ClimbConstants.CLIMB_STOW_POSITION)) {
        // Climb is below the stow position, slowly raise it to stow position
        // NOTE: It's possible that the absolute position hasn't been set yet, and the climb is actually ABOVE the stow
        // position. In this case, it's still OK to run upward because we'll trip the top limit, which will reset the
        // position, and then move back down to the stow position.
        climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_STOW_FORWARD_VOLTAGE));
      }
    }
  }

  /**
   * Prepares to climb to L1.
   */
  public void prepareToL1() {
    currentAction = ClimbAction.PREPARE_L1;
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_FORWARD_VOLTAGE));
  }

  /**
   * Moves to L1 position.
   */
  public void L1() {
    currentAction = ClimbAction.L1;
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_REVERSE_VOLTAGE));
  }

  /**
   * Moves to the L1 hang position, which is not fully hanging on L1 so the robot can descend.
   */
  public void L1Hang() {
    currentAction = ClimbAction.L1_HANG;
  }

  /**
   * Prepares to climb to L2.
   */
  public void prepareToL2() {
    currentAction = ClimbAction.PREPARE_L2;
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_FORWARD_VOLTAGE));
  }

  /**
   * Moves to L2 position.
   */
  public void L2() {
    currentAction = ClimbAction.L2;
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_REVERSE_VOLTAGE));
  }

  /**
   * Prepares to climb to L3.
   */
  public void prepareToL3() {
    currentAction = ClimbAction.PREPARE_L3;
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_FORWARD_VOLTAGE));
  }

  /**
   * Moves to L3 position.
   */
  public void L3() {
    currentAction = ClimbAction.L3;
    climbMotorLeader.setControl(voltageOut.withOutput(CLIMB_OUTPUT_REVERSE_VOLTAGE));
  }

  /**
   * Moves to the stow position.
   * <p>
   * NOTE: This only retracts the shorter arm's hook if it is below the stow position when it is called. If the shorter
   * arm's hook is above the stow positon, it will not retract. The automated use of this method is for when the robot
   * descends from L1 and then needs to stow for teleop.
   */
  public void stow() {
    currentAction = ClimbAction.STOW;
    // periodic() will handle moving to the stow position
  }

  /**
   * Stops all climb motion
   */
  public void stop() {
    currentAction = ClimbAction.IDLE;
    climbMotorLeader.stopMotor();
  }

  /**
   * Gets the current action being performed by the climb mechanism.
   * 
   * @return the current ClimbAction
   */
  @Logged(name = "currentAction")
  public ClimbAction getCurrentAction() {
    return currentAction;
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
   * Checks if the current action is complete and ready for the next command.
   * 
   * @return true if the current action has reached its target state
   */
  @Logged
  public boolean isActionComplete() {
    switch (currentAction) {
      case PREPARE_L1:
      case PREPARE_L2:
      case PREPARE_L3:
        return topLimitSwitchSignal.refresh().getValue();
      case L1:
      case L2:
      case L3:
        return bottomLimitSwitchSignal.refresh().getValue();
      case L1_HANG:
      case STOW:
        return stowSensorSignal.refresh().getValue();
      case IDLE:
        return true;
      default:
        // should never happen
        return false;
    }
  }

}
