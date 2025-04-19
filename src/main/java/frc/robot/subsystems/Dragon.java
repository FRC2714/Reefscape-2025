// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.DragonConstants;
import frc.robot.Constants.DragonConstants.LaserCanConstants;
import frc.robot.Constants.DragonConstants.PivotSetpoints;
import frc.robot.Constants.DragonConstants.RollerSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Robot;
import frc.robot.subsystems.StateMachine.ScoreLevel;
import frc.robot.utils.TunableNumber;
import java.util.function.BooleanSupplier;

public class Dragon extends SubsystemBase {

  public enum DragonSetpoint {
    STARTING_CONFIG,
    STOW,
    HANDOFF,
    HANDOFF_STANDBY,
    L1,
    L2,
    L3,
    L4,
    CLIMB,
    ALGAE_HIGH,
    ALGAE_LOW,
    ALGAE_READY,
    RETRACT
  }

  public enum DragonState {
    STARTING_CONFIG,
    STOW,
    HANDOFF_READY,
    HANDOFF,
    SCORE_READY,
    SCORE,
    POOP_READY,
    CLIMB,
    SCORE_STANDBY,
    ALGAE_REMOVE
  }

  // Tunables
  private final TunableNumber tunableAngle, tunableP;
  private SparkFlexConfig tunableConfig = Configs.Dragon.pivotConfig;

  private DragonSetpoint m_dragonSetpoint;
  private DragonState m_dragonState;
  private DragonSetpoint m_previousSetpoint;

  private boolean coralOnDragon;

  LaserCan m_laserCan = new LaserCan(1);

  // Pivot Arm
  private SparkFlex pivotMotor =
      new SparkFlex(DragonConstants.kPivotMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController pivotSparkClosedLoopController =
      pivotMotor.getClosedLoopController();
  private ArmFeedforward pivotFF = new ArmFeedforward(0, DragonConstants.kG, 0);
  private AbsoluteEncoder pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();

  // Pivot rollers
  private SparkFlex pivotRollers =
      new SparkFlex(DragonConstants.kPivotRollerMotorCanId, MotorType.kBrushless);

  private SparkLimitSwitch beamBreak = pivotRollers.getForwardLimitSwitch();

  private double pivotCurrentTarget = PivotSetpoints.kStow;

  private DCMotor pivotMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim pivotMotorSim;
  private final SingleJointedArmSim m_pivotSim =
      new SingleJointedArmSim(
          pivotMotorModel,
          SimulationRobotConstants.kPivotReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kPivotLength, SimulationRobotConstants.kPivotMass),
          SimulationRobotConstants.kPivotLength,
          SimulationRobotConstants.kMinAngleRads,
          SimulationRobotConstants.kMaxAngleRads,
          true,
          SimulationRobotConstants.kMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d for visualization
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Dragon Root", 25, 25);

  private final MechanismLigament2d m_DragonMech2D =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Pivot",
              SimulationRobotConstants.kPivotLength * SimulationRobotConstants.kPixelsPerMeter,
              180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));

  /** Creates a new Elevator and Pivot. */
  public Dragon() {
    tunableAngle = new TunableNumber("Dragon/Pivot Angle");
    tunableP = new TunableNumber("Dragon/Pivot P");
    tunableAngle.setDefault(0);
    tunableP.setDefault(0);

    try {
      m_laserCan.setRangingMode(RangingMode.SHORT);
      m_laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 6, 6));
      m_laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
    } catch (Exception e) {
      e.printStackTrace();
    }

    pivotMotor.configure(
        Configs.Dragon.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotRollers.configure(
        Configs.Dragon.pivotRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_dragonState = DragonState.STOW;
    m_dragonSetpoint = DragonSetpoint.STOW;

    pivotMotorSim = new SparkFlexSim(pivotMotor, pivotMotorModel);

    SmartDashboard.putData("Mech2D's/Dragon", m_mech2d);

    coralOnDragon = false;
  }

  public double getPivotPosition() {
    return pivotAbsoluteEncoder.getPosition();
  }

  private void moveToSetpoint() {
    pivotSparkClosedLoopController.setReference(
        pivotCurrentTarget,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        pivotFF.calculate(pivotCurrentTarget, 0));
  }

  public boolean atSetpoint() {
    if (Robot.isSimulation()) {
      return true;
    }
    return Math.abs(pivotCurrentTarget - pivotAbsoluteEncoder.getPosition())
        <= DragonConstants.kPivotThreshold;
  }

  public boolean isClearFromElevator() {
    if (Robot.isSimulation()) {
      return true;
    }
    return pivotAbsoluteEncoder.getPosition() < DragonConstants.kClearFromElevatorAngle;
  }

  public boolean isClearFromReef() {
    if (Robot.isSimulation()) {
      return true;
    }
    return pivotAbsoluteEncoder.getPosition() < DragonConstants.kClearFromReefAngle;
  }

  public boolean isClearFromReefNoCoral() {
    if (Robot.isSimulation()) {
      return true;
    }
    return pivotAbsoluteEncoder.getPosition() < DragonConstants.kClearFromReefNoCoralAngle;
  }

  public boolean isClearToScoreL4() {
    if (Robot.isSimulation()) {
      return true;
    }
    return pivotAbsoluteEncoder.getPosition() > DragonConstants.kClearToScoreL4Angle;
  }

  private void setDragonState(DragonState state) {
    m_dragonState = state;
  }

  private void setDragonSetpoint(DragonSetpoint setpoint) {
    m_dragonSetpoint = setpoint;
  }

  private void setPivot(DragonSetpoint setpoint) {
    setDragonSetpoint(setpoint);
    switch (m_dragonSetpoint) {
      case STARTING_CONFIG:
        pivotCurrentTarget = PivotSetpoints.kStartingConfig;
        break;
      case STOW:
        pivotCurrentTarget = PivotSetpoints.kStow;
        break;
      case HANDOFF:
        pivotCurrentTarget = PivotSetpoints.kHandoff;
        break;
      case HANDOFF_STANDBY:
        pivotCurrentTarget = PivotSetpoints.kHandoffStandby;
        break;
      case L1:
        pivotCurrentTarget = PivotSetpoints.kLevel1;
        break;
      case L2:
        pivotCurrentTarget = PivotSetpoints.kLevel2;
        break;
      case L3:
        pivotCurrentTarget = PivotSetpoints.kLevel3;
        break;
      case L4:
        pivotCurrentTarget = PivotSetpoints.kLevel4;
        break;
      case CLIMB:
        pivotCurrentTarget = PivotSetpoints.kClimb;
        break;
      case ALGAE_HIGH:
        pivotCurrentTarget = PivotSetpoints.kAlgaeHigh;
        break;
      case ALGAE_LOW:
        pivotCurrentTarget = PivotSetpoints.kAlgaeLow;
        break;
      case ALGAE_READY:
        pivotCurrentTarget = PivotSetpoints.kAlgaeReady;
      case RETRACT:
        pivotCurrentTarget = PivotSetpoints.kRetract;
        break;
    }
    moveToSetpoint();
  }

  private void setRollerPower(double power) {
    pivotRollers.set(power);
  }

  public BooleanSupplier rollerCurrentSpikeDetected() {
    return () -> pivotRollers.getOutputCurrent() >= DragonConstants.kRollerCurrentThreshold;
  }

  public Command removeAlgae(DragonSetpoint level) {
    return this.run(
            () -> {
              setPivot(level);
              setDragonState(DragonState.ALGAE_REMOVE);
            })
        .withName("removeAlgae()");
  }

  public Command readyAlgaeRemove() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.ALGAE_READY);
            })
        .withName("readyAlgaeRemove()");
  }

  public Command stow() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.STOW);
              setRollerPower(RollerSetpoints.kStop);
              setDragonState(DragonState.STOW);
            })
        .withName("stow()");
  }

  public Command stowAndHold() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.STOW);
              setRollerPower(RollerSetpoints.kHold);
              setDragonState(DragonState.STOW);
            })
        .withName("stow()");
  }

  public Command moveToStartingConfig() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.STARTING_CONFIG);
              setRollerPower(RollerSetpoints.kHold);
              setDragonState(DragonState.STARTING_CONFIG);
            })
        .withName("moveToStartingConfig()");
  }

  public Command handoffReady() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.HANDOFF);
              setRollerPower(RollerSetpoints.kStop);
              setDragonState(DragonState.HANDOFF_READY);
            })
        .withName("handoffReady()");
  }

  public Command handoffStandby() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.HANDOFF_STANDBY);
              setRollerPower(RollerSetpoints.kStop);
              setDragonState(DragonState.HANDOFF_READY);
            })
        .withName("handoffReady()");
  }

  public Command handoff() {
    return handoffReady()
        .until(this::atSetpoint)
        .andThen(
            this.run(
                () -> {
                  setPivot(DragonSetpoint.HANDOFF);
                  setRollerPower(RollerSetpoints.kIntake);
                  setDragonState(DragonState.HANDOFF);
                }))
        .withName("handoff()");
  }

  public Command poopReady() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.STOW);
              setRollerPower(RollerSetpoints.kStop);
              setDragonState(DragonState.POOP_READY);
            })
        .withName("poopReady()");
  }

  public Command scoreReadyLevel(DragonSetpoint level) {
    return this.run(
            () -> {
              setPivot(level);
              setRollerPower(RollerSetpoints.kHold);
              setDragonState(DragonState.SCORE_READY);
            })
        .withName("scoreReadyLevel()");
  }

  public Command climb() {
    return this.run(
        () -> {
          setPivot(DragonSetpoint.CLIMB);
          setRollerPower(RollerSetpoints.kStop);
          setDragonState(DragonState.CLIMB);
        });
  }

  public Command score() {
    return this.run(
            () -> {
              m_previousSetpoint = m_dragonSetpoint;
              setRollerPower(
                  StateMachine.LEVEL == ScoreLevel.L2
                      ? RollerSetpoints.kExtakeL2
                      : RollerSetpoints.kExtake);
              setDragonState(DragonState.SCORE);
            })
        .withName("score()"); // ADD BACK AFTER TESTING
  }

  public Command scoreAuto() {
    return this.run(
            () -> {
              m_previousSetpoint = m_dragonSetpoint;
              setRollerPower(
                  StateMachine.LEVEL == ScoreLevel.L2
                      ? RollerSetpoints.kExtakeL2
                      : RollerSetpoints.kExtakeAuto);
              setDragonState(DragonState.SCORE);
            })
        .withName("score()"); // ADD BACK AFTER TESTING
  }

  public Command retract() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.RETRACT);
              setRollerPower(RollerSetpoints.kHold);
              setDragonState(DragonState.SCORE_READY);
            })
        .withName("retract()");
  }

  public Command retractWithNoHold() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.RETRACT);
              setRollerPower(RollerSetpoints.kExtake);
              setDragonState(DragonState.SCORE_READY);
            })
        .withName("retract()");
  }

  public Command stopRoller() {
    return this.run(
            () -> {
              setRollerPower(RollerSetpoints.kStop);
            })
        .onlyIf(this::atSetpoint); // ADD BACK AFTER TESTING
  }

  public Command stopScore() {
    return this.run(
            () -> {
              setRollerPower(RollerSetpoints.kHold);
              setDragonState(DragonState.SCORE_READY);
            })
        .withName("stopScore()");
  }

  public Command scoreStandby() {
    return this.run(
            () -> {
              setPivot(DragonSetpoint.STOW);
              setRollerPower(RollerSetpoints.kHold);
              setDragonState(DragonState.SCORE_STANDBY);
            })
        .withName("score standby");
  }

  public double getSimulationCurrentDraw() {
    return m_pivotSim.getCurrentDrawAmps();
  }

  public boolean isCoralOnDragon() {
    if (Robot.isSimulation()) return coralOnDragon;
    return beamBreak.isPressed();
  }

  public void coralOnDragonTrue() {
    coralOnDragon = true;
  }

  public void coralOnDragonFalse() {
    coralOnDragon = false;
  }

  public DragonSetpoint getSetpoint() {
    return m_dragonSetpoint;
  }

  public DragonState getState() {
    return m_dragonState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Dragon/Roller/Roller Current", pivotRollers.getOutputCurrent());
    SmartDashboard.putNumber("Dragon/Pivot/Current Position", pivotAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("Dragon/Pivot/Setpoint", pivotCurrentTarget);
    SmartDashboard.putBoolean("Dragon/Pivot/at Setpoint?", atSetpoint());

    SmartDashboard.putNumber("Dragon/Pivot/Current", pivotMotor.getOutputCurrent());

    SmartDashboard.putNumber("Dragon/Roller/Roller Power", pivotRollers.getAppliedOutput());

    SmartDashboard.putString("Dragon/State", m_dragonState.toString());
    SmartDashboard.putString(
        "Dragon/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");
    SmartDashboard.putBoolean("Dragon/Coral on Dragon", isCoralOnDragon());

    m_DragonMech2D.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    pivotAbsoluteEncoder.getPosition() / SimulationRobotConstants.kPivotReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );

    // Tunable If's
    if (tunableAngle.hasChanged()) {
      pivotCurrentTarget = tunableAngle.get();
      moveToSetpoint();
    }
    if (tunableP.hasChanged()) {
      tunableConfig.closedLoop.p(tunableP.get());
      pivotMotor.configure(
          tunableConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_pivotSim.setInput(pivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    // Next, we update it. The standard loop time is 20ms.
    m_pivotSim.update(0.020);

    pivotMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_pivotSim.getVelocityRadPerSec() * SimulationRobotConstants.kPivotReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java
  }
}
