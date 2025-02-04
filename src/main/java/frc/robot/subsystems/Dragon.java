// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.DragonConstants.PivotSetpoints;
import frc.robot.Constants.DragonConstants.RollerSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Constants.DragonConstants;

public class Dragon extends SubsystemBase {


  private enum DragonSetpoint {
    STOW,
    HANDOFF,
    L1,
    L2,
    L3,
    L4
  }

  public enum DragonState {
    STOW,
    HANDOFF_READY,
    HANDOFF,
    SCORE_READY,
    SCORE,
    POOP_READY
  }

  private DragonSetpoint m_dragonSetpoint;
  private DragonState m_dragonState;

  private boolean coralOnDragon;

  // Pivot Arm
  private SparkFlex pivotMotor = new SparkFlex(DragonConstants.kPivotMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController pivotSparkClosedLoopController = pivotMotor.getClosedLoopController();
  private AbsoluteEncoder pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();

  // Pivot rollers
  private SparkFlex pivotRollers = new SparkFlex(DragonConstants.kPivotRollerMotorCanID, MotorType.kBrushless); 

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

  //Mechanism2d for visualization
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

    pivotMotor.configure(
      Configs.Dragon.pivotConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

      pivotRollers.configure(
        Configs.Dragon.pivotRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_dragonState = DragonState.STOW;
    m_dragonSetpoint = DragonSetpoint.STOW;

    pivotMotorSim = new SparkFlexSim(pivotMotor, pivotMotorModel);

    SmartDashboard.putData("dragon", m_mech2d);

    coralOnDragon = false;
  }

  private void moveToSetpoint() {
    pivotSparkClosedLoopController.setReference(pivotCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  private BooleanSupplier atSetpoint() {
    return () -> Math.abs(pivotCurrentTarget - pivotAbsoluteEncoder.getPosition()) <= DragonConstants.kPivotThreshold;
  }

  private Command setDragonStateCommand(DragonState state) {
    return new InstantCommand(() -> m_dragonState = state);
  }

  private Command setDragonSetpointCommand(DragonSetpoint setpoint) {
    return new InstantCommand(() -> m_dragonSetpoint = setpoint);
  }

  private Command setPivotCommand(DragonSetpoint setpoint) {
    return new SequentialCommandGroup(
      setDragonSetpointCommand(setpoint),
      new InstantCommand(
      () -> {
        switch (m_dragonSetpoint) {
          case STOW:
            pivotCurrentTarget = PivotSetpoints.kStow;
            break;
          case HANDOFF:
            pivotCurrentTarget = PivotSetpoints.kHandoff;
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
        }}),
        new InstantCommand(() -> moveToSetpoint()),
        new WaitUntilCommand(atSetpoint())
      );
  }

  private Command setRollerPowerCommand(double power) {
    return new InstantCommand(() -> pivotRollers.set(power));
  }

  public BooleanSupplier rollerCurrentSpikeDetected() {
    return () -> pivotRollers.getOutputCurrent() >= DragonConstants.kRollerCurrentThreshold;
  }

  public Command stow() {
    return new ParallelCommandGroup(
      setPivotCommand(DragonSetpoint.STOW),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setDragonStateCommand(DragonState.STOW));
  }

  public Command handoffReady() {
    return new ParallelCommandGroup(
      setPivotCommand(DragonSetpoint.HANDOFF),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setDragonStateCommand(DragonState.HANDOFF_READY));
  }

  public Command handoff() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        setPivotCommand(DragonSetpoint.HANDOFF),
        setRollerPowerCommand(RollerSetpoints.kStop)
      ),
      setRollerPowerCommand(RollerSetpoints.kIntake)
    ).andThen(setDragonStateCommand(DragonState.HANDOFF));
  }

  public Command poopReadyL1() {
    return new ParallelCommandGroup(
      setPivotCommand(DragonSetpoint.STOW),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setDragonStateCommand(DragonState.POOP_READY));
  }

  public Command scoreReadyL1() {
    return new ParallelCommandGroup(
      setPivotCommand(DragonSetpoint.L1),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setDragonStateCommand(DragonState.SCORE_READY));
  }

  public Command scoreReadyL2() {
    return new ParallelCommandGroup(
      setPivotCommand(DragonSetpoint.L2),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setDragonStateCommand(DragonState.SCORE_READY));
  }

  public Command scoreReadyL3() {
    return new ParallelCommandGroup(
      setPivotCommand(DragonSetpoint.L3),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setDragonStateCommand(DragonState.SCORE_READY));
  }

  public Command scoreReadyL4() {
    return new ParallelCommandGroup(
      setPivotCommand(DragonSetpoint.L4),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setDragonStateCommand(DragonState.SCORE_READY));
  }

  public Command score() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> coralOnDragon = false),
      setRollerPowerCommand(RollerSetpoints.kExtake)
    ).andThen(setDragonStateCommand(DragonState.SCORE));
  }
  
  public double getSimulationCurrentDraw() {
    return m_pivotSim.getCurrentDrawAmps();
  }

  private void setCoralOnDragon() {
    if (!coralOnDragon) {
      coralOnDragon = rollerCurrentSpikeDetected().getAsBoolean() ? true : false;
    }
  }

  public BooleanSupplier isCoralOnDragon() {
    return () -> coralOnDragon;
  }

  public DragonSetpoint getSetpoint()
  {
    return m_dragonSetpoint;
  }

  public DragonState getState()
  {
    return m_dragonState;
  }

        
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Dragon/Pivot/Target Position", pivotCurrentTarget);
    SmartDashboard.putNumber("Dragon/Pivot/Actual Position", pivotAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("roller power", pivotRollers.getAppliedOutput());

    SmartDashboard.putString("Dragon State", m_dragonState.toString());
    SmartDashboard.putBoolean("Dragon Pivot at Setpoint?", atSetpoint().getAsBoolean());

    setCoralOnDragon();


    m_DragonMech2D.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    pivotAbsoluteEncoder.getPosition() / SimulationRobotConstants.kPivotReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );
  }


  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_pivotSim.setInput(pivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Pivot Position", pivotAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Setpoint", pivotCurrentTarget);
    SmartDashboard.putNumber("roller speed", pivotRollers.getAppliedOutput());
    

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