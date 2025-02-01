// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


  private enum DragonState {
    STOW,
    HANDOFF,
    L1,
    L2,
    L3,
    L4
  }

  private DragonState m_dragonState;

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

    pivotMotorSim = new SparkFlexSim(pivotMotor, pivotMotorModel);

    SmartDashboard.putData("dragon", m_mech2d);
  }

  private void moveToSetpoint() 
  {
    pivotSparkClosedLoopController.setReference(pivotCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  private Command setDragonStateCommand(DragonState state) {
    return new InstantCommand(() -> m_dragonState = state);
  }

  private Command setSetpointCommand(DragonState setpoint) {
    return new SequentialCommandGroup(
        setDragonStateCommand(setpoint),
        new InstantCommand(
        () -> {
          switch (m_dragonState) {
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
          new InstantCommand(() -> moveToSetpoint()));
    }

    private Command setRollerPowerCommand(double power) {
      return new InstantCommand(() -> pivotRollers.set(power));
    }

    public Command moveToStow() {
      return setSetpointCommand(DragonState.STOW);
    }
  
    public Command moveToHandoff() {
      return setSetpointCommand(DragonState.HANDOFF);
    }
  
    public Command moveToL1() {
      return setSetpointCommand(DragonState.L1);
    }
  
    public Command moveToL2() {
      return setSetpointCommand(DragonState.L2);
    }
  
    public Command moveToL3() {
      return setSetpointCommand(DragonState.L3);
    }
  
    public Command moveToL4() {
      return setSetpointCommand(DragonState.L4);
    }

    public Command intake() {
      return setRollerPowerCommand(RollerSetpoints.kIntake);
    }

    public Command extake() {
      return setRollerPowerCommand(RollerSetpoints.kExtake);
    }
    
    public double getSimulationCurrentDraw() {
      return m_pivotSim.getCurrentDrawAmps();
    }
        
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Pivot/Target Position", pivotCurrentTarget);
    SmartDashboard.putNumber("Pivot/Actual Position", pivotAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("roller power", pivotRollers.getAppliedOutput());

    SmartDashboard.putString("Dragon State", m_dragonState.toString());


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