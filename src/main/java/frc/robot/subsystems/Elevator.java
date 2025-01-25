// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.superstructure.StateMachine.State;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SimulationRobotConstants;

public class Elevator extends SubsystemBase {

  // Elevator
  private SparkFlex elevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private SparkFlex elevatorFollower = new SparkFlex(ElevatorConstants.kElevatorFollowerCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorSparkClosedLoopController = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();


  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorConstants.ElevatorSetpoints.kStow;

  //Simulation testing
  private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      elevatorMotorModel,
      SimulationRobotConstants.kElevatorGearing,
      SimulationRobotConstants.kCarriageMass,
      SimulationRobotConstants.kElevatorDrumRadius,
      SimulationRobotConstants.kMinElevatorHeightMeters,
      SimulationRobotConstants.kMaxElevatorHeightMeters,
      true,
      SimulationRobotConstants.kMinElevatorHeightMeters,
      0.0,
      0.0);



  //Mechanism2d for visualization
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d(
          "Elevator",
          SimulationRobotConstants.kMinElevatorHeightMeters
              * SimulationRobotConstants.kPixelsPerMeter,
          90));
  /** Creates a new Elevator and Pivot. */
  public Elevator() {
    elevatorMotor.configure(
        Configs.Elevator.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    elevatorFollower.configure(
        Configs.Elevator.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    elevatorFollower.configure(
        Configs.Elevator.elevatorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putData("Elevator", m_mech2d);


    elevatorMotorSim = new SparkFlexSim(elevatorMotor, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);

  }

  private void moveToSetpoint() {
    elevatorSparkClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to
      // "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    }
    else if(!wasResetByLimit && elevatorMotor.getForwardLimitSwitch().isPressed())
    {
      elevatorEncoder.setPosition(32);
      wasResetByLimit = true;
    }
    else if (!elevatorMotor.getReverseLimitSwitch().isPressed() && !elevatorMotor.getForwardLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  public Command setSetpointCommand(State setpoint) {
    return new SequentialCommandGroup(
        new InstantCommand(
        () -> {
          switch (setpoint) {
            case STOW:
              elevatorCurrentTarget = ElevatorSetpoints.kStow;
              break;
            case HANDOFF:
              elevatorCurrentTarget = ElevatorSetpoints.kHandoff;
              break;
            case L1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case L2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case L3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case L4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }}),
          new InstantCommand(() -> moveToSetpoint()));
        }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    zeroElevatorOnLimitSwitch();

    SmartDashboard.putNumber("Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator/Actual Position", elevatorEncoder.getPosition());



    m_elevatorMech2d.setLength(
        SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
            + SimulationRobotConstants.kPixelsPerMeter
                * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
  }

  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Setpoint", elevatorCurrentTarget);

    

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    m_elevatorMech2d.setLength(
      SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
          + SimulationRobotConstants.kPixelsPerMeter
              * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
              * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));

    // Iterate the elevator SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
            / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
            * SimulationRobotConstants.kElevatorGearing)
            * 60.0, 
        RobotController.getBatteryVoltage(),
        0.02);

 

    // SimBattery is updated in Robot.java
  }
}