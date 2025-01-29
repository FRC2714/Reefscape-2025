// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DragonConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.StateMachine.State;

public class Superstructure extends SubsystemBase {

  private AlgaeIntake m_algaeIntake;
  private CoralIntake m_coralIntake;
  private Dragon m_dragon;
  private Elevator m_elevator;
  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;

  /** Creates a new Superstructure. */
  public Superstructure(AlgaeIntake m_algaeIntake,
                      CoralIntake m_coralIntake,
                      Dragon m_dragon,
                      Elevator m_elevator,
                      Limelight m_leftLimelight,
                      Limelight m_rightLimelight) {

    this.m_algaeIntake = m_algaeIntake;
    this.m_coralIntake = m_coralIntake;
    this.m_dragon = m_dragon;
    this.m_elevator = m_elevator;
    this.m_leftLimelight = m_leftLimelight;
    this.m_rightLimelight = m_rightLimelight;

  }

  public Command stowCoralIntake() {
    return m_coralIntake.stowCommand();
  }

  public Command intakeCoral() {
    return m_coralIntake.intakeCommand();
  }

  public Command extakeCoral() {
    return m_coralIntake.extakeCommand();
  }

  public Command handoffCoralIntake() {
    return m_coralIntake.handoffCommand();
  }

  public Command handoffElevator() {
    return m_elevator.setSetpointCommand(State.HANDOFF);
  }

  public Command handoffDragon() {
    return m_dragon.setSetpointCommand(State.HANDOFF);
  }

  public Command stowAlgaeIntake() {
    return m_algaeIntake.stowCommand();
  }

  public Command intakeAlgae() {
    return m_algaeIntake.intakeCommand();
  }

  public Command extakeAlgae() {
    return m_algaeIntake.extakeCommand();
  }

  public Command scoreAlgae() {
    return m_algaeIntake.scoreAlgaeProcessor();
  }

  public Command scoreCoral() {
    return m_dragon.setRollerPowerCommand();
  }

  public Command setElevatorPosition(State elevatorState) {
    return m_elevator.setSetpointCommand(elevatorState);
  }

  public Command setDragonPosition(State dragonState) {
    return m_dragon.setSetpointCommand(dragonState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override 
  public void simulationPeriodic() {}
}
