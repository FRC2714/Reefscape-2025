// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Superstructure;

public class StateMachine extends SubsystemBase {

  private Superstructure m_superstructure;

  
  public enum State {
    EXTAKE,
    HANDOFF,
    INTAKE,
    STOW,
    L1,
    L2,
    L3,
    L4,
    SCORE
  }

  private State coralIntakeState;
  private State algaeIntakeState;
  private State elevatorState;
  private State dragonState;

  /** Creates a new Statemachine. */
  public StateMachine(Superstructure m_superstructure) {

    this.m_superstructure = m_superstructure;

    coralIntakeState = State.STOW;
    algaeIntakeState = State.STOW;
    dragonState = State.STOW;
    elevatorState = State.STOW;
  }

  public Command setCoralIntakeState(State coralIntakeState) {
    return new InstantCommand(() -> this.coralIntakeState = coralIntakeState);
  }

  public Command setAlgaeIntakeState(State algaeIntakeState) {
    return new InstantCommand(() -> this.algaeIntakeState = algaeIntakeState);
  }

  public Command setDragonState(State dragonState) {
    return new InstantCommand(() -> this.dragonState = dragonState);
  }

  public Command setElevatorState(State elevatorState) {
    return new InstantCommand(() -> this.elevatorState = elevatorState);
  }

  public Command coralIntakeSelectCommand(State coralIntakeState) {
    return new SequentialCommandGroup(
      setCoralIntakeState(coralIntakeState),
      new SelectCommand<State>(Map.ofEntries(
        Map.entry(State.EXTAKE, m_superstructure.extakeCoral()), 
        Map.entry(State.INTAKE, m_superstructure.intakeCoral()),
        Map.entry(State.HANDOFF, m_superstructure.handoffCoralIntake()),
        Map.entry(State.STOW, m_superstructure.stowCoralIntake())
      ), () -> coralIntakeState)
    );
    
  }

  public Command elevatorSelectCommand(State elevatorState) {
    return new SequentialCommandGroup(
      setElevatorState(elevatorState),
      m_superstructure.setElevatorPosition(elevatorState)
    );
  
  }

  public Command algaeIntakeSelectCommand(State algaeIntakeState) {
    return new SequentialCommandGroup(
      setAlgaeIntakeState(algaeIntakeState),
      new SelectCommand<>(Map.ofEntries(
        Map.entry(State.EXTAKE, m_superstructure.extakeAlgae()), 
        Map.entry(State.INTAKE, m_superstructure.intakeAlgae()),
        Map.entry(State.SCORE, m_superstructure.scoreAlgae()),
        Map.entry(State.STOW, m_superstructure.stowAlgaeIntake())
      ), () -> algaeIntakeState)
    );
  }

  public Command dragonSelectCommand(State dragonState) {
    return new SequentialCommandGroup(
      setDragonState(dragonState),
      m_superstructure.setDragonPosition(dragonState)
    );
  }

  public Command scoreLevel(State level) {
    return new SequentialCommandGroup(
      dragonSelectCommand(State.STOW), 
      elevatorSelectCommand(level),
      dragonSelectCommand(level)
    );
  }

  public Command handoffCoral()
  {
    return new SequentialCommandGroup(
      dragonSelectCommand(State.STOW), 
      elevatorSelectCommand(State.HANDOFF),
      dragonSelectCommand(State.HANDOFF),
      coralIntakeSelectCommand(State.INTAKE)
    );
  }

  public Command stowElevator() {
    return new SequentialCommandGroup(
      dragonSelectCommand(State.STOW),
      elevatorSelectCommand(State.STOW)
    );
  }

  public Command intakeCoral() {
    return coralIntakeSelectCommand(State.INTAKE);
  }

  public Command extakeCoral() {
    return coralIntakeSelectCommand(State.EXTAKE);
  }

  public Command stowCoralIntake() {
    return coralIntakeSelectCommand(State.STOW);
  }

  public Command intakeAlgae() {
    return algaeIntakeSelectCommand(State.INTAKE);
  }

  public Command extakeAlgae() {
    return algaeIntakeSelectCommand(State.EXTAKE);
  }

  public Command scoreAlgae() {
    return algaeIntakeSelectCommand(State.SCORE);
  }

  public Command coralHandoff() {
    return new SequentialCommandGroup(
      dragonSelectCommand(State.STOW),
      elevatorSelectCommand(State.HANDOFF),
      dragonSelectCommand(State.HANDOFF),
      coralIntakeSelectCommand(State.HANDOFF)
    );
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Coral Intake State", coralIntakeState.toString());
    SmartDashboard.putString("Algae Intake State", algaeIntakeState.toString());
    SmartDashboard.putString("Dragon State", dragonState.toString());
    SmartDashboard.putString("Elevator State", elevatorState.toString());
  }
}
