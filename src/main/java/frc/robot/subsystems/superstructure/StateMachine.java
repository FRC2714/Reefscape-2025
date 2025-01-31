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
import frc.robot.subsystems.Limelight.Align;
import frc.robot.subsystems.superstructure.Superstructure;

public class StateMachine extends SubsystemBase {

  private Superstructure m_superstructure;

  private int targetId;
  private Align branchSide;
  private int [] allianceSideId;

  
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

  public enum limelightState{
    SIDE1RIGHT,
    SIDE2LEFT,
    SIDE2RIGHT,
    SIDE3LEFT,
    SIDE3RIGHT,
    SIDE4LEFT,
    SIDE4RIGHT,
    SIDE5LEFT,
    SIDE5RIGHT,
    SIDE6LEFT,
    SIDE6RIGHT,
    SIDE1LEFT;

  }



  private State coralIntakeState;
  private State algaeIntakeState;
  private State elevatorState;
  private State dragonState;
  private limelightState limelightState;

  /** Creates a new Statemachine. */
  public StateMachine(Superstructure m_superstructure) {

    this.m_superstructure = m_superstructure;

    coralIntakeState = State.STOW;
    algaeIntakeState = State.STOW;
    dragonState = State.STOW;
    elevatorState = State.STOW;
    limelightState = limelightState.SIDE1RIGHT;

    allianceSideId = new int[6];
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

  public Command setlimeLightState(limelightState limelightState)
  {
    return new InstantCommand(() -> this.limelightState = limelightState);
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

  public Command limelightSelectCommand(limelightState limelightState)
  {
        return this.runOnce(() -> {
        switch(limelightState)
        {
          case SIDE1RIGHT:
            targetId = allianceSideId[0];
            branchSide = Align.RIGHT;
            break;

          case SIDE2LEFT:
            targetId = allianceSideId[1];
            branchSide = Align.LEFT;
            break;
          
          case SIDE2RIGHT:
            targetId = allianceSideId[1];
            branchSide = Align.RIGHT;
            break;

          case SIDE3LEFT:
            targetId = allianceSideId[2];
            branchSide = Align.LEFT;
            break;
          
          case SIDE3RIGHT:
            targetId = allianceSideId[2];
            branchSide = Align.RIGHT;
            break;

          case SIDE4LEFT:
            targetId = allianceSideId[3];
            branchSide = Align.LEFT;
            break;

          case SIDE4RIGHT:
            targetId = allianceSideId[3];
            branchSide = Align.RIGHT;
            break;

          case SIDE5LEFT:
            targetId = allianceSideId[4];
            branchSide = Align.LEFT;
            break;

          case SIDE5RIGHT:
            targetId = allianceSideId[4];
            branchSide = Align.RIGHT;
            break;
          
          case SIDE6LEFT:
            targetId = allianceSideId[5];
            branchSide = Align.LEFT;
            break;
          
          case SIDE6RIGHT:
            targetId = allianceSideId[5];
            branchSide = Align.RIGHT;
            break;
          
          case SIDE1LEFT:
            targetId = allianceSideId[0];
            branchSide = Align.LEFT;
            break;
        }
        });
  }

  public Align getBranchSide()
  {
    return branchSide;
  }

  public Command scoreLevel(State level) {
    return new SequentialCommandGroup(
      dragonSelectCommand(State.STOW), 
      elevatorSelectCommand(level),
      dragonSelectCommand(level)
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

  public Command scoreCoral()
  {
    return m_superstructure.scoreCoral();
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
