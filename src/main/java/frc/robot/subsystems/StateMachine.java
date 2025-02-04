// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;

public class StateMachine extends SubsystemBase {

  private Dragon m_dragon;
  private Elevator m_elevator;
  private CoralIntake m_coralIntake;
  private AlgaeIntake m_algaeIntake;
  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;
  private Limelight m_backLimelight;
  private LED m_blinkin;

  private boolean overrideIntake;


  /** Creates a new StateMachine. */
  public StateMachine(Dragon m_dragon, Elevator m_elevator,CoralIntake m_coralIntake,AlgaeIntake m_algaeIntake,Limelight m_leftLimelight,
    Limelight m_rightLimelight, Limelight m_backLimelight, LED m_blinkin) 
    {
      this.m_algaeIntake = m_algaeIntake;
      this.m_coralIntake = m_coralIntake;
      this.m_dragon = m_dragon;
      this.m_elevator = m_elevator;
      this.m_blinkin = m_blinkin;
      this.m_leftLimelight = m_leftLimelight;
      this.m_rightLimelight = m_rightLimelight;
      this.m_backLimelight = m_backLimelight;

      this.overrideIntake = false;
    }

    public Command stowElevator() {
      return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToStow()
      );
    }

    public Command setL1() {
      if(CoralIntakeStates.HANDOFFREADY == m_coralIntake.getState()) //will only set L1 positions if the handoff is ready
      {
          if (m_dragon.isCoralOnDragon().getAsBoolean()) { //if the dragon has a coral it will move the elevator
          return new SequentialCommandGroup(
            m_dragon.stow(),
            m_elevator.moveToL1(),
            m_dragon.scoreReadyL1()
          );
        }
        return new ParallelCommandGroup( //if not it will poop it
          new SequentialCommandGroup(
            m_dragon.stow(),
            m_elevator.moveToPoop(),
            m_coralIntake.poopReadyL1(),
            m_dragon.poopReadyL1()
          )
        );
      }
      return new InstantCommand(); //the elevator will not move if the handoff is not ready
    }

    public Command setL2() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) { //if the dragon already has a coral
        return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL2(),
          m_dragon.scoreReadyL2()
        );
      }
      if(CoralIntakeStates.HANDOFFREADY == m_coralIntake.getState()) //this is if we have a coral in our intake and we are ready to handoff
      {
        return new SequentialCommandGroup(
          m_dragon.handoff(),
          m_coralIntake.handoff(),
          new WaitUntilCommand(m_dragon.isCoralOnDragon()),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              m_dragon.stow(),
              m_elevator.moveToL2(),
              m_dragon.scoreReadyL2()
            )
          )
        );
      }
      return new InstantCommand();
    }

    public Command setL3() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) {
        return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL3(),
          m_dragon.scoreReadyL3()
        );
      }
      if(CoralIntakeStates.HANDOFFREADY == m_coralIntake.getState())
      {
        return new SequentialCommandGroup(
          m_dragon.handoff(),
          m_coralIntake.handoff(),
          new WaitUntilCommand(m_dragon.isCoralOnDragon()),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              m_dragon.stow(),
              m_elevator.moveToL3(),
              m_dragon.scoreReadyL3()
            )
          )
        );
      }
      return new InstantCommand();
    }

    public Command setL4() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) {
        return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL4(),
          m_dragon.scoreReadyL4()
        );
      }
      if(CoralIntakeStates.HANDOFFREADY == m_coralIntake.getState())
      {
        return new SequentialCommandGroup(
          m_dragon.handoff(),
          m_coralIntake.handoff(),
          new WaitUntilCommand(m_dragon.isCoralOnDragon()),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              m_dragon.stow(),
              m_elevator.moveToL4(),
              m_dragon.scoreReadyL4()
            )
          )
        );
      }
      return new InstantCommand();
    }

    public Command scoreCoral() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) { //if the dragon has a piece regardless if the op pressed any level but 1 or if we decided to move levels
        return new SequentialCommandGroup(m_dragon.score(), m_coralIntake.intake());
      }
      else if (CoralIntakeStates.POOPREADY == m_coralIntake.getState()) { //if the op selects l1 and the driver wants to score, it will poop
        return new SequentialCommandGroup(
          m_coralIntake.poopL1(),
          new WaitUntilCommand(() -> !m_coralIntake.isLoaded()),
          m_coralIntake.intakeReady()
        );
      }
      return new InstantCommand();
    }

    public Command intakeCoral() {
      if(CoralIntakeStates.INTAKEREADY == m_coralIntake.getState()) //can only intake if the intake is ready 
      {
        return new SequentialCommandGroup(
          m_coralIntake.intake(),
          new WaitUntilCommand(() -> m_coralIntake.isLoaded()),
          m_coralIntake.handoffReady()
        );
      }
      return new InstantCommand(); //if its not ready it will do nothign
    }

    public Command intakeReadyCoral() {
      return m_coralIntake.intakeReady();
    }

    public Command extakeCoral() {
      return m_coralIntake.extake();
    }

    public Command stowCoralIntake() {
      return m_coralIntake.stow();
    }

    public Command handoffReady() {
      return m_coralIntake.handoffReady();
    }

    public Command intakeAlgae() {
      return m_algaeIntake.moveToIntake();
    }

    public Command extakeAlgae() {
      return m_algaeIntake.moveToExtake();
    }

    public Command stowAlgae() {
      return m_algaeIntake.moveToStow();
    }

    public Command scoreAlgae() {
      return m_algaeIntake.moveToScore();
    }



  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
