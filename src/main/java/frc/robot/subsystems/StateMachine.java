// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.Dragon.DragonState;
import frc.robot.subsystems.Elevator.ElevatorState;

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
      return new ConditionalCommand(
        new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL1(),
          m_dragon.scoreReadyL1()
        ),
        new ConditionalCommand(
          new SequentialCommandGroup(
            m_dragon.stow(),
            m_elevator.moveToPoop(),
            m_coralIntake.poopReadyL1(),
            m_dragon.poopReadyL1()
          ),
          new InstantCommand(),
          () -> CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
        ),
        m_dragon.isCoralOnDragon()
      );
    }

    public Command setL2() {
      return new ConditionalCommand(
        new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL2(),
          m_dragon.scoreReadyL2()
        ),
        new ConditionalCommand(
          new SequentialCommandGroup(
            handoff(),
            new SequentialCommandGroup(
              m_dragon.stow(),
              m_elevator.moveToL2(),
              m_dragon.scoreReadyL2()
            )
          ),
          new InstantCommand(),
          () -> CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
        ),
        m_dragon.isCoralOnDragon()
      );
    }

    public Command setL3() {
      return new ConditionalCommand(
        new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL3(),
          m_dragon.scoreReadyL3()
        ),
        new ConditionalCommand(
          new SequentialCommandGroup(
            handoff(),
            new SequentialCommandGroup(
              m_dragon.stow(),
              m_elevator.moveToL3(),
              m_dragon.scoreReadyL3()
            )
          ),
          new InstantCommand(),
          () -> CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
        ),
        m_dragon.isCoralOnDragon()
      );
    }

    public Command setL4() {
      return new ConditionalCommand(
        new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL4(),
          m_dragon.scoreReadyL4()
        ),
        new ConditionalCommand(
          new SequentialCommandGroup(
            handoff(),
            new SequentialCommandGroup(
              m_dragon.stow(),
              m_elevator.moveToL4(),
              m_dragon.scoreReadyL4()
            )
          ),
          new InstantCommand(),
          () -> CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
        ),
        m_dragon.isCoralOnDragon()
      );
    }

    public Command handoff() {
      return new SequentialCommandGroup(
        m_dragon.stow(),
        m_elevator.moveToHandoff(),
        m_dragon.handoffReady(),
        new WaitUntilCommand(() -> (DragonState.HANDOFF_READY == m_dragon.getState() && ElevatorState.HANDOFF == m_elevator.getState())),
        m_coralIntake.handoff(),
        new WaitUntilCommand(m_dragon.isCoralOnDragon()),
        m_coralIntake.stow()
      );
    }

    public Command scoreCoral() {
      return new ConditionalCommand(
        new SequentialCommandGroup(
          m_dragon.score(),
          new WaitUntilCommand(() -> !m_dragon.isCoralOnDragon().getAsBoolean()),
          m_dragon.stow(),
          m_elevator.moveToHandoff(),
          m_dragon.handoffReady(),
          m_coralIntake.intakeReady()
        ),
        new ConditionalCommand(
            new SequentialCommandGroup(
            m_coralIntake.poopL1(),
            new WaitUntilCommand(() -> !m_coralIntake.isLoaded()),
            m_dragon.stow(),
            m_elevator.moveToHandoff(),
            m_dragon.handoffReady(),
            m_coralIntake.intakeReady()
          ),
          new InstantCommand(),
          () -> (CoralIntakeState.POOP_READY == m_coralIntake.getState()  && DragonState.POOP_READY == m_dragon.getState() && ElevatorState.POOP == m_elevator.getState())
        ),
        () -> (m_dragon.isCoralOnDragon().getAsBoolean() && DragonState.SCORE_READY == m_dragon.getState() && ElevatorState.SCORE_READY == m_elevator.getState())
      );
    }

    public Command intakeCoral() {
      return new ConditionalCommand(
        new SequentialCommandGroup(
          m_coralIntake.intake(),
          new WaitUntilCommand(() -> m_coralIntake.isLoaded()),
          m_coralIntake.handoffReady()
        ),
        new InstantCommand(),
        () -> CoralIntakeState.INTAKE_READY == m_coralIntake.getState()
      );

    }

    public Command setDefaultStates() {
      return new ParallelCommandGroup(
        m_coralIntake.intakeReady(),
        m_algaeIntake.moveToStow(),
        new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToHandoff(),
          m_dragon.handoffReady()
        )
      );
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



  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
