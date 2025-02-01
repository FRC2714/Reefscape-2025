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

    public Command setL1() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) {
        return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL1(),
          m_dragon.scoreReadyL1()
        );
      }
      return new ParallelCommandGroup(
        new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToPoop(),
          m_dragon.poopReadyL1()
        ),
        m_coralIntake.poopReadyL1()
      );
    }

    public Command setL2() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) {
        return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL2(),
          m_dragon.scoreReadyL2()
        );
      }
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          m_dragon.handoff(),
          m_coralIntake.handoffReady()
        ),
        m_coralIntake.handoff(),
        new WaitUntilCommand(m_dragon.isCoralOnDragon()),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            m_dragon.stow(),
            m_elevator.moveToL2(),
            m_dragon.scoreReadyL2()
          ),
          m_coralIntake.intakeReady()
        )
      );
    }

    public Command setL3() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) {
        return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL3(),
          m_dragon.scoreReadyL3()
        );
      }
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          m_dragon.handoff(),
          m_coralIntake.handoffReady()
        ),
        m_coralIntake.handoff(),
        new WaitUntilCommand(m_dragon.isCoralOnDragon()),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            m_dragon.stow(),
            m_elevator.moveToL3(),
            m_dragon.scoreReadyL3()
          ),
          m_coralIntake.intakeReady()
        )
      );
    }

    public Command setL4() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) {
        return new SequentialCommandGroup(
          m_dragon.stow(),
          m_elevator.moveToL4(),
          m_dragon.scoreReadyL4()
        );
      }
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          m_dragon.handoff(),
          m_coralIntake.handoffReady()
        ),
        m_coralIntake.handoff(),
        new WaitUntilCommand(m_dragon.isCoralOnDragon()),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            m_dragon.stow(),
            m_elevator.moveToL4(),
            m_dragon.scoreReadyL4()
          ),
          m_coralIntake.intakeReady()
        )
      );
    }

    public Command scoreCoral() {
      if (m_dragon.isCoralOnDragon().getAsBoolean()) {
        return m_dragon.score();
      }
      else if (m_coralIntake.isLoaded()) {
        return new SequentialCommandGroup(
          m_coralIntake.poopL1(),
          new WaitUntilCommand(() -> !m_coralIntake.isLoaded()),
          m_coralIntake.intakeReady()
        );
      }
      return new InstantCommand();
    }

    public Command intakeCoral() {
      return new SequentialCommandGroup(
        m_coralIntake.intake(),
        new WaitUntilCommand(() -> m_coralIntake.isLoaded()),
        m_coralIntake.handoffReady()
      );
    }

    public Command intakeReadyCoral() {
      return m_coralIntake.intakeReady();
    }

    public Command extakeCoral() {
      return m_coralIntake.extake();
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
