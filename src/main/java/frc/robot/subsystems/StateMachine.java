// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.Dragon.DragonSetpoint;
import frc.robot.subsystems.Dragon.DragonState;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.Elevator.ElevatorState;

public class StateMachine extends SubsystemBase {
  enum State {
    IDLE,
    INTAKE,
    POOP_STANDBY,
    POOP_READY,
    POOP_SCORE,
    EXTAKE,
    HANDOFF,
    DRAGON_STANDBY,
    DRAGON_READY,
    DRAGON_SCORE
  }

  private Dragon m_dragon;
  private Elevator m_elevator;
  private CoralIntake m_coralIntake;
  private AlgaeIntake m_algaeIntake;
  private Climber m_climber;
  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;
  private Limelight m_backLimelight;
  private LED m_blinkin;

  private boolean manualOverride;
  private State m_state = State.IDLE;

  private enum ScoreLevel {
    L1,
    L2,
    L3,
    L4
  }

  private HashMap<ScoreLevel, ElevatorSetpoint> elevatorMap = new HashMap<>();
  private HashMap<ScoreLevel, DragonSetpoint> dragonMap = new HashMap<>();

  /** Creates a new StateMachine. */
  public StateMachine(Dragon m_dragon, Elevator m_elevator, CoralIntake m_coralIntake, AlgaeIntake m_algaeIntake,
      Climber m_climber,
      Limelight m_leftLimelight,
      Limelight m_rightLimelight, Limelight m_backLimelight, LED m_blinkin) {
    this.m_algaeIntake = m_algaeIntake;
    this.m_coralIntake = m_coralIntake;
    this.m_dragon = m_dragon;
    this.m_elevator = m_elevator;
    this.m_climber = m_climber;
    this.m_blinkin = m_blinkin;
    this.m_leftLimelight = m_leftLimelight;
    this.m_rightLimelight = m_rightLimelight;
    this.m_backLimelight = m_backLimelight;

    manualOverride = false;

    populateElevatorMap();
    populateDragonMap();
  }

  private void populateElevatorMap() {
    elevatorMap.put(ScoreLevel.L1, ElevatorSetpoint.L1);
    elevatorMap.put(ScoreLevel.L2, ElevatorSetpoint.L2);
    elevatorMap.put(ScoreLevel.L3, ElevatorSetpoint.L3);
    elevatorMap.put(ScoreLevel.L4, ElevatorSetpoint.L4);
  }

  private void populateDragonMap() {
    dragonMap.put(ScoreLevel.L1, DragonSetpoint.L1);
    dragonMap.put(ScoreLevel.L2, DragonSetpoint.L2);
    dragonMap.put(ScoreLevel.L3, DragonSetpoint.L3);
    dragonMap.put(ScoreLevel.L4, DragonSetpoint.L4);
  }

  public Command enableManualOverride() {
    return new InstantCommand(() -> manualOverride = true);
  }

  public Command disableManualOverride() {
    return new InstantCommand(() -> manualOverride = false);
  }

  public Command setDefaultStates() {
    return new InstantCommand(() -> {
      m_coralIntake.intakeReady().until(m_coralIntake::atSetpoint)
          .alongWith(m_algaeIntake.stow().until(m_algaeIntake::atSetpoint))
          .alongWith(
              m_dragon.stow().until(m_dragon::atSetpoint)
                  .andThen(m_elevator.moveToHandoff().until(m_elevator::atSetpoint))
                  .andThen(m_dragon.handoffReady().until(m_dragon::atSetpoint)))
          .alongWith(disableManualOverride())
          .schedule();
    });
  }

  /* Coral intake, dragon, and elevator commands */

  private Command idleSequence() {
    return m_dragon.stow().until(m_dragon::atSetpoint)
        .andThen(m_elevator.moveToHandoff().until(m_elevator::atSetpoint))
        .andThen(m_dragon.handoffReady().until(m_dragon::atSetpoint))
        .andThen(m_coralIntake.intakeReady().until(m_coralIntake::atSetpoint))
        .beforeStarting(() -> m_state = State.IDLE);
  }

  private Command intakeSequence() {
    return m_coralIntake.intake()
        .until(m_coralIntake::isLoaded)
        .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(
            new SelectCommand<ElevatorSetpoint>(Map.ofEntries(
                Map.entry(ElevatorSetpoint.POOP, setL1()),
                Map.entry(ElevatorSetpoint.L2, setL2()),
                Map.entry(ElevatorSetpoint.L3, setL3()),
                Map.entry(ElevatorSetpoint.L4, setL4())), () -> m_elevator.getSetpoint()))
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  private Command extakeSequence() {
    return m_coralIntake.extakeReady().until(m_coralIntake::atSetpoint)
        .andThen(m_coralIntake.extake())
        .beforeStarting(() -> m_state = State.EXTAKE);
  }

  private Command handoffSequence() {
    return m_dragon.stow().until(m_dragon::atSetpoint)
        .andThen(m_elevator.moveToHandoff().until(m_elevator::atSetpoint))
        .andThen(m_dragon.handoff()
            .until(m_dragon::atSetpoint))
        .andThen(m_coralIntake.handoff().until(() -> m_coralIntake.atSetpoint() && m_dragon.isCoralOnDragon()))
        .andThen(m_coralIntake.intakeReady().until(m_coralIntake::atSetpoint))
        .beforeStarting(() -> m_state = State.HANDOFF);
  }

  private Command dragonStandbySequence() {
    return Commands.none() // TODO: implement
        .beforeStarting(() -> m_state = State.DRAGON_STANDBY);
  }

  private Command scoreReadySequence(ScoreLevel level) {
    return m_dragon.stow().until(m_dragon::atSetpoint)
        .andThen(m_elevator.moveToLevel(elevatorMap.get(level)).until(m_elevator::atSetpoint))
        .andThen(m_dragon.scoreReadyLevel(dragonMap.get(level)).until(m_dragon::atSetpoint))
        .beforeStarting(() -> m_state = State.DRAGON_READY);
  }

  private Command dragonScoreSequence() {
    return m_dragon.score()
        .until(() -> !m_dragon.isCoralOnDragon())
        .beforeStarting(() -> m_state = State.DRAGON_SCORE);
  }

  private Command poopStandbySequenece() {
    return Commands.none() // TODO: implement
        .beforeStarting(() -> m_state = State.POOP_STANDBY);
  }

  private Command poopReadySequence() {
    return m_dragon.stow().until(m_dragon::atSetpoint)
        .andThen(m_elevator.moveToPoop().until(m_elevator::atSetpoint))
        .andThen(m_coralIntake.poopReady().until(m_coralIntake::atSetpoint))
        .andThen(m_dragon.poopReady().until(m_dragon::atSetpoint))
        .beforeStarting(() -> m_state = State.POOP_READY);
  }

  private Command poopScoreSequence() {
    return m_coralIntake.poopL1()
        .until(() -> !m_coralIntake.isLoaded())
        .beforeStarting(() -> m_state = State.POOP_SCORE);
  }

  public Command idle() {
    return new InstantCommand(() -> {
      idleSequence().schedule();
    });
  }

  public Command intakeCoral() {
    return new InstantCommand(() -> {
      if (manualOverride) {
        intakeSequence().schedule();
      } else if (m_state == State.INTAKE) {
        intakeSequence().schedule();
      } else if (m_state == State.EXTAKE || CoralIntakeState.STOW == m_coralIntake.getState()) {
        m_coralIntake.intakeReady().until(m_coralIntake::atSetpoint)
            .andThen(intakeSequence()).schedule();
      }
    });
  }

  public Command extakeCoral() {
    return new InstantCommand(() -> {
      if (manualOverride) {
        m_coralIntake.extake().schedule();
      } else if (m_state == State.EXTAKE) {
        m_coralIntake.extake().schedule();
      } else if (m_state == State.INTAKE || m_state == State.HANDOFF) {
        extakeSequence().schedule();
      }
    });
  }

  public Command setL1() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_dragon.isCoralOnDragon()) {
            scoreReadySequence(ScoreLevel.L1).schedule();
          } else if (m_state == State.HANDOFF || m_state == State.POOP_READY) {
            poopReadySequence().schedule();
          }
        });
  }

  public Command setL2() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_dragon.isCoralOnDragon()) {
            scoreReadySequence(ScoreLevel.L2).schedule();
          } else if ((m_state == State.HANDOFF || m_state == State.POOP_READY)) {
            handoffSequence()
                .andThen(scoreReadySequence(ScoreLevel.L2)).schedule();
          }
        });
  }

  public Command setL3() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_dragon.isCoralOnDragon()) {
            scoreReadySequence(ScoreLevel.L3).schedule();
          } else if ((m_state == State.HANDOFF || m_state == State.POOP_READY)) {
            handoffSequence()
                .andThen(scoreReadySequence(ScoreLevel.L3)).schedule();
          }

        });
  }

  public Command setL4() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_dragon.isCoralOnDragon()) {
            scoreReadySequence(ScoreLevel.L4).schedule();
          } else if ((m_state == State.HANDOFF || m_state == State.POOP_READY)) {
            handoffSequence()
                .andThen(scoreReadySequence(ScoreLevel.L4)).schedule();
          }

        });
  }

  public Command scoreCoral() {
    return new InstantCommand(
        () -> {
          if (manualOverride) {
            m_dragon.score().schedule();
          } else if (m_dragon.isCoralOnDragon() && m_state == State.DRAGON_READY
              && ElevatorState.SCORE_READY == m_elevator.getState()) {
            dragonScoreSequence().schedule();
          } else if ((m_state == State.POOP_READY && ElevatorSetpoint.POOP == m_elevator.getSetpoint())) {
            poopScoreSequence().schedule();
          }
        });
  }

  public Command handoffManual() {
    return new InstantCommand(() -> {
      if (manualOverride) {
        handoffSequence().schedule();
      }
    });
  }

  /* Algae commands */

  public Command intakeAlgae() {
    return new InstantCommand(() -> m_algaeIntake.intake().until(m_algaeIntake::atSetpoint).schedule());
  }

  public Command extakeAlgae() {
    return new InstantCommand(() -> m_algaeIntake.extake().until(m_algaeIntake::atSetpoint).schedule());
  }

  public Command stowAlgae() {
    return new InstantCommand(() -> m_algaeIntake.stow().until(m_algaeIntake::atSetpoint).schedule());
  }

  /* Climber commands */

  private Command climbSequence() {
    return m_algaeIntake.climb().alongWith(m_coralIntake.climb()).alongWith(m_dragon.climb())
        .alongWith(m_elevator.moveToStow())
        .until(() -> m_algaeIntake.atSetpoint() && m_coralIntake.atSetpoint() && m_dragon.atSetpoint()
            && m_elevator.atSetpoint());
  }

  public Command deployClimber() {
    return new InstantCommand(() -> climbSequence()
        .andThen(m_climber.deploy().until(m_climber::atSetpoint)).schedule());
  }

  public Command retractClimber() {
    return new InstantCommand(() -> climbSequence()
        .andThen(m_climber.retract().until(m_climber::atSetpoint)).schedule());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("State Machine/Manual Override", manualOverride);
    SmartDashboard.putString("State Machine/State", m_state.toString());
  }
}