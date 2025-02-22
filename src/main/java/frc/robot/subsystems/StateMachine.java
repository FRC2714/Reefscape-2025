// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dragon.DragonSetpoint;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;

public class StateMachine extends SubsystemBase {
  public enum State {
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

  private boolean manualOverride;
  private boolean autoHandoff;
  private State m_state = State.IDLE;
  private boolean elevatorHasReset = false;

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
      Climber m_climber) {
    this.m_algaeIntake = m_algaeIntake;
    this.m_coralIntake = m_coralIntake;
    this.m_dragon = m_dragon;
    this.m_elevator = m_elevator;
    this.m_climber = m_climber;

    manualOverride = false;
    autoHandoff = true;

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

  public void setAutoHandoff(boolean enable) {
    autoHandoff = enable;
  }

  public Command enableAutoHandoff() {
    return new InstantCommand(() -> autoHandoff = true);
  }

  public Command disableAutoHandoff() {
    return new InstantCommand(() -> autoHandoff = false);
  }

  public Command setDefaultStates() {
    return new InstantCommand(() -> {
      m_coralIntake.intakeReady().until(m_coralIntake::atSetpoint)
          // .alongWith(m_algaeIntake.stow().until(m_algaeIntake::atSetpoint))
          .alongWith(
              m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
                  .andThen(m_elevator.moveToHandoff().until(m_elevator::atSetpoint))
                  .andThen(m_dragon.handoffReady().until(m_dragon::atSetpoint)))
          .schedule();
    });
  }

  public State getState() {
    return m_state;
  }

  /* Coral intake, dragon, and elevator commands */

  private Command idleSequence() {
    return (m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
        .andThen(m_elevator.moveToHandoff().until(m_elevator::atSetpoint))
        .andThen(m_dragon.handoffReady().until(m_dragon::atSetpoint)))
        .alongWith(m_coralIntake.intakeReady().until(m_coralIntake::atSetpoint))
        .beforeStarting(() -> m_state = State.IDLE);
  }

  private Command intakeSequence() {
    return m_coralIntake.intake()
        .until(m_coralIntake::isLoaded)
        .alongWith(m_dragon.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(
            new ConditionalCommand(
                handoffSequence(),
                poopStandbySequence(),
                () -> autoHandoff))
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  private Command extakeSequence() {
    return m_coralIntake.extakeReady().until(m_coralIntake::atSetpoint)
        .andThen(m_coralIntake.extake())
        .beforeStarting(() -> m_state = State.EXTAKE);
  }

  private Command handoffSequence() {
    return (m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
        .andThen(m_elevator.moveToHandoff().until(m_elevator::atSetpoint))
        .andThen(m_dragon.handoffReady()).until(m_dragon::atSetpoint))
        .alongWith(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(m_coralIntake.handoff().until(() -> m_dragon.isCoralOnDragon())
            .alongWith(m_dragon.handoff().until(() -> m_dragon.isCoralOnDragon())))
        .andThen(dragonStandbySequence()
            .alongWith(m_coralIntake.intakeReady()).until(m_dragon::isClearFromElevator))
        .beforeStarting(() -> m_state = State.HANDOFF);
  }

  private Command dragonStandbySequence() {
    return m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
        .andThen(m_elevator.moveToStow()
            .alongWith(m_dragon.scoreStandby()))
        .beforeStarting(() -> m_state = State.DRAGON_STANDBY);
  }

  private Command scoreReadySequence(ScoreLevel level) {
    return m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
        .andThen(m_elevator.moveToLevel(elevatorMap.get(level))
            .alongWith(m_dragon.scoreReadyLevel(dragonMap.get(level))))
        .beforeStarting(() -> m_state = State.DRAGON_READY);
  }

  public Command stopScore() {
    return new InstantCommand(() -> {
      if (m_state == State.DRAGON_SCORE)
        m_dragon.stopScore()
            .beforeStarting(() -> m_state = State.DRAGON_READY).schedule();
      else if (m_state == State.POOP_SCORE)
        m_coralIntake.stopPoopL1()
            .beforeStarting(() -> m_state = State.POOP_READY).schedule();
    });
  }

  private Command dragonScoreSequence() {
    return m_dragon.score()
        .until(() -> !m_dragon.isCoralOnDragon())
        .beforeStarting(() -> m_state = State.DRAGON_SCORE);
  }

  private Command poopStandbySequence() {
    return (m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
        .andThen(m_elevator.moveToHandoff().until(m_elevator::atSetpoint))
        .andThen(m_dragon.handoffReady().until(m_dragon::atSetpoint)))
        .alongWith(m_coralIntake.poopStandby().until(m_coralIntake::atSetpoint))
        .beforeStarting(() -> m_state = State.POOP_STANDBY);
  }

  private Command poopReadySequence() {
    return (m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
        .andThen(m_elevator.moveToPoop().until(m_elevator::atSetpoint))
        .andThen(m_dragon.poopReady().until(m_dragon::atSetpoint)))
        .alongWith(m_coralIntake.poopReady().until(m_coralIntake::atSetpoint))
        .beforeStarting(() -> m_state = State.POOP_READY);
  }

  private Command poopScoreSequence() {
    return m_coralIntake.poopL1()
        .until(() -> !m_coralIntake.isLoaded())
        .andThen(idleSequence())
        .beforeStarting(() -> m_state = State.POOP_SCORE);
  }

  public Command idle() {
    return new InstantCommand(() -> {
      if (manualOverride) {
        idleSequence().schedule();
      } else if (m_state == State.INTAKE || m_state == State.EXTAKE) {
        idleSequence().schedule();
      } else if (m_state == State.DRAGON_READY || m_state == State.POOP_READY) {
        if (m_state == State.DRAGON_READY && m_dragon.isCoralOnDragon()) {
          // Go to dragon standby if dragon is still loaded
          dragonStandbySequence().schedule();
        } else if (m_state == State.POOP_READY && m_coralIntake.isLoaded()) {
          // Go to poop standby if coral is still loaded
          poopStandbySequence().schedule();
        } else {
          // Go to true idle if there is no coral loaded at all
          idleSequence().schedule();
        }
      } else if (m_state == State.DRAGON_STANDBY || m_state == State.POOP_STANDBY) {
        if (m_state == State.DRAGON_STANDBY && !m_dragon.isCoralOnDragon()
            || m_state == State.POOP_STANDBY && !m_coralIntake.isLoaded()) {
          // Go to true idle if there is no coral loaded at all
          idleSequence().schedule();
        }
      }
    }).withName("idle()");
  }

  public Command intakeCoral() {
    return new InstantCommand(() -> {
      if (manualOverride || m_state == State.IDLE) {
        intakeSequence().schedule();
      }
    }).withName("intakeCoral()");
  }

  public Command extakeCoral() {
    return new InstantCommand(() -> {
      if (manualOverride || m_state == State.POOP_STANDBY) {
        extakeSequence().schedule();
      }
    }).withName("extakeCoral()");
  }

  public Command stopExtakeCoral() {
    return new InstantCommand(() -> {
      if (manualOverride || m_state == State.EXTAKE) {
        poopStandbySequence().schedule();
      }
    }).withName("stopExtakeCoral()");
  }

  public Command setL1() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_state == State.DRAGON_READY || m_state == State.DRAGON_STANDBY) {
            scoreReadySequence(ScoreLevel.L1).schedule();
          } else if (m_state == State.POOP_STANDBY || m_state == State.POOP_READY) {
            poopReadySequence().schedule();
          }
        }).withName("setL1()");
  }

  public Command setL2() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_state == State.DRAGON_READY || m_state == State.DRAGON_STANDBY) {
            scoreReadySequence(ScoreLevel.L2).schedule();
          } else if ((m_state == State.POOP_STANDBY || m_state == State.POOP_READY)) {
            handoffSequence()
                .andThen(scoreReadySequence(ScoreLevel.L2)).schedule();
          }
        }).withName("setL2()");
  }

  public Command setL3() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_state == State.DRAGON_READY || m_state == State.DRAGON_STANDBY) {
            scoreReadySequence(ScoreLevel.L3).schedule();
          } else if ((m_state == State.POOP_STANDBY || m_state == State.POOP_READY)) {
            handoffSequence()
                .andThen(scoreReadySequence(ScoreLevel.L3)).schedule();
          }

        }).withName("setL3()");
  }

  public Command setL4() {
    return new InstantCommand(
        () -> {
          if (manualOverride || m_state == State.DRAGON_READY || m_state == State.DRAGON_STANDBY) {
            scoreReadySequence(ScoreLevel.L4).schedule();
          } else if ((m_state == State.POOP_STANDBY || m_state == State.POOP_READY)) {
            handoffSequence()
                .andThen(scoreReadySequence(ScoreLevel.L4)).schedule();
          }

        }).withName("setL4()");
  }

  public Command scoreCoral() {
    return new InstantCommand(
        () -> {
          if (manualOverride) {
            m_dragon.score().schedule();
          } else if (m_state == State.DRAGON_READY) {
            dragonScoreSequence().schedule();
          } else if (m_state == State.POOP_READY) {
            poopScoreSequence().schedule();
          }
        }).withName("scoreCoral()");
  }

  public Command handoffManual() {
    return new InstantCommand(() -> {
      if (manualOverride) {
        handoffSequence().schedule();
      }
    }).withName("handoffManual()");
  }

  /* Algae commands */

  public Command intakeAlgae() {
    return new InstantCommand(() -> m_algaeIntake.intake().schedule());
  }

  public Command extakeAlgae() {
    return new InstantCommand(() -> m_algaeIntake.extake().schedule());
  }

  public Command stowAlgae() {
    return new InstantCommand(() -> m_algaeIntake.stow().schedule());
  }

  /* Climber commands */

  private Command climbSequence() {
    return m_algaeIntake.climb().until(m_algaeIntake::atSetpoint)
        .alongWith(m_coralIntake.climb().until(m_coralIntake::atSetpoint))
        .alongWith(m_dragon.stow().onlyIf(() -> !m_elevator.atSetpoint()).until(m_dragon::isClearFromElevator)
            .andThen(m_elevator.moveToStow().until(m_elevator::atSetpoint))
            .andThen(m_dragon.climb().until(m_dragon::atSetpoint)));
  }

  public Command deployClimber() {
    return new InstantCommand(() -> climbSequence()
        .andThen(m_climber.deploy().until(m_climber::atSetpoint)).schedule());
  }

  public Command retractClimber() {
    return new InstantCommand(() -> climbSequence()
        .andThen(m_climber.retract().until(m_climber::atSetpoint)).schedule());
  }

  public Command elevatorHomingSequence() {
    return ((m_dragon.stow().until(m_dragon::atSetpoint)).onlyIf(() -> !m_elevator.reverseLimitSwitchPressed())
        .andThen(m_elevator.homingSequence())
        .andThen(() -> elevatorHasReset = true)).onlyIf(() -> !elevatorHasReset);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("State Machine/Manual Override", manualOverride);
    SmartDashboard.putBoolean("Elevator homing done?", elevatorHasReset);
    SmartDashboard.putBoolean("State Machine/Auto Handoff", autoHandoff);
    SmartDashboard.putString("State Machine/State", m_state.toString());
    SmartDashboard.putString("State Machine/Current Comamand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

  }
}
