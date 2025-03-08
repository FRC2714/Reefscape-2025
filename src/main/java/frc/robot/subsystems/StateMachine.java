// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dragon.DragonSetpoint;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import java.util.HashMap;

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
    DRAGON_SCORE,
    CLIMB_READY,
    CLIMB,
    ALGAE_REMOVE
  }

  private Dragon m_dragon;
  private Elevator m_elevator;
  private CoralIntake m_coralIntake;
  private Climber m_climber;

  private boolean manualOverride;
  private boolean autoHandoff;
  private State m_state = State.IDLE;
  private boolean elevatorHasReset = false;

  public enum ScoreLevel {
    L1,
    L2,
    L3,
    L4
  }

  ScoreLevel m_level = null;

  private HashMap<ScoreLevel, ElevatorSetpoint> elevatorMap = new HashMap<>();
  private HashMap<ScoreLevel, DragonSetpoint> dragonMap = new HashMap<>();

  /** Creates a new StateMachine. */
  public StateMachine(
      Dragon m_dragon, Elevator m_elevator, CoralIntake m_coralIntake, Climber m_climber) {
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
    return new InstantCommand(
        () -> {
          if (m_state == State.CLIMB) climbSequence().schedule();
          else if (m_dragon.isCoralOnDragon()) dragonStandbySequence().schedule();
          else idleSequence().schedule();
        });
  }

  public Command setAutonomousDefaultStates() {
    return new InstantCommand(
        () -> {
          m_coralIntake.stow().alongWith(dragonStandbySequence()).schedule();
        });
  }

  public State getState() {
    return m_state;
  }

  /* Coral intake, dragon, and elevator commands */

  public Command idleSequence() {
    return (m_dragon
            .stow()
            .until(m_dragon::isClearFromReef)
            .andThen(m_elevator.moveToHandoff().until(m_elevator::isClearToStowDragon)))
        .andThen(m_dragon.stow())
        .alongWith(m_coralIntake.intakeReady())
        .beforeStarting(() -> m_state = State.IDLE);
  }

  private Command oneCoralBetweenIntakeSequence() {
    return m_coralIntake
        .coralBetween()
        .until(m_coralIntake::isLoaded)
        .alongWith(m_dragon.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(
            new ConditionalCommand(handoffSequence(), poopStandbySequence(), () -> autoHandoff))
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  public Command intakeAndContinueSequence() {
    return intakeSequence()
        .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(
            new ConditionalCommand(handoffSequence(), poopStandbySequence(), () -> autoHandoff))
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  public Command intakeSequence() {
    return m_coralIntake
        .intake()
        .alongWith(m_dragon.handoffReady())
        .until(m_coralIntake::isLoaded)
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  public Command intakeSequenceAuto() {
    return m_dragon
        .handoffReady()
        .andThen(m_coralIntake.intake())
        .until(m_coralIntake::isLoaded)
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  private Command extakeSequence() {
    return m_coralIntake
        .extakeReady()
        .until(m_coralIntake::atSetpoint)
        .andThen(m_coralIntake.extake())
        .beforeStarting(() -> m_state = State.EXTAKE);
  }

  public Command handoffSequence() {
    return (m_dragon
            .stow()
            .onlyIf(() -> !m_elevator.atSetpoint())
            .until(m_dragon::isClearFromElevator)
            .andThen(m_elevator.moveToHandoff().until(m_elevator::isClearToStowDragon))
            .andThen(m_dragon.handoffReady())
            .until(m_dragon::atSetpoint))
        .alongWith(
            m_coralIntake
                .takeLaxative()
                .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint)))
        .andThen(
            m_coralIntake
                .handoff()
                .until(() -> m_dragon.isCoralOnDragon())
                .alongWith(m_dragon.handoff().until(() -> m_dragon.isCoralOnDragon())))
        .andThen(
            dragonStandbySequence()
                .alongWith(m_coralIntake.intakeReady())
                .until(m_dragon::isClearFromElevator))
        .beforeStarting(() -> m_state = State.HANDOFF);
  }

  public Command dragonStandbySequence() {
    return m_dragon
        .stow()
        .onlyIf(() -> !m_elevator.atSetpoint())
        .until(m_dragon::isClearFromElevator)
        .andThen(m_elevator.moveToStow().alongWith(m_dragon.scoreStandby()))
        .beforeStarting(() -> m_state = State.DRAGON_STANDBY);
  }

  public Command scoreReadySequence(ScoreLevel level) {
    return m_dragon
        .stow()
        .onlyIf(() -> !m_elevator.atSetpoint() || m_dragon.getSetpoint() == DragonSetpoint.L4)
        .until(m_dragon::isClearFromReef)
        .andThen(
            m_elevator
                .moveToLevel(elevatorMap.get(level))
                .alongWith(m_dragon.scoreReadyLevel(dragonMap.get(level)))
                .until(() -> m_dragon.atSetpoint() && m_elevator.atSetpoint()))
        .beforeStarting(() -> m_state = State.DRAGON_READY);
  }

  public Command scoreReadyL4Sequence(ScoreLevel level) {
    return m_dragon
        .stow()
        .onlyIf(() -> !m_elevator.atSetpoint() || level == ScoreLevel.L4)
        .until(m_dragon::isClearFromReef)
        .andThen(
            m_dragon.retract().until(m_dragon::atSetpoint).onlyIf(() -> level == ScoreLevel.L4))
        .alongWith(m_elevator.moveToLevel(elevatorMap.get(level)).until(m_elevator::atSetpoint))
        .beforeStarting(() -> m_state = State.DRAGON_READY);
  }

  public Command stopScore() {
    return new InstantCommand(
        () -> {
          if (m_state == State.DRAGON_SCORE)
            if (m_level == ScoreLevel.L4) {
              m_dragon
                  .retract()
                  .until(m_dragon::atSetpoint)
                  .beforeStarting(() -> m_state = State.DRAGON_READY)
                  .schedule();
            } else {
              m_dragon.stopScore().beforeStarting(() -> m_state = State.DRAGON_READY).schedule();
            }
          else if (m_state == State.POOP_SCORE)
            m_coralIntake.stopPoopL1().beforeStarting(() -> m_state = State.POOP_READY).schedule();
        });
  }

  public Command dragonScoreSequence() {
    if (m_elevator.getSetpoint() == ElevatorSetpoint.L4) {
      return m_dragon
          .scoreReadyLevel(DragonSetpoint.L4)
          .until(m_dragon::atSetpoint)
          .andThen(m_dragon.score())
          .beforeStarting(() -> m_state = State.DRAGON_SCORE);
    } else {
      return m_dragon.score().beforeStarting(() -> m_state = State.DRAGON_SCORE);
    }
  }

  private Command poopStandbySequence() {
    return (m_dragon
            .stow()
            .onlyIf(() -> !m_elevator.atSetpoint())
            .until(m_dragon::isClearFromElevator)
            .andThen(m_elevator.moveToHandoff().until(m_elevator::isClearToStowDragon))
            .andThen(m_dragon.handoffReady().until(m_dragon::atSetpoint)))
        .alongWith(m_coralIntake.poopStandby().until(m_coralIntake::atSetpoint))
        .beforeStarting(() -> m_state = State.POOP_STANDBY);
  }

  private Command poopReadySequence() {
    return (m_dragon
            .stow()
            .onlyIf(() -> !m_elevator.atSetpoint())
            .until(m_dragon::isClearFromElevator)
            .andThen(m_elevator.moveToPoop().until(m_elevator::atSetpoint))
            .andThen(m_dragon.poopReady().until(m_dragon::atSetpoint)))
        .alongWith(
            m_coralIntake
                .takeLaxative()
                .andThen(m_coralIntake.poopReady().until(m_coralIntake::atSetpoint)))
        .beforeStarting(() -> m_state = State.POOP_READY);
  }

  private Command poopScoreSequence() {
    return m_coralIntake
        .poopL1()
        .until(() -> !m_coralIntake.isLoaded())
        .andThen(idleSequence())
        .beforeStarting(() -> m_state = State.POOP_SCORE);
  }

  public Command algaeRemovalSequence(DragonSetpoint level) {
    return m_dragon
        .readyAlgaeRemove()
        .until(m_dragon::atSetpoint)
        .onlyIf(() -> m_state == State.DRAGON_STANDBY || m_state == State.POOP_STANDBY)
        .andThen(
            m_dragon
                .removeAlgae(level)
                .until(m_dragon::atSetpoint)
                .beforeStarting(() -> m_state = State.ALGAE_REMOVE))
        .withName("algaeRemovalSequence()");
  }

  public Command idle() {
    return new InstantCommand(
            () -> {
              if (manualOverride) {
                idleSequence().schedule();
              } else if (m_state == State.INTAKE
                  || m_state == State.EXTAKE
                  || m_state == State.ALGAE_REMOVE) {
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
            })
        .withName("idle()");
  }

  public Command removeAlgaeReady() {
    return new InstantCommand(
            () -> {
              if (m_state == State.IDLE
                  || m_state == State.DRAGON_STANDBY
                  || m_state == State.POOP_STANDBY) {
                m_dragon.readyAlgaeRemove().schedule();
              }
            })
        .withName("removeAlgaeReady()");
  }

  public Command removeAlgae(DragonSetpoint level) {
    return new InstantCommand(
            () -> {
              if (m_state == State.IDLE
                  || m_state == State.DRAGON_STANDBY
                  || m_state == State.POOP_STANDBY) {
                if (m_state == State.DRAGON_STANDBY) {
                  algaeRemovalSequence(level).andThen(dragonStandbySequence()).schedule();
                }
                if (m_state == State.POOP_STANDBY) {
                  algaeRemovalSequence(level).andThen(poopStandbySequence()).schedule();
                }
                if (m_state == State.IDLE) {
                  algaeRemovalSequence(level).andThen(idleSequence()).schedule();
                }
              }
            })
        .withName("removeAlgae()");
  }

  public Command intakeCoral() {
    return new InstantCommand(
            () -> {
              if (manualOverride || m_state == State.IDLE || m_state == State.DRAGON_STANDBY) {
                intakeAndContinueSequence().onlyIf(() -> !m_dragon.isCoralOnDragon()).schedule();
              }
            })
        .withName("intakeCoral()");
  }

  public Command oneCoralBetweenIntake() {
    return new InstantCommand(
            () -> {
              if (manualOverride || m_state == State.IDLE) {
                oneCoralBetweenIntakeSequence().schedule();
              }
            })
        .withName("oneCoralBetweenIntake()");
  }

  public Command extakeCoral() {
    return new InstantCommand(
            () -> {
              if (manualOverride || m_state == State.POOP_STANDBY) {
                extakeSequence().schedule();
              }
            })
        .withName("extakeCoral()");
  }

  public Command stopExtakeCoral() {
    return new InstantCommand(
            () -> {
              if (manualOverride || m_state == State.EXTAKE) {
                poopStandbySequence().schedule();
              }
            })
        .withName("stopExtakeCoral()");
  }

  public Command setLevel(ScoreLevel level) {
    m_level = level;
    return new InstantCommand(
            () -> {
              if (level == ScoreLevel.L1) {
                if (manualOverride
                    || m_state == State.DRAGON_READY
                    || m_state == State.DRAGON_STANDBY) {
                  scoreReadySequence(level).schedule();
                } else if (m_state == State.POOP_STANDBY || m_state == State.POOP_READY) {
                  poopReadySequence().schedule();
                }
              } else if (level == ScoreLevel.L4) {
                if (manualOverride
                    || m_state == State.DRAGON_READY
                    || m_state == State.DRAGON_STANDBY) {
                  scoreReadyL4Sequence(level).schedule();
                } else if ((m_state == State.POOP_STANDBY || m_state == State.POOP_READY)) {
                  handoffSequence().andThen(scoreReadyL4Sequence(level)).schedule();
                }
              } else {
                if (manualOverride
                    || m_state == State.DRAGON_READY
                    || m_state == State.DRAGON_STANDBY) {
                  scoreReadySequence(level).schedule();
                } else if ((m_state == State.POOP_STANDBY || m_state == State.POOP_READY)) {
                  handoffSequence().andThen(scoreReadySequence(level)).schedule();
                }
              }
            })
        .withName("setLevel" + "(" + level.toString() + ")");
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
            })
        .withName("scoreCoral()");
  }

  public Command scoreCoralAuto() {
    return dragonScoreSequence().withTimeout(.5).andThen(stopScore()).withName("scoreCoralAuto()");
  }

  public Command handoffManual() {
    return new InstantCommand(
            () -> {
              if (manualOverride) {
                handoffSequence().schedule();
              }
            })
        .withName("handoffManual()");
  }

  /* Climber commands */

  /** Run the winch until the limit switch is pressed then set the state to climb. */
  private Command climbSequence() {
    return m_climber
        .retract()
        .until(m_climber::pastClimbSetpoint)
        .beforeStarting(() -> m_state = State.CLIMB)
        .withName("climbSequence()");
  }

  /** Move subsystems out of the way then deploy the climber at setpoint activated. */
  private Command deployClimberSequence() {
    return m_coralIntake
        .stow()
        .alongWith(m_elevator.moveToStow())
        .alongWith(m_dragon.climb())
        .until(() -> m_coralIntake.atSetpoint() && m_dragon.atSetpoint())
        .andThen(m_climber.deploy().until(m_climber::atSetpoint))
        .beforeStarting(() -> m_state = State.CLIMB_READY)
        .withName("deployClimberSequence()");
  }

  /** Move climber back then return subsystems to their stow state. */
  private Command retractClimberSequence() {
    return m_climber
        .retract()
        .until(m_climber::limitSwitchPressed)
        .andThen(setDefaultStates())
        .withName("retractClimberSequence()");
  }

  public Command deployClimber() {
    return new InstantCommand(
            () -> {
              if (m_state == State.IDLE
                  || m_state == State.POOP_STANDBY
                  || m_state == State.DRAGON_STANDBY
                  || m_state == State.CLIMB) {
                deployClimberSequence().schedule();
              }
            })
        .withName("deployClimber()");
  }

  public Command retractClimber() {
    return new InstantCommand(
            () -> {
              if (m_state == State.CLIMB_READY) {
                retractClimberSequence().schedule();
              }
            })
        .withName("retractClimber()");
  }

  public Command climb() {
    return new InstantCommand(
            () -> {
              if (m_state == State.CLIMB_READY) {
                climbSequence().schedule();
              }
            })
        .withName("climb()");
  }

  public Command homingSequence() {
    return m_dragon
        .stow()
        .until(m_dragon::isClearFromElevator)
        .onlyIf(() -> !m_elevator.reverseLimitSwitchPressed())
        .andThen(m_elevator.homingSequence().until(m_elevator::reverseLimitSwitchPressed))
        .andThen(m_climber.retract().until(m_climber::limitSwitchPressed))
        .beforeStarting(() -> elevatorHasReset = true)
        .onlyIf(() -> !elevatorHasReset);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("State Machine/Manual Override", manualOverride);
    SmartDashboard.putBoolean("Elevator homing done?", elevatorHasReset);
    SmartDashboard.putBoolean("State Machine/Auto Handoff", autoHandoff);
    SmartDashboard.putString("State Machine/State", m_state.toString());
    SmartDashboard.putString(
        "State Machine/Current Comamand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
  }
}
