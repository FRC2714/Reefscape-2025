// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dragon.DragonSetpoint;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import java.util.HashMap;
import java.util.List;

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

  private List<Integer> lowStalks = List.of(1, 2, 5, 6, 9, 10);

  private int m_stalk;

  public enum ScoreLevel {
    L1,
    L2,
    L3,
    L4,
    ALGAE_HIGH,
    ALGAE_LOW
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

    this.m_stalk = 1;

    manualOverride = false;
    autoHandoff = false;

    populateElevatorMap();
    populateDragonMap();
  }

  private void populateElevatorMap() {
    elevatorMap.put(ScoreLevel.L1, ElevatorSetpoint.L1);
    elevatorMap.put(ScoreLevel.L2, ElevatorSetpoint.L2);
    elevatorMap.put(ScoreLevel.L3, ElevatorSetpoint.L3);
    elevatorMap.put(ScoreLevel.L4, ElevatorSetpoint.L4);
    elevatorMap.put(ScoreLevel.ALGAE_HIGH, ElevatorSetpoint.ALGAE_HIGH);
    elevatorMap.put(ScoreLevel.ALGAE_LOW, ElevatorSetpoint.ALGAE_LOW);
  }

  private void populateDragonMap() {
    dragonMap.put(ScoreLevel.L1, DragonSetpoint.L1);
    dragonMap.put(ScoreLevel.L2, DragonSetpoint.L2);
    dragonMap.put(ScoreLevel.L3, DragonSetpoint.L3);
    dragonMap.put(ScoreLevel.L4, DragonSetpoint.L4);
    dragonMap.put(ScoreLevel.ALGAE_HIGH, DragonSetpoint.ALGAE_HIGH);
    dragonMap.put(ScoreLevel.ALGAE_LOW, DragonSetpoint.ALGAE_LOW);
  }

  public void setReefStalkNumber(int stalk) {
    m_stalk = stalk;
  }

  public Command enableManualOverride() {
    return new InstantCommand(() -> manualOverride = true).ignoringDisable(true);
  }

  public Command disableManualOverride() {
    return new InstantCommand(() -> manualOverride = false).ignoringDisable(true);
  }

  public void setAutoHandoff(boolean enable) {
    autoHandoff = enable;
  }

  public Command enableAutoHandoff() {
    return new InstantCommand(() -> autoHandoff = true).ignoringDisable(true);
  }

  public Command disableAutoHandoff() {
    return new InstantCommand(() -> autoHandoff = false).ignoringDisable(true);
  }

  public Command setTeleOpDefaultStates() {
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
          m_coralIntake
              .stow()
              .until(m_coralIntake::atSetpoint)
              .alongWith(dragonStandbySequence())
              .schedule();
        });
  }

  public Command setAutonomousSetup() {
    return new InstantCommand(
        () -> {
          m_coralIntake
              .stow()
              .until(m_coralIntake::atSetpoint)
              .andThen(m_dragon.moveToStartingConfig())
              .schedule();
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
        .andThen(
            m_dragon
                .stow()
                .until(m_dragon::isClearFromReef)
                .alongWith(m_coralIntake.intakeReady().until(m_coralIntake::atSetpoint)))
        .andThen(() -> m_state = State.IDLE);
  }

  private Command oneCoralBetweenIntakeSequence() {
    return m_coralIntake
        .coralBetween()
        .until(m_coralIntake::isLoaded)
        .alongWith(m_dragon.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint))
        .andThen(
            new ConditionalCommand(
                handoffSequence(),
                poopStandbySequence(),
                () -> autoHandoff || DriverStation.isAutonomous()))
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  public Command intakeAndContinueSequence() {
    return intakeSequence()
        .andThen(
            new ConditionalCommand(
                handoffSequence(),
                poopStandbySequence(),
                () -> autoHandoff || DriverStation.isAutonomous()))
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  public Command intakeSequence() {
    return (m_coralIntake
            .intake()
            .until(m_coralIntake::isLoaded)
            .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint)))
        .alongWith(m_dragon.handoffReady().until(m_dragon::atSetpoint))
        .beforeStarting(() -> m_state = State.INTAKE);
  }

  public Command intakeSequenceAuto() {
    return (m_coralIntake
            .intake()
            .onlyIf(() -> !m_coralIntake.isLoaded())
            .until(m_coralIntake::isLoaded)
            .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint)))
        .alongWith(m_dragon.handoffReady().until(m_dragon::atSetpoint))
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
    return ((m_dragon
                .stow()
                .onlyIf(() -> !m_elevator.atSetpoint())
                .until(m_dragon::isClearFromElevator)
                .andThen(m_elevator.moveToHandoff().until(m_elevator::isClearToStowDragon))
                .andThen(m_dragon.handoffReady())
                .until(m_dragon::atSetpoint))
            .alongWith(
                m_coralIntake
                    .takeLaxative()
                    .andThen(m_coralIntake.handoffReady().until(m_coralIntake::atSetpoint))))
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
    return new InstantCommand(() -> m_level = ScoreLevel.L4)
        .andThen(
            m_dragon
                .stow()
                .onlyIf(() -> !m_elevator.atSetpoint() || level == ScoreLevel.L4)
                .until(m_dragon::isClearFromReef))
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
                  .until(m_dragon::isClearFromReef)
                  .andThen(
                      removeAlgae(
                          ScoreLevel
                              .ALGAE_HIGH)) // Default to higher removal to not break anything in
                  // case of manual override
                  .beforeStarting(() -> m_state = State.DRAGON_READY)
                  .schedule();
            } else {
              m_dragon
                  .stopScore()
                  .until(m_dragon::isClearFromReef)
                  .beforeStarting(() -> m_state = State.DRAGON_READY)
                  .schedule();
            }
          else if (m_state == State.POOP_SCORE)
            m_coralIntake
                .stopPoopL1()
                .until(m_coralIntake::atSetpoint)
                .beforeStarting(() -> m_state = State.POOP_READY)
                .schedule();
        });
  }

  public Command stopScoreAuto() {
    return new ConditionalCommand(
        m_dragon
            .retract()
            .until(m_dragon::atSetpoint)
            .beforeStarting(() -> m_state = State.DRAGON_READY),
        m_dragon
            .stopScore()
            .until(m_dragon::atSetpoint)
            .beforeStarting(() -> m_state = State.DRAGON_READY),
        () -> m_level == ScoreLevel.L4);
  }

  public Command dragonScoreSequence() {
    return m_dragon
        .score()
        .until(() -> !m_dragon.isCoralOnDragon())
        .andThen(stopScore())
        .beforeStarting(() -> m_state = State.DRAGON_SCORE);
  }

  public Command dragonScoreSequenceAuto() {
    return m_dragon
        .score()
        .until(() -> !m_dragon.isCoralOnDragon())
        .andThen(stopScoreAuto())
        .beforeStarting(() -> m_state = State.DRAGON_SCORE);
  }

  public Command dragonScoreL4Sequence() {
    return m_dragon
        .scoreReadyLevel(DragonSetpoint.L4)
        .until(m_dragon::isClearToScoreL4)
        .andThen(m_dragon.score().until(() -> !m_dragon.isCoralOnDragon()))
        .andThen(stopScore())
        .beforeStarting(() -> m_state = State.DRAGON_SCORE);
  }

  public Command dragonScoreL4SequenceAuto() {
    return m_dragon
        .scoreReadyLevel(DragonSetpoint.L4)
        .until(m_dragon::isClearToScoreL4)
        .andThen(m_dragon.score().until(() -> !m_dragon.isCoralOnDragon()))
        .andThen(stopScoreAuto())
        .beforeStarting(() -> m_state = State.DRAGON_SCORE);
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

  public Command algaeRemovalSequence(ScoreLevel level) {
    return m_dragon
        .stow()
        .until(m_dragon::isClearFromReef)
        .andThen(
            m_elevator
                .moveToLevel(elevatorMap.get(level))
                .until(m_elevator::atSetpoint)
                .alongWith(m_dragon.removeAlgae(dragonMap.get(level)).until(m_dragon::atSetpoint)))
        .beforeStarting(() -> m_state = State.ALGAE_REMOVE)
        .withName("algaeRemovalSequence()");
  }

  public Command idle() {
    return new InstantCommand(
            () -> {
              if (manualOverride) {
                idleSequence().schedule();
              } else if (m_state == State.INTAKE
                  || m_state == State.EXTAKE
                  || m_state == State.ALGAE_REMOVE
                  || m_state == State.IDLE) {
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

  public Command removeAlgae(ScoreLevel level) {
    return new InstantCommand(
            () -> {
              if (manualOverride) {
                algaeRemovalSequence(level).schedule();
              } else {
                if (m_state == State.IDLE
                    || m_state == State.DRAGON_READY
                    || m_state == State.DRAGON_STANDBY
                    || m_state == State.ALGAE_REMOVE) {
                  algaeRemovalSequence(
                          lowStalks.contains(m_stalk)
                              ? ScoreLevel.ALGAE_LOW
                              : ScoreLevel.ALGAE_HIGH)
                      .schedule();
                }
              }
            })
        .withName("removeAlgae()");
  }

  public Command intakeCoral() {
    return new InstantCommand(
            () -> {
              if (manualOverride
                  || m_state == State.IDLE
                  || m_state == State.DRAGON_READY
                  || m_state == State.DRAGON_STANDBY) {
                idleSequence()
                    .andThen(intakeAndContinueSequence().onlyIf(() -> !m_dragon.isCoralOnDragon()))
                    .schedule();
              } else if (m_state == State.INTAKE) {
                intakeAndContinueSequence().onlyIf(() -> !m_dragon.isCoralOnDragon()).schedule();
              }
            })
        .withName("intakeCoral()");
  }

  public Command oneCoralBetweenIntake() {
    return new InstantCommand(
            () -> {
              if (manualOverride || m_state == State.IDLE || m_state == State.INTAKE) {
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
    return new InstantCommand(
            () -> {
              m_level = level;
              if (level == ScoreLevel.L1) {
                if (manualOverride
                    || m_state == State.DRAGON_READY
                    || m_state == State.DRAGON_STANDBY
                    || m_state == State.ALGAE_REMOVE) {
                  scoreReadySequence(level).schedule();
                } else if (m_state == State.POOP_STANDBY || m_state == State.POOP_READY) {
                  poopReadySequence().schedule();
                }
              } else if (level == ScoreLevel.L4) {
                if (manualOverride
                    || m_state == State.DRAGON_READY
                    || m_state == State.DRAGON_STANDBY
                    || m_state == State.ALGAE_REMOVE) {
                  scoreReadyL4Sequence(level).schedule();
                } else if ((m_state == State.POOP_STANDBY || m_state == State.POOP_READY)) {
                  handoffSequence().andThen(scoreReadyL4Sequence(level)).schedule();
                }
              } else {
                if (manualOverride
                    || m_state == State.DRAGON_READY
                    || m_state == State.DRAGON_STANDBY
                    || m_state == State.ALGAE_REMOVE) {
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
              if (manualOverride || m_state == State.DRAGON_READY) {
                if (m_level == ScoreLevel.L4) {
                  dragonScoreL4Sequence().schedule();
                } else {
                  dragonScoreSequence().schedule();
                }

              } else if (m_state == State.POOP_READY) {
                poopScoreSequence().schedule();
              }
            })
        .withName("scoreCoral()");
  }

  public Command scoreCoralAuto() {
    return new ConditionalCommand(
            dragonScoreL4SequenceAuto(), dragonScoreSequenceAuto(), () -> m_level == ScoreLevel.L4)
        .withName("scoreCoralAuto()");
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
        .andThen(setTeleOpDefaultStates())
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

  public boolean isReadyToScore() {
    return (m_state == State.DRAGON_READY
            && m_dragon.atSetpoint()
            && m_elevator.atSetpoint()
            && (m_level == ScoreLevel.L4 ? m_dragon.isBranchDetected() : true))
        || (m_state == State.POOP_READY
            && m_coralIntake.atSetpoint()
            && m_dragon.atSetpoint()
            && m_elevator.atSetpoint());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("State Machine/Manual Override", manualOverride);
    SmartDashboard.putBoolean("Elevator homing done?", elevatorHasReset);
    SmartDashboard.putBoolean("State Machine/Auto Handoff", autoHandoff);
    SmartDashboard.putString("State Machine/State", m_state.toString());
    SmartDashboard.putString("State Machine/Level", m_level == null ? "None" : m_level.toString());
    SmartDashboard.putString(
        "State Machine/Current Comamand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
    SmartDashboard.putBoolean("State Machine/Ready to Score", isReadyToScore());

    if (isReadyToScore() && !manualOverride) {
      scoreCoral();
    }
  }
}
