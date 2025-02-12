// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.Dragon.DragonState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;

public class StateMachine {

  private Dragon m_dragon;
  private Elevator m_elevator;
  private CoralIntake m_coralIntake;
  private AlgaeIntake m_algaeIntake;
  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;
  private Limelight m_backLimelight;
  private LED m_blinkin;

  /** Creates a new StateMachine. */
  public StateMachine(Dragon m_dragon, Elevator m_elevator, CoralIntake m_coralIntake, AlgaeIntake m_algaeIntake,
      Limelight m_leftLimelight,
      Limelight m_rightLimelight, Limelight m_backLimelight, LED m_blinkin) {
    this.m_algaeIntake = m_algaeIntake;
    this.m_coralIntake = m_coralIntake;
    this.m_dragon = m_dragon;
    this.m_elevator = m_elevator;
    this.m_blinkin = m_blinkin;
    this.m_leftLimelight = m_leftLimelight;
    this.m_rightLimelight = m_rightLimelight;
    this.m_backLimelight = m_backLimelight;
  }

  public Command stowElevator() {
    return m_dragon.stow().until(m_dragon.atSetpoint())
        .andThen(m_elevator.moveToStow());
  }

  public Command setL1() {
    return new ConditionalCommand(
        m_dragon.stow()
            .andThen(m_elevator.moveToL1().until(m_elevator.atSetpoint()))
            .andThen(m_dragon.scoreReadyL1().until(m_dragon.atSetpoint())),
        new ConditionalCommand(
            m_dragon.stow()
                .andThen(m_elevator.moveToPoop()).until(m_elevator.atSetpoint())
                .andThen(m_coralIntake.poopReadyL1().until(m_coralIntake.atSetpoint()))
                .andThen(m_dragon.poopReadyL1().until(m_dragon.atSetpoint())),

            moveElevatorToHandoff()
                .andThen(new InstantCommand(() -> m_elevator.setElevatorSetpoint(ElevatorSetpoint.POOP))),

            () -> (CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
                || CoralIntakeState.POOP_READY == m_coralIntake.getState())),
        m_dragon.isCoralOnDragon());
  }

  public Command setL2() {
    return new ConditionalCommand(
        m_dragon.stow()
            .andThen(m_elevator.moveToL2().until(m_elevator.atSetpoint()))
            .andThen(m_dragon.scoreReadyL2().until(m_dragon.atSetpoint())),

        new ConditionalCommand(
            handoff()
                .andThen(m_dragon.stow().until(m_dragon.atSetpoint()))
                .andThen(m_elevator.moveToL2().until(m_elevator.atSetpoint()))
                .andThen(m_dragon.scoreReadyL2().until(m_dragon.atSetpoint())),

            moveElevatorToHandoff()
                .andThen(new InstantCommand(() -> m_elevator.setElevatorSetpoint(ElevatorSetpoint.L2))),
            () -> (CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
                || CoralIntakeState.POOP_READY == m_coralIntake.getState())),
        m_dragon.isCoralOnDragon());
  }

  public Command setL3() {
    return new ConditionalCommand(
        m_dragon.stow()
            .andThen(m_elevator.moveToL3().until(m_elevator.atSetpoint()))
            .andThen(m_dragon.scoreReadyL3().until(m_dragon.atSetpoint())),

        new ConditionalCommand(
            handoff()
                .andThen(m_dragon.stow().until(m_dragon.atSetpoint()))
                .andThen(m_elevator.moveToL3().until(m_elevator.atSetpoint()))
                .andThen(m_dragon.scoreReadyL3().until(m_dragon.atSetpoint())),

            moveElevatorToHandoff()
                .andThen(new InstantCommand(() -> m_elevator.setElevatorSetpoint(ElevatorSetpoint.L3))),
            () -> (CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
                || CoralIntakeState.POOP_READY == m_coralIntake.getState())),
        m_dragon.isCoralOnDragon());
  }

  public Command setL4() {
    return new ConditionalCommand(
        m_dragon.stow()
            .andThen(m_elevator.moveToL4().until(m_elevator.atSetpoint()))
            .andThen(m_dragon.scoreReadyL4().until(m_dragon.atSetpoint())),

        new ConditionalCommand(
            handoff()
                .andThen(m_dragon.stow().until(m_dragon.atSetpoint()))
                .andThen(m_elevator.moveToL4().until(m_elevator.atSetpoint()))
                .andThen(m_dragon.scoreReadyL4().until(m_dragon.atSetpoint())),

            moveElevatorToHandoff()
                .andThen(new InstantCommand(() -> m_elevator.setElevatorSetpoint(ElevatorSetpoint.L4))),
            () -> (CoralIntakeState.HANDOFF_READY == m_coralIntake.getState()
                || CoralIntakeState.POOP_READY == m_coralIntake.getState())),
        m_dragon.isCoralOnDragon());
  }

  public Command handoff() {
    return m_dragon.stow().until(m_dragon.atSetpoint())
        .andThen(m_elevator.moveToHandoff().until(m_elevator.atSetpoint()))
        .andThen(m_dragon.handoffReady()
            .until(m_dragon.atSetpoint())
            .until(() -> DragonState.HANDOFF_READY == m_dragon.getState()
                && ElevatorState.HANDOFF == m_elevator.getState()))
        .andThen(m_dragon.handoff().until(m_dragon.atSetpoint()))
        .andThen(m_coralIntake.handoff().until(m_coralIntake.atSetpoint()))
        .until(m_dragon.isCoralOnDragon())
        .andThen(m_coralIntake.intakeReady());
  }

  public Command scoreCoral() {
    return new ConditionalCommand(
        m_dragon.score().until(() -> !m_dragon.isCoralOnDragon().getAsBoolean())
            .andThen(m_dragon.stow().until(m_dragon.atSetpoint()))
            .andThen(m_elevator.moveToHandoff().until(m_elevator.atSetpoint()))
            .andThen(m_dragon.handoffReady().until(m_dragon.atSetpoint()))
            .andThen(m_coralIntake.intakeReady()),
        m_coralIntake.poopL1().until(() -> !m_coralIntake.isLoaded())
            .andThen(m_dragon.stow().until(m_dragon.atSetpoint()))
            .andThen(m_elevator.moveToHandoff().until(m_elevator.atSetpoint()))
            .andThen(m_dragon.handoffReady().until(m_dragon.atSetpoint()))
            .andThen(m_coralIntake.intakeReady())
            .onlyIf(() -> (CoralIntakeState.POOP_READY == m_coralIntake.getState()
                && DragonState.POOP_READY == m_dragon.getState() && ElevatorState.POOP == m_elevator.getState())),
        () -> (m_dragon.isCoralOnDragon().getAsBoolean() && DragonState.SCORE_READY == m_dragon.getState()
            && ElevatorState.SCORE_READY == m_elevator.getState()));
  }

  public Command intakeCoral() {
    return new ConditionalCommand(
        intakeSequence(),
        m_coralIntake.intakeReady().until(m_coralIntake.atSetpoint())
            .andThen(intakeSequence()).onlyIf(
                () -> CoralIntakeState.EXTAKE == m_coralIntake.getState()
                    || CoralIntakeState.STOW == m_coralIntake.getState()),
        () -> CoralIntakeState.INTAKE_READY == m_coralIntake.getState());
  }

  private Command intakeSequence() {
    return m_coralIntake.intake()
      .until(m_coralIntake::isLoaded)
      .andThen(m_coralIntake.handoffReady().until(m_coralIntake.atSetpoint()))
      .andThen(
        new SelectCommand<ElevatorSetpoint>(Map.ofEntries(
          Map.entry(ElevatorSetpoint.POOP, setL1()),
          Map.entry(ElevatorSetpoint.L2, setL2()),
          Map.entry(ElevatorSetpoint.L3, setL3()),
          Map.entry(ElevatorSetpoint.L4, setL4())), () -> m_elevator.getSetpoint()));
  }

  public Command setDefaultStates() {
    return m_coralIntake.intakeReady().until(m_coralIntake.atSetpoint())
        .alongWith(m_algaeIntake.stow()).until(m_algaeIntake.atSetpoint())
        .alongWith(
            m_dragon.stow().until(m_dragon.atSetpoint())
                .andThen(m_elevator.moveToHandoff().until(m_elevator.atSetpoint()))
                .andThen(m_dragon.handoffReady().until(m_dragon.atSetpoint())));
  }

  public Command extakeCoral() {
    return m_coralIntake.extake();
  }

  public Command stowCoralIntake() {
    return m_coralIntake.stow();
  }

  public Command intakeAlgae() {
    return m_algaeIntake.intake();
  }

  public Command extakeAlgae() {
    return m_algaeIntake.extake();
  }

  public Command stowAlgae() {
    return m_algaeIntake.stow();
  }

  public Command moveElevatorToHandoff() {
    return m_dragon.stow().until(m_dragon.atSetpoint())
        .andThen(m_elevator.moveToHandoff().until(m_elevator.atSetpoint()))
        .andThen(m_dragon.handoffReady().until(m_dragon.atSetpoint()));
  }

  public Command stow() {
    return stowElevator().until(m_elevator.atSetpoint())
        .alongWith(stowAlgae().until(m_algaeIntake.atSetpoint()))
        .alongWith(stowCoralIntake().until(m_coralIntake.atSetpoint()));
  }
}
