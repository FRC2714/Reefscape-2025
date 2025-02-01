// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

public class StateMachine extends SubsystemBase {

  public enum CoralState {
    LOADED,
    EMPTY
  }

  private CoralState m_coralState;

  private AlgaeIntake m_algaeIntake;
  private CoralIntake m_coralIntake;
  private Dragon m_dragon;
  private Elevator m_elevator;
  private LED m_blinkin;
  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;
  private Limelight m_backLimelight;

  /** Creates a new Superstructure. */
  public StateMachine(
    AlgaeIntake m_algaeIntake,
    CoralIntake m_coralIntake,
    Dragon m_dragon,
    Elevator m_elevator,
    LED m_blinkin,
    Limelight m_leftLimelight,
    Limelight m_rightLimelight,
    Limelight m_backLimelight) {

    this.m_algaeIntake = m_algaeIntake;
    this.m_coralIntake = m_coralIntake;
    this.m_dragon = m_dragon;
    this.m_elevator = m_elevator;
    this.m_blinkin = m_blinkin;
    this.m_leftLimelight = m_leftLimelight;
    this.m_rightLimelight = m_rightLimelight;
    this.m_backLimelight = m_backLimelight;

    this.m_coralState = CoralState.EMPTY;

  }

  private void setCoralStateLoaded() {
    if (m_coralIntake.isLoaded())
      m_coralState = CoralState.LOADED;
  }

  private void setCoralStateEmpty() {
    m_coralState = CoralState.EMPTY;
  }

  public Command elevator_moveToStow() {
    return new SequentialCommandGroup(
      m_dragon.moveToStow(), 
      m_elevator.moveToStow()
    );
  }

  public Command elevator_moveToHandoff() {
    return new SequentialCommandGroup(
      m_dragon.moveToStow(), 
      m_elevator.moveToHandoff(),
      m_dragon.moveToHandoff()
    );
  }

  public Command elevator_moveToL1() {
    return new SequentialCommandGroup(
      m_dragon.moveToStow(), 
      m_elevator.moveToL1(),
      m_dragon.moveToL1()
    );
  }

  public Command elevator_moveToL2() {
    return new SequentialCommandGroup(
      m_dragon.moveToStow(), 
      m_elevator.moveToL2(),
      m_dragon.moveToL2()
    );
  }

  public Command elevator_moveToL3() {
    return new SequentialCommandGroup(
      m_dragon.moveToStow(), 
      m_elevator.moveToL3(),
      m_dragon.moveToL3()
    );
  }

  public Command elevator_moveToL4() {
    return new SequentialCommandGroup(
      m_dragon.moveToStow(), 
      m_elevator.moveToL4(),
      m_dragon.moveToL4()
    );
  }

  public Command handoffCoral() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          m_dragon.moveToStow(),
          elevator_moveToHandoff(),
          m_dragon.moveToHandoff()
        ),
        m_dragon.intake(),
        m_coralIntake.moveToHandoff()
      ),
      m_coralIntake.extake()
    );
  }

  public Command scoreCoral() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setCoralStateEmpty()),
      m_dragon.extake()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Coral Cargo State", m_coralState.toString());

    setCoralStateLoaded();
  }
}
