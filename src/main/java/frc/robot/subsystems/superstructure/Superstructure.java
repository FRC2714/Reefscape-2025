// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Dragon;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.superstructure.StateMachine.State;

public class Superstructure extends SubsystemBase {

  private AlgaeIntake m_algaeIntake;
  private CoralIntake m_coralIntake;
  private Dragon m_dragon;
  private Elevator m_elevator;
  private LED m_blinkin;
  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;

  /** Creates a new Superstructure. */
  public Superstructure(AlgaeIntake m_algaeIntake,
                      CoralIntake m_coralIntake,
                      Dragon m_dragon,
                      Elevator m_elevator,
                      LED m_blinkin,
                      Limelight m_leftLimelight,
                      Limelight m_rightLimelight) {

    this.m_algaeIntake = m_algaeIntake;
    this.m_coralIntake = m_coralIntake;
    this.m_dragon = m_dragon;
    this.m_elevator = m_elevator;
    this.m_blinkin = m_blinkin;
    this.m_leftLimelight = m_leftLimelight;
    this.m_rightLimelight = m_rightLimelight;

  }

  public Command stowCoralIntake() {
    return m_coralIntake.stowCommand();
  }

  public Command intakeCoral() {
    return m_coralIntake.intakeCommand();
  }

  public Command extakeCoral() {
    return m_coralIntake.extakeCommand();
  }

  public Command handoffCoralIntake() {
    return m_coralIntake.handoffCommand();
  }

  public Command handoffElevator() {
    return m_elevator.setSetpointCommand(State.HANDOFF);
  }

  public Command handoffDragon() {
    return m_dragon.setSetpointCommand(State.HANDOFF);
  }

  public Command stowAlgaeIntake() {
    return m_algaeIntake.stowCommand();
  }

  public Command intakeAlgae() {
    return m_algaeIntake.intakeCommand();
  }

  public Command extakeAlgae() {
    return m_algaeIntake.extakeCommand();
  }

  public Command scoreAlgae() {
    return m_algaeIntake.scoreAlgaeProcessor();
  }

  public Command scoreCoral() {
    m_blinkin.setViolet(); //when it is scoring the LEDs should be purple
    return m_dragon.setRollerPowerCommand();
  }

  public Command setElevatorPosition(State elevatorState) {
    return m_elevator.setSetpointCommand(elevatorState);
  }

  public Command setDragonPosition(State dragonState) {
    return m_dragon.setSetpointCommand(dragonState);
  }

  public boolean readyToRotate()
  {
    if(m_leftLimelight.isTargetVisible() && m_rightLimelight.isTargetVisible()) //if both r able to see
    {
        if(m_leftLimelight.getTargetID() == m_rightLimelight.getTargetID()) //if both see the same tag
        {
          return true;
        }
        return false;
    }
    else if(m_leftLimelight.isTargetVisible()) //if only the left is able to see
    {
      return true;
    }
    else if(m_rightLimelight.isTargetVisible()) //if only the right is able to see
    {
      return true;
    }
    return false;
  }


  @Override
  public void periodic() {
    // This methodwill be called once per scheduler run

    if(readyToRotate()) //if a valid target is seen
    {
      m_blinkin.setYellow();
    }

      

  }

  @Override 
  public void simulationPeriodic() {}
}
