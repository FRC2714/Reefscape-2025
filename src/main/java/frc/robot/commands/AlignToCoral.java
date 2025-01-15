// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCoral extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_rightLimelight;
  private Limelight m_leftLimelight;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  private int pipelineNum;

  public AlignToCoral(DriveSubsystem m_drivetrain, Limelight m_rightLimelight, Limelight m_leftLimelight, int pipelineNum) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_rightLimelight = m_rightLimelight;
    this.m_leftLimelight = m_leftLimelight;
    this.pipelineNum = pipelineNum;


    xController = new PIDController(0.55, 0, 0); //tune these later
    yController = new PIDController(0.25, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);
    
    addRequirements(m_drivetrain);

    xController.setSetpoint(.23);
    yController.setSetpoint(0);
    thetaController.setSetpoint(0);
    thetaController.enableContinuousInput(-180, 180);

    xController.setTolerance(.2);
    yController.setTolerance(.2);
    thetaController.setTolerance(.1);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(pipelineNum == 1)
    {
       m_rightLimelight.setCoralTagPipelineRight();
       m_leftLimelight.setCoralTagPipelineRight();
    }
    else if(pipelineNum == 2)
    {
      m_rightLimelight.setCoralTagPipelineLeft();
      m_leftLimelight.setCoralTagPipelineLeft();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_leftLimelight.isTargetVisible()) {
      switch(m_leftLimelight.getTargetID()) {
        case 6:
        case 19:
          thetaController.setSetpoint(300);
          break;

        case 7:
        case 18: 
          thetaController.setSetpoint(0);
          break;

        case 8:
        case 17:
          thetaController.setSetpoint(60);
          break;

        case 9:
        case 16:
          thetaController.setSetpoint(120);
          break;

        case 10:
        case 15: 
          thetaController.setSetpoint(180);
          break;

        case 11:
        case 14:
          thetaController.setSetpoint(240);
          break;
      }
        m_drivetrain.drive(-xController.calculate(m_leftLimelight.getDistanceToGoalMeters()),
         yController.calculate(m_leftLimelight.getXOffsetRadians()),
          thetaController.calculate(m_drivetrain.getHeading()), 
          false);
    }
    else if(m_rightLimelight.isTargetVisible()) {
      switch(m_rightLimelight.getTargetID()) {
        case 6:
        case 19:
          thetaController.setSetpoint(300);
          break;

        case 7:
        case 18: 
          thetaController.setSetpoint(0);
          break;

        case 8:
        case 17:
          thetaController.setSetpoint(60);
          break;

        case 9:
        case 16:
          thetaController.setSetpoint(120);
          break;

        case 10:
        case 15: 
          thetaController.setSetpoint(180);
          break;

        case 11:
        case 14:
          thetaController.setSetpoint(240);
          break;
      }
      m_drivetrain.drive(-xController.calculate(m_rightLimelight.getDistanceToGoalMeters()),
         yController.calculate(m_rightLimelight.getXOffsetRadians()),
          thetaController.calculate(m_drivetrain.getHeading()), 
          false);
    }
    else if(m_leftLimelight.isTargetVisible() && m_rightLimelight.isTargetVisible()) {
      if (m_leftLimelight.getDistanceToGoalMeters() > m_rightLimelight.getDistanceToGoalMeters()) {
        switch(m_rightLimelight.getTargetID()) {
          case 6:
          case 19:
            thetaController.setSetpoint(300);
            break;
  
          case 7:
          case 18: 
            thetaController.setSetpoint(0);
            break;
  
          case 8:
          case 17:
            thetaController.setSetpoint(60);
            break;
  
          case 9:
          case 16:
            thetaController.setSetpoint(120);
            break;
  
          case 10:
          case 15: 
            thetaController.setSetpoint(180);
            break;
  
          case 11:
          case 14:
            thetaController.setSetpoint(240);
            break;
        }
        m_drivetrain.drive(-xController.calculate(m_rightLimelight.getDistanceToGoalMeters()),
         yController.calculate(m_rightLimelight.getXOffsetRadians()),
          thetaController.calculate(m_drivetrain.getHeading()), 
          false);
      }
      else {
        switch(m_leftLimelight.getTargetID()) {
          case 6:
          case 19:
            thetaController.setSetpoint(300);
            break;
  
          case 7:
          case 18: 
            thetaController.setSetpoint(0);
            break;
  
          case 8:
          case 17:
            thetaController.setSetpoint(60);
            break;
  
          case 9:
          case 16:
            thetaController.setSetpoint(120);
            break;
  
          case 10:
          case 15: 
            thetaController.setSetpoint(180);
            break;
  
          case 11:
          case 14:
            thetaController.setSetpoint(240);
            break;
        }
        m_drivetrain.drive(-xController.calculate(m_rightLimelight.getDistanceToGoalMeters()),
         yController.calculate(m_rightLimelight.getXOffsetRadians()),
          thetaController.calculate(m_drivetrain.getHeading()), 
          false);
      }
    }
    else {
      m_drivetrain.drive(0, 0, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
