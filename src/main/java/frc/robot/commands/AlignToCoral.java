// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Align;
import frc.robot.utils.LimelightHelpers.RawFiducial;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCoral extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_rightLimelight;
  private Limelight m_leftLimelight;
  private Align side;

  // private Limelight m_rightLimelight;
  // private Limelight m_leftLimelight;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;


  public AlignToCoral(DriveSubsystem m_drivetrain, Limelight m_rightLimelight, Limelight m_leftLimelight, Align side) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_rightLimelight = m_rightLimelight;
    this.m_leftLimelight = m_leftLimelight;
    this.side = side;


    xController = new PIDController(0.55, 0, 0); //tune these later
    yController = new PIDController(0.25, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);
    
    addRequirements(m_drivetrain);

    xController.setSetpoint(Units.inchesToMeters(13));
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
    if(side == Align.RIGHT) {
       m_rightLimelight.setCoralTagPipelineRight();
       m_leftLimelight.setCoralTagPipelineRight();
    }
    else if(side == Align.LEFT) {
      m_rightLimelight.setCoralTagPipelineLeft();
      m_leftLimelight.setCoralTagPipelineLeft();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  //when both cameras see:
  //right align = right default (pipelin 1)
  //left align = left default (pipelin 2)
  //if both cameras see, the default camera will be set to whatever side the driver wants to align to (ie. left bumper = left align = default left camera which uses pipeline 1)
  @Override
  public void execute() {
      if(m_leftLimelight.isTargetVisible() && m_rightLimelight.isTargetVisible()) {//if both are visible
        if(m_leftLimelight.getTargetID() == m_rightLimelight.getTargetID()) { // checks if both see same april tag
          if(side == Align.RIGHT) //driver align right so right camera
          {
            updateThetaControllerSetpoint(m_leftLimelight.getTargetID());
    
            m_drivetrain.drive(-xController.calculate(m_leftLimelight.getDistanceToGoalMeters()),
            yController.calculate(m_leftLimelight.getXOffsetRadians()),
            thetaController.calculate(m_drivetrain.getHeading()), 
            false);
          }
          else //driver aligns left so left camera
          {
            updateThetaControllerSetpoint(m_rightLimelight.getTargetID());
    
            m_drivetrain.drive(-xController.calculate(m_rightLimelight.getDistanceToGoalMeters()),
            yController.calculate(m_rightLimelight.getXOffsetRadians()),
            thetaController.calculate(m_drivetrain.getHeading()), 
            false);
          }
        }
        else
        {
          m_drivetrain.drive(0,0,0, true);
        }
      }
      else if((m_leftLimelight.isTargetVisible())) { //if can only see left, then do whatever we did before
        updateThetaControllerSetpoint(m_leftLimelight.getTargetID());
        m_drivetrain.drive(-xController.calculate(m_leftLimelight.getDistanceToGoalMeters()),
          yController.calculate(m_leftLimelight.getXOffsetRadians()),
          thetaController.calculate(m_drivetrain.getHeading()), 
          false);
      }
      else if ((m_rightLimelight.isTargetVisible())) {  //same thing when the camera sees right 
        updateThetaControllerSetpoint(m_rightLimelight.getTargetID());
        m_drivetrain.drive(-xController.calculate(m_rightLimelight.getDistanceToGoalMeters()),
        yController.calculate(m_rightLimelight.getXOffsetRadians()),
        thetaController.calculate(m_drivetrain.getHeading()), 
        false);
      }
      else {
        m_drivetrain.drive(0, 0, 0, true);
      }
}

  private void updateThetaControllerSetpoint(int targetID) {
    switch (targetID) {
      case 6, 19 -> thetaController.setSetpoint(300);
      case 7, 18 -> thetaController.setSetpoint(0);
      case 8, 17 -> thetaController.setSetpoint(60);
      case 9, 16 -> thetaController.setSetpoint(120);
      case 10, 15 -> thetaController.setSetpoint(180);
      case 11, 14 -> thetaController.setSetpoint(240);
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
