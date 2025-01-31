// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.superstructure.StateMachine.limelightState;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.superstructure.StateMachine.limelightState;


public class Limelight extends SubsystemBase {
  
  private String m_limelightName;
  private double m_cameraHeight;
  private double m_mountingAngle;
  private double m_goalHeight;

  private int targetID;
  private Align branchSide;
  private int [] allianceSideId;
  

  public enum Align
  {
    LEFT,
    RIGHT
  }

  public Limelight(String m_limelightName, double m_cameraHeight, double m_mountingAngle, double m_goalHeight) {

    this.m_limelightName = m_limelightName;
    this.m_cameraHeight = m_cameraHeight;
    this.m_mountingAngle = m_mountingAngle;
    this.m_goalHeight = m_goalHeight;

    allianceSideId = new int[6];

  }

  public String getName() {
    return m_limelightName;
  }

  public double getDistanceToGoalInches() {
    return (m_goalHeight - m_cameraHeight) / Math.tan(Units.degreesToRadians(m_mountingAngle + getYAngleOffsetDegrees()));
  }

  public void setGoalHeight(double goalHeight) {
		this.m_goalHeight = goalHeight;
	}
  
  public double getGoalHeight() {
    return m_goalHeight;
  }

  public double getDistanceToGoalMeters() {
		return Units.inchesToMeters(getDistanceToGoalInches());
	}

  // Offset in Degrees
  public double getYAngleOffsetDegrees() {
    return LimelightHelpers.getTY(m_limelightName);
  }

  public double getXAngleOffsetDegrees() {
    return LimelightHelpers.getTX(m_limelightName); 
  }

  public double getXOffsetRadians() {
		return Units.degreesToRadians(getXAngleOffsetDegrees());
	}

  public boolean isTargetVisible() {
		return LimelightHelpers.getTV(m_limelightName);
	}

  public RawFiducial[] getRawFiducials() {
    return LimelightHelpers.getRawFiducials(m_limelightName);
  }

  public RawFiducial getRawFiducial(double id) {
    for (RawFiducial fiducial : getRawFiducials()) {
      if (fiducial.id == id) {
        return fiducial;
      }
    }
    return null;
  }

  public boolean hasRawFiducial(double id) {
    for (RawFiducial fiducial : getRawFiducials()) {
      if (fiducial.id == id) {
        return true;
      }
    }
    return false;
  }

  public double getRawFiducialDistToCamera(RawFiducial rawFiducial) {
    return rawFiducial.distToCamera;
  }

  public double getRawFiducialTX(RawFiducial rawFiducial) {
    return rawFiducial.txnc;
  }

  public double getRawFiducialTY(RawFiducial rawFiducial) {
    return rawFiducial.tync;
  }

  // Pipline Stuff
	public void setCoralTagPipelineRight() {
		LimelightHelpers.setPipelineIndex(m_limelightName, LimelightConstants.kRightReefBranchPipeline);
	}

	public void setCoralTagPipelineLeft() {
		LimelightHelpers.setPipelineIndex(m_limelightName, LimelightConstants.kLeftReefBranchPipeline);
	}

  public void setProcessorTagPipeline() {
		LimelightHelpers.setPipelineIndex(m_limelightName, LimelightConstants.kProcessorPipeline);
	}

  public int getTargetID()
  {
    return (int)LimelightHelpers.getFiducialID(m_limelightName);
  }

  public Command setLocationCommand(limelightState limelightState)
  {
      if(DriverStation.getAlliance().equals(Alliance.Red)) //if side is red, the array should be for red
      {
        allianceSideId = LimelightConstants.redSideId;
      }
      else if(DriverStation.getAlliance().equals(Alliance.Blue)) //if side is blue, the array should be blue
      {
        allianceSideId = LimelightConstants.blueSideId;
      }

      return this.runOnce(() -> {
        switch(limelightState)
        {
          case SIDE1RIGHT:
            targetID = allianceSideId[0];
            branchSide = Align.RIGHT;
            break;

          case SIDE2LEFT:
            targetID = allianceSideId[1];
            branchSide = Align.LEFT;
            break;
          
          case SIDE2RIGHT:
            targetID = allianceSideId[1];
            branchSide = Align.RIGHT;
            break;

          case SIDE3LEFT:
            targetID = allianceSideId[2];
            branchSide = Align.LEFT;
            break;
          
          case SIDE3RIGHT:
            targetID = allianceSideId[2];
            branchSide = Align.RIGHT;
            break;

          case SIDE4LEFT:
            targetID = allianceSideId[3];
            branchSide = Align.LEFT;
            break;

          case SIDE4RIGHT:
            targetID = allianceSideId[3];
            branchSide = Align.RIGHT;
            break;

          case SIDE5LEFT:
            targetID = allianceSideId[4];
            branchSide = Align.LEFT;
            break;

          case SIDE5RIGHT:
            targetID = allianceSideId[4];
            branchSide = Align.RIGHT;
            break;
          
          case SIDE6LEFT:
            targetID = allianceSideId[5];
            branchSide = Align.LEFT;
            break;
          
          case SIDE6RIGHT:
            targetID = allianceSideId[5];
            branchSide = Align.RIGHT;
            break;
          
          case SIDE1LEFT:
            targetID = allianceSideId[0];
            branchSide = Align.LEFT;
            break;
        }
        });

  }

  public Align getBranchSide()
  {
    return branchSide;
  }
  


  @Override
  public void periodic()
  {
    //SmartDashboard.putNumber("distance to goal", getDistanceToGoalMeters());
    //SmartDashboard.putString("asdfas", "m_limelightName");
    SmartDashboard.putNumber("X offset", getXAngleOffsetDegrees());
    SmartDashboard.putNumber("Y offset", getYAngleOffsetDegrees());
    SmartDashboard.putNumber("distance to goal", getDistanceToGoalMeters());

  }
  
}
