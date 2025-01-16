// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase {
  
  private String m_limelightName;
  private double m_cameraHeight;
  private double m_mountingAngle;
  private double m_goalHeight;

  public Limelight(String m_limelightName, double m_cameraHeight, double m_mountingAngle, double m_goalHeight) {

    this.m_limelightName = m_limelightName;
    this.m_cameraHeight = m_cameraHeight;
    this.m_mountingAngle = m_mountingAngle;
    this.m_goalHeight = m_goalHeight;

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

  // Pipline Stuff
	public void setCoralTagPipelineRight() {
		LimelightHelpers.setPipelineIndex(m_limelightName, 1);
	}

	public void setCoralTagPipelineLeft() {
		LimelightHelpers.setPipelineIndex(m_limelightName, 2);
	}

  public int getTargetID()
  {
    return (int)LimelightHelpers.getFiducialID(m_limelightName);
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
