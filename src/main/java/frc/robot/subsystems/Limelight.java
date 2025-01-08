// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase {
  
  private String limelightName = LimelightConstants.kLimelightName;
  private double kCameraHeight = LimelightConstants.kCameraHeight;
  private double kMountingAngle = LimelightConstants.kMountingAngle;
  private double kGoalHeight = LimelightConstants.kGoalHeight;

  public Limelight() {}

  public double getDistanceToGoalInches() {
    return (kGoalHeight - kCameraHeight) / Math.tan(Units.degreesToRadians(kMountingAngle + getYAngleOffsetDegrees()));
  }

  public void setGoalHeight(double GoalHeight) {
		this.kGoalHeight = GoalHeight;
	}
  
  public double getGoalHeight() {
    return kGoalHeight;
  }

  public double getDistanceToGoalMeters() {
		return Units.inchesToMeters(getDistanceToGoalInches());
	}

  // Offset in Degrees
  public double getYAngleOffsetDegrees() {
    return LimelightHelpers.getTY(limelightName);
  }

  public double getXAngleOffsetDegrees() {
    return -1 * LimelightHelpers.getTX(limelightName); // must be negative
  }

  // Offset in Radians
  public double getYOffsetRadians() {
		return Units.degreesToRadians(getYAngleOffsetDegrees());
	}

  public double getXOffsetRadians() {
		return Units.degreesToRadians(getXAngleOffsetDegrees());
	}

  public boolean isTargetVisible() {
		return LimelightHelpers.getTV(limelightName);
	}

  // Pipline Stuff
	public void setAprilTagPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 1);
	}

	public void setAprilTagFarPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 2);
	}
}