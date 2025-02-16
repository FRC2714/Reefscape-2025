// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

  private String m_limelightName;
  private double m_cameraHeight;
  private double m_mountingAngle;
  private double m_goalHeight;
  Field2d m_field = new Field2d();
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("m_limelight");

  public enum Align {
    LEFT,
    RIGHT
  }

  public Limelight(String m_limelightName, double m_cameraHeight, double m_mountingAngle, double m_goalHeight) {

    this.m_limelightName = m_limelightName;
    this.m_cameraHeight = m_cameraHeight;
    this.m_mountingAngle = m_mountingAngle;
    this.m_goalHeight = m_goalHeight;

    SmartDashboard.putData(m_field);

  }

  public String getName() {
    return m_limelightName;
  }

  public double getDistanceToGoalInches() {
    return (m_goalHeight - m_cameraHeight)
        / Math.tan(Units.degreesToRadians(m_mountingAngle + getYAngleOffsetDegrees()));
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
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(m_limelightName, pipeline);
  }

  public boolean coralStationInRange() {
    return getDistanceToGoalMeters() < LimelightConstants.kCoralStationDistanceThreshold;
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

  public static Pose2d getBotPose2d(Limelight limelight) {
    return LimelightHelpers.getBotPose2d_wpiBlue("limelight_back");
  }

  public static Pose3d getBotPose3d(Limelight limelight) {
    return LimelightHelpers.getBotPose3d_wpiBlue("limelight_back");
  }

  public static Pose2d getBotPose2d_TargetSpace(Limelight limelight) {
    return LimelightHelpers.getBotPose3d_TargetSpace("limelight_back").toPose2d();
  }

  public static Pose3d getBotPose3d_TargetSpace(Limelight limelight) {
    return LimelightHelpers.getBotPose3d_TargetSpace("limelight_back");
  }

  public int getTargetID() {
    return (int) LimelightHelpers.getFiducialID(m_limelightName);
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      // LimelightHelpers.setBotPose2d_wpiBlue("limelight_back", new Pose2d(0, 0, new
      // Rotation2d()));
      // LimelightHelpers.setBotPose3d_wpiBlue("limelight_back", new Pose3d(0, 0, 0,
      // new Rotation3d()));
      // LimelightHelpers.setBotPose2d_wpiBlue("limelight_back", new Pose2d(0, 0, new
      // Rotation2d()));
      // LimelightHelpers.setBotPose3d_wpiBlue("limelight_back", new Pose3d(0, 0, 0,
      // new Rotation3d()));
    } else if (DriverStation.getAlliance().get().toString().equals("Red")) {
      setProcessorTagPipeline();
    } else {
      setPipeline(6);
      // SmartDashboard.putNumber("distance to goal", getDistanceToGoalMeters());
      // SmartDashboard.putString("asdfas", "limelight_back");
      SmartDashboard.putNumber("X offset", getXAngleOffsetDegrees());
      SmartDashboard.putNumber("Y offset", getYAngleOffsetDegrees());
      SmartDashboard.putNumber("distance to goal", getDistanceToGoalMeters());
      // Pose2d LLpose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-back");
      // if(LimelightHelpers.getTV("limelight-back")){
      // m_field.setRobotPose(LLpose);
      // }
      // direct odometry editing
    }

  }
}
