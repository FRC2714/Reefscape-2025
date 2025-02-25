// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final Field2d m_field = new Field2d();

  public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  private Field2d m_fieldPose = new Field2d();

  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    SmartDashboard.putData("Field", m_fieldPose);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getEstimatedPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Override the X feedback
    PPHolonomicDriveController.overrideXFeedback(() -> {
      // Calculate feedback from your custom PID controller
      // override w aligntocoral pid values
      return 0.0;
    });
    // Clear x feedback once autoaling is done
    PPHolonomicDriveController.clearXFeedbackOverride();

    // Override the Y feedback
    PPHolonomicDriveController.overrideYFeedback(() -> {
      // Calculate feedback from your custom PID controller
      return 0.0;
    });
    // Clear the Y feedback override
    PPHolonomicDriveController.clearYFeedbackOverride();

    // Override the rotation feedback
    PPHolonomicDriveController.overrideRotationFeedback(() -> {
      // Calculate feedback from your custom PID controller
      return 0.0;
    });
    // Clear the rotation feedback override
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    // Clear all feedback overrides
    PPHolonomicDriveController.clearFeedbackOverrides();

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics,
        // Rotation2d.fromDegrees(navx.getHeading()),
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        },
        new Pose2d(),
        LimelightConstants.m_stateStdDevs,
        LimelightConstants.m_visionStdDevs);
    m_gyro.reset();
  }

  public SysIdRoutine sysIdDrive() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(5), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageForwardTest(voltage.in(Volts)), null, this));
  }

  public SysIdRoutine sysIdRotation() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(5), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageRotateTest(voltage.in(Volts)), null, this));
  }

  public Command translationalQuasistatic() {
    return new SequentialCommandGroup(
        sysIdDrive().quasistatic(SysIdRoutine.Direction.kForward),
        sysIdDrive().quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command rotationalQuasistatic() {
    return new SequentialCommandGroup(
        sysIdRotation().quasistatic(SysIdRoutine.Direction.kForward),
        sysIdRotation().quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command translationalDynamic() {
    return new SequentialCommandGroup(
        sysIdDrive().dynamic(SysIdRoutine.Direction.kForward),
        sysIdDrive().dynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command rotationalDynamic() {
    return new SequentialCommandGroup(
        sysIdRotation().dynamic(SysIdRoutine.Direction.kForward),
        sysIdRotation().dynamic(SysIdRoutine.Direction.kReverse));
  }

  private void driveVoltageForwardTest(double voltage) {
    var direction = new Rotation2d();
    m_frontLeft.setVoltageAngle(voltage, direction);
    m_frontRight.setVoltageAngle(voltage, direction);
    m_rearLeft.setVoltageAngle(voltage, direction);
    m_rearRight.setVoltageAngle(voltage, direction);
  }

  private void driveVoltageRotateTest(double voltage) {
    m_frontLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(-45.0));
    m_frontRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(45.0));
    m_rearLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(45.0));
    m_rearRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(-45.0));
  }

  public void resetEstimatedHeading(Rotation2d rotation) {
    swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition() },
        new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), rotation));
  }

  public void resetPose(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition() },
        pose);
  }

  public void update() {
    // if (Robot.isSimulation()) {
    //   return;
    // } else if (DriverStation.getAlliance().get().toString().equals("Red")) {
    //   swerveDrivePoseEstimator.update(
    //       Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
    //       new SwerveModulePosition[] {
    //           m_frontLeft.getPositionPoseRed(),
    //           m_frontRight.getPositionPoseRed(),
    //           m_rearLeft.getPositionPoseRed(),
    //           m_rearRight.getPositionPoseRed()
    //       });
    // } else {
    //   swerveDrivePoseEstimator.update(
    //       Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
    //       new SwerveModulePosition[] {
    //           m_frontLeft.getPositionPoseBlue(),
    //           m_frontRight.getPositionPoseBlue(),
    //           m_rearLeft.getPositionPoseBlue(),
    //           m_rearRight.getPositionPoseBlue()
    //       });
    // }
    swerveDrivePoseEstimator.update(
          Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          });

    // boolean useMegaTag2 = true; // set to false to use MegaTag1
    // boolean doRejectUpdate = false;
    // if (useMegaTag2 == false) {
    //   LimelightHelpers.PoseEstimate mt1back = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
    //   LimelightHelpers.PoseEstimate mt1Right = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
    //   LimelightHelpers.PoseEstimate mt1left = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

    //   if (mt1back.tagCount == 1 && mt1back.rawFiducials.length == 1
    //       || mt1Right.tagCount == 1 && mt1Right.rawFiducials.length == 1
    //       || mt1left.tagCount == 1 && mt1left.rawFiducials.length == 1) {
    //     if (mt1back.rawFiducials[0].ambiguity > .7 && mt1Right.rawFiducials[0].ambiguity > .7
    //         && mt1left.rawFiducials[0].ambiguity > .7) {
    //       doRejectUpdate = true;
    //     }
    //     if (mt1back.rawFiducials[0].distToCamera > 3 && mt1Right.rawFiducials[0].distToCamera > 3
    //         && mt1left.rawFiducials[0].distToCamera > 3) {
    //       doRejectUpdate = true;
    //     }
    //   }
    //   if (mt1back.tagCount == 0 && mt1Right.tagCount == 0 && mt1left.tagCount == 0) {
    //     doRejectUpdate = true;
    //   }

    //   if (!doRejectUpdate) {
    //     System.out.println("using mt1");
    //     swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    //     if (mt1back.tagCount > mt1Right.tagCount && mt1back.tagCount > mt1left.tagCount) {
    //       swerveDrivePoseEstimator.addVisionMeasurement(
    //           mt1back.pose,
    //           mt1back.timestampSeconds);
    //     } else if (mt1left.tagCount > mt1Right.tagCount && mt1left.tagCount > mt1Right.tagCount) {
    //       swerveDrivePoseEstimator.addVisionMeasurement(
    //           mt1Right.pose,
    //           mt1Right.timestampSeconds);
    //     } else {
    //       swerveDrivePoseEstimator.addVisionMeasurement(
    //           mt1left.pose,
    //           mt1left.timestampSeconds);
    //     }
    //   }
    // } else if (useMegaTag2 == true) {
    //   LimelightHelpers.PoseEstimate mt2back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
    //   LimelightHelpers.PoseEstimate mt2right = LimelightHelpers
    //       .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    //   LimelightHelpers.PoseEstimate mt2left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

    //   if (Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
    //                                         // vision updates
    //   {
    //     doRejectUpdate = true;
    //   }
    //   if (mt2back == null || mt2left == null || mt2right == null
    //       || mt2back.tagCount == 0 && mt2left.tagCount == 0 && mt2right.tagCount == 0) { // mt2right == null ||
    //     // mt2right.tagCount
    //     // == 0 &&
    //     doRejectUpdate = true;
    //   } else if (mt2back.avgTagDist > 3 && mt2right.avgTagDist > 3
    //       && mt2left.avgTagDist > 3) {
    //     doRejectUpdate = true;
    //   }

    //   if (!doRejectUpdate) {
    //     swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 9999999));
    //     if (mt2left.rawFiducials.length > mt2back.rawFiducials.length
    //         && mt2left.rawFiducials.length > mt2right.rawFiducials.length
    //         && mt2left.tagCount > mt2back.tagCount) { // && mt2left.tagCount >= mt2right.tagCount
    //       System.out.println("update left");
    //       swerveDrivePoseEstimator.addVisionMeasurement(
    //           mt2left.pose,
    //           mt2left.timestampSeconds);
    //     } else if (mt2right.rawFiducials.length > mt2back.rawFiducials.length
    //         && mt2right.rawFiducials.length > mt2left.rawFiducials.length &&
    //         mt2right.tagCount > mt2left.tagCount) {
    //       System.out.println("update right");
    //       swerveDrivePoseEstimator.addVisionMeasurement(
    //           mt2right.pose,
    //           mt2right.timestampSeconds);
    //     } else if (mt2back.rawFiducials.length > 0.1) {
    //       System.out.println("update back");
    //       swerveDrivePoseEstimator.addVisionMeasurement(
    //           mt2back.pose,
    //           mt2back.timestampSeconds);
    //     }
    //   }
    // }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left pipeline", LimelightHelpers.getCurrentPipelineIndex("limelight-left"));
    SmartDashboard.putNumber("right pipeline", LimelightHelpers.getCurrentPipelineIndex("limelight-right"));
    SmartDashboard.putNumber("back pipeline", LimelightHelpers.getCurrentPipelineIndex("limelight-back"));

    // Update the odometry in the periodic block

    SmartDashboard.putNumber("Drive/Pose/Front Left Position", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Drive/Pose/Front Right Position", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("Drive/Pose/Rear Left Position", m_rearLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Drive/Pose/Rear Right Position", m_rearRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("Drive/Pose/Heading", getHeading());
    LimelightHelpers.SetRobotOrientation("limelight-back",
        swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right",
        swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-left",
        swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    update();
    m_fieldPose.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */

  public Pose2d getEstimatedPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = speeds.vxMetersPerSecond;
    double ySpeedDelivered = speeds.vyMetersPerSecond;
    double rotDelivered = speeds.omegaRadiansPerSecond;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void flipHeading() {
    m_gyro.setGyroAngle(IMUAxis.kZ, 180);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
}
