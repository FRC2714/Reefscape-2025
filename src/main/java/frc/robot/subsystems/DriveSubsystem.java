// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase.ControlType;

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
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
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
  // private final SysIdRoutine sysIdRoutine =
  // new SysIdRoutine(
  // // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
  // new SysIdRoutine.Config(),
  // new SysIdRoutine.Mechanism(
  // // Tell SysId how to plumb the driving voltage to the motors.
  // (Voltage volts) -> {
  // m_frontLeft.setVoltageAngle(volts.in(Units.Volts), null);
  // m_frontRight.setVoltageAngle(volts.in(Units.Volts), null);
  // m_rearLeft.setVoltageAngle(volts.in(Units.Volts), null);
  // m_rearRight.setVoltageAngle(volts.in(Units.Volts), null);
  // },
  // log -> {
  // log.motor("frontRight")
  // .voltage(Units.Volts.of(m_frontLeft.getVoltageOutput()))
  // .linearPosition(Units.Meters.of(m_frontRight.getPosition().distanceMeters))
  // .linearVelocity(Units.MetersPerSecond.of(m_frontRight.getState().speedMetersPerSecond));
  // log.motor("frontLeft")
  // .voltage(Units.Volts.of(m_frontLeft.getVoltageOutput()))
  // .linearPosition(Units.Meters.of(m_frontRight.getPosition().distanceMeters))
  // .linearVelocity(Units.MetersPerSecond.of(m_frontLeft.getState().speedMetersPerSecond));
  // log.motor("rearRight")
  // .voltage(Units.Volts.of(m_frontLeft.getVoltageOutput()))
  // .linearPosition(Units.Meters.of(m_frontRight.getPosition().distanceMeters))
  // .linearVelocity(Units.MetersPerSecond.of(m_rearRight.getState().speedMetersPerSecond));
  // log.motor("rearLeft")
  // .voltage(Units.Volts.of(m_frontLeft.getVoltageOutput()))
  // .linearPosition(Units.Meters.of(m_frontRight.getPosition().distanceMeters))
  // .linearVelocity(Units.MetersPerSecond.of(m_rearLeft.getState().speedMetersPerSecond));
  // },
  // this));
  // Create the SysId routine
  SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (voltage) -> this.driveVoltageForwardTest(voltage.in(Volts)),
          null, // No log consumer, since data is recorded by URCL
          this));

  private final SysIdRoutine rotationRoutine;
  private final SysIdRoutine driveRoutine;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

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
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
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
    driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(5), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageForwardTest(voltage.in(Volts)), null, this));
    rotationRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(5), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageRotateTest(voltage.in(Volts)), null, this));

  }

  public SysIdRoutine sysIdDrive() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageForwardTest(voltage.in(Volts)), null, this));
  }

  public SysIdRoutine sysIdRotation() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10),
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageRotateTest(voltage.in(Volts)), null, this));
  }

  public Command translationalQuasistatic() {
    return new SequentialCommandGroup(
        driveRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command rotationalQuasistatic() {
    return new SequentialCommandGroup(
        rotationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        rotationRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command translationalDynamic() {
    return new SequentialCommandGroup(
        driveRoutine.dynamic(SysIdRoutine.Direction.kForward),
        driveRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command rotationalDynamic() {
    return new SequentialCommandGroup(
        rotationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        rotationRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  private void driveVoltageForwardTest(double voltage) {
    m_frontLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(90.0));
    m_frontRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(-180.0));
    m_rearLeft.setVoltageAngle(voltage, Rotation2d.fromDegrees(0.0));
    m_rearRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(90.0));
  }

  private void driveVoltageRotateTest(double voltage) {
    m_frontLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(62.0));
    m_frontRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(27.9));
    m_rearLeft.setVoltageAngle(voltage, Rotation2d.fromDegrees(27.9));
    m_rearRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(62.0));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("Front Left Position", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Front Right Position", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("Rear Left Position", m_rearLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Rear Right Position", m_rearRight.getPosition().distanceMeters);

    m_frontRight.periodic();

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

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

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
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
