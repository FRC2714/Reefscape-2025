// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

// ! Update Values for AlphaBot
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;

    public static final boolean kGyroReversed = false;

    public static final double kTurningEncoderPositionFactor = 2 * Math.PI;
    public static final double kDriveEncoderVelocityFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;
  }

  public static final class ClimberConstants {
    public static final int kPivotMotorCanId = 51;

    public static final double kPivotThreshold = 5;

    public static final class PivotSetpoints {
      public static final double kStow = -15;
      public static final double kDeploy = 0;
      public static final double kRetract = -7;
    }
  }

  public static final class AlgaeIntakeConstants {
    public static final int kRollerMotorCanId = 9; // tune ts
    public static final int kPivotMotorCanId = 10; // tune ts

    public static final double kPivotThreshold = 5; // tune

    public static final class PivotSetpoints {
      public static final double kStow = 15; // TBD
      public static final double kIntake = 0; // TBD
      public static final double kExtake = 0;
      public static final double kScore = 7; // TBD (extake position)
      public static final double kClimb = 3;
    }

    public static final class RollerSetpoints {
      public static final double kIntake = 0.5; // tune ts
      public static final double kExtake = -0.5; // tune ts
      public static final double kStop = 0;
    }
  }

  public static final class CoralIntakeConstants {
    public static final int kRollerMotorCanId = 11;
    public static final int kPivotMotorCanId = 12;
    public static final int kIndexerMotorCanId = 13;

    public static final double kPivotThreshold = 5; // tune

    public static final int kBeamBreakDioChannel = 0;

    public static final double kPivotReduction = 1;

    public static final class PivotSetpoints {
      public static final double kStow = 0; // TBD
      public static final double kIntake = 120; // TBD
      public static final double kExtake = 60; // TBD
      public static final double kHandoff = 270; // TBD
      public static final double kEject = 360; // TBD
      public static final double kZeroOffsetDegrees = 270;
      public static final double kClimb = 90;
    }

    public static final class RollerSetpoints {
      public static final double kIntake = 0.5; // tune ts
      public static final double kExtake = -0.5; // tune ts
      public static final double kStop = 0;
    }
  }

  public static final class SimulationRobotConstants {
    // ! Remove *20 from all values for final Robot Testing
    public static final double kPixelsPerMeter = 1 * 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass = 4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = Units.inchesToMeters(35.2 * 20); // m
    public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(66.7 * 20); // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg

    public static final double kPivotReduction = 60; // 60:1
    public static final double kPivotLength = Units.inchesToMeters(10); // m
    public static final double kPivotMass = 4.3; // Kg

    public static final double kMinAngleRads = Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads = Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = Units.inchesToMeters(15); // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);

    public static final double kCoralIntakeMinAngleRads = Units.degreesToRadians(0);
    public static final double kCoralIntakeMaxAngleRads = Units.degreesToRadians(360 * kIntakeReduction);

    public static final double kIntakeShortBarLength = Units.inchesToMeters(11);
    public static final double kIntakeLongBarLength = Units.inchesToMeters(13);
    public static final double kCoralIntakeLength = Units.inchesToMeters(11 * 20);
    public static final double kCoralStandLength = Units.inchesToMeters(35.2 * 20);
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(90);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kTurningMotorReduction = 1;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;

    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // ! Update Values for AlphaBot
  public static final class LimelightConstants {
    public static final Matrix<N3, N1> m_stateStdDevs = VecBuilder.fill(0.15, 0.15, 0.00001); // needs tuning
    public static final Matrix<N3, N1> m_visionStdDevs = VecBuilder.fill(0.00001, 0.00001, 999999); // needs tuning
    public static final String kRightLimelightName = "limelight-right";
    public static final String kLeftLimelightName = "limelight-left";
    public static final String kBackLimelightName = "limelight-back";
    public static final double kRightCameraHeight = 17 + 2.75;
    public static final double kLeftCameraHeight = 17.5 + 2.75;
    public static final double kBackCameraHeight = 2.75 + 15.5; // tune later
    public static final double kRightMountingAngle = -21;
    public static final double kLeftMountingAngle = -23;
    public static final double kBackMountingAngle = 0; // tune later
    public static final double kReefTagHeight = 12;
    public static final double kProcessorTagHeight = 0; // tune later
    public static final int kRightReefBranchPipeline = 1;
    public static final int kLeftReefBranchPipeline = 2;
    public static final int kProcessorPipeline = 0; // TBD

    public static final double kCoralStationDistanceThreshold = 0; // tune
  }

  // ! Update Values for AlphaBot
  public static final class ElevatorConstants {

    public static final int kElevatorMotorCanId = 14; // TBD
    public static final int kElevatorFollowerCanId = 15; // TBD

    public static final double kSetpointThreshold = 5; // tune

    public static final class ElevatorLevels {
      public static final double kStow = 0;
      public static final double kPoop = 0;
      public static final double kHandoff = 90;
      public static final double kLevel1 = 20;
      public static final double kLevel2 = 30;
      public static final double kLevel3 = 100;
      public static final double kLevel4 = 150;
    }
  }

  public static final class DragonConstants {

    public static final int kPivotMotorCanId = 16; // TBD
    public static final int kPivotRollerMotorCanID = 17; // TBD
    public static final double kPivotReduction = 1; // TODO
    public static final double kPivotThreshold = 5; // tune
    public static final double kRollerCurrentThreshold = 5; // tune

    public static final class PivotSetpoints {
      public static final double kStow = 0;
      public static final double kHandoff = 33;
      public static final double kLevel1 = 1;
      public static final double kLevel2 = 2;
      public static final double kLevel3 = 2;
      public static final double kLevel4 = 19;
      public static final double kClimb = 1;
    }

    public static final class RollerSetpoints {
      public static final double kIntake = 0.5; // tune ts (pos when getting from station)
      public static final double kExtake = -0.5; /// tune ts (neg when scoring)
      public static final double kStop = 0;
    }
  }

  public static final class LEDConstants {
    public static final int kBlinkinPort = 2;

    public static final double kBlue = 0.87;
    public static final double kGreen = 0.77;
    public static final double kWhite = 0.93;
    public static final double kYellow = 0.69;
    public static final double kHeartbeatRed = -0.25;
    public static final double kViolet = 0.91;
    public static final double kOrange = 0.65;

  }
};