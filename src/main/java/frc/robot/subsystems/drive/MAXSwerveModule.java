// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule extends SubsystemBase {
  private final SparkFlex m_drivingFlex;
  private final SparkFlex m_turningSpark;

  private final DCMotor m_drivingMotorModel;
  private final DCMotor m_turningMotorModel;
  private final SparkFlexSim drivingMotorSim;
  private final SparkFlexSim turningMotorSim;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SimpleMotorFeedforward m_driveFeedforward, m_turningFeedforward;

  private final DCMotorSim m_driveSim, m_turningSim;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkFlex(turningCANId, MotorType.kBrushless);

    m_driveFeedforward = new SimpleMotorFeedforward(0.17, 2.15, 0.30895);
    m_turningFeedforward = new SimpleMotorFeedforward(0.35233, 0.39185, 0.0058658);

    m_drivingMotorModel = DCMotor.getNeoVortex(1);
    m_turningMotorModel = DCMotor.getNeo550(1);

    drivingMotorSim = new SparkFlexSim(m_drivingFlex, m_drivingMotorModel);
    turningMotorSim = new SparkFlexSim(m_turningSpark, m_turningMotorModel);

    m_driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(m_driveFeedforward.getKv(), m_driveFeedforward.getKa()),
        m_drivingMotorModel,
        new double[] { 0.001, 0.001 });

    m_turningSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(m_turningFeedforward.getKv(), m_turningFeedforward.getKa()),
        m_turningMotorModel,
        new double[] { 0.001, 0.001 });

    m_drivingEncoder = m_drivingFlex.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingFlex.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingFlex.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void setVoltageAngle(double voltage, Rotation2d angle) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState desiredState = new SwerveModuleState(0, angle);
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    // m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond,
    // ControlType.kVelocity);
    m_drivingFlex.setVoltage(voltage);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {

    m_driveSim.setInput(drivingMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_driveSim.update(0.020);

    m_turningSim.setInput(turningMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_turningSim.update(0.020);

    drivingMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kDrivingMotorReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    turningMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_turningSim.getAngularVelocityRadPerSec() * ModuleConstants.kTurningMotorReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    SmartDashboard.putNumber("Drive/Sim/drive sim", drivingMotorSim.getAppliedOutput());
    SmartDashboard.putNumber("Drive/Sim/turn sim", turningMotorSim.getAppliedOutput());
  }
}
