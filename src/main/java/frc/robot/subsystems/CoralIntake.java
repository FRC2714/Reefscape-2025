// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.CoralIntakeConstants.PivotSetpoints;
import frc.robot.Constants.CoralIntakeConstants.RollerSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Robot;

public class CoralIntake extends SubsystemBase {
  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so
  // we also need to
  // initialize the closed loop controller and encoder.

  private enum CoralIntakeSetpoint {
    STOW,
    HANDOFF,
    INTAKE,
    EXTAKE,
    POOP,
    CLIMB
  }

  public enum CoralIntakeState {
    STOW,
    INTAKE_READY,
    INTAKE,
    EXTAKE_READY,
    EXTAKE,
    HANDOFF_READY,
    HANDOFF,
    POOP_READY,
    POOP_SCORE,
    CLIMB,
  }

  private CoralIntakeSetpoint m_coralIntakeSetpoint;
  private CoralIntakeState m_coralIntakeState;

  private double pivotCurrentTarget = PivotSetpoints.kStow;

  private boolean loaded;

  private SparkFlex pivotMotor = new SparkFlex(CoralIntakeConstants.kPivotMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private SparkFlex rollerMotor = new SparkFlex(CoralIntakeConstants.kRollerMotorCanId, MotorType.kBrushless);
  private SparkFlex indexerMotor = new SparkFlex(CoralIntakeConstants.kIndexerMotorCanId, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  private DigitalInput beamBreak = new DigitalInput(CoralIntakeConstants.kBeamBreakDioChannel);

  // Simulation setup and variables
  private DCMotor armMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim armMotorSim;
  private final SingleJointedArmSim m_intakeSim = new SingleJointedArmSim(
      armMotorModel,
      SimulationRobotConstants.kIntakeReduction,
      SingleJointedArmSim.estimateMOI(
          SimulationRobotConstants.kIntakeLength, SimulationRobotConstants.kIntakeMass),
      SimulationRobotConstants.kCoralIntakeLength,
      SimulationRobotConstants.kCoralIntakeMinAngleRads,
      SimulationRobotConstants.kCoralIntakeMaxAngleRads,
      true,
      SimulationRobotConstants.kCoralIntakeMinAngleRads,
      0.0,
      0.0);

  // Mechanism2d setup for subsytem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Coral Intake Root", 25.1, 0);
  private final MechanismLigament2d coralStand = m_mech2dRoot.append(
      new MechanismLigament2d(
          "Coral Stand", SimulationRobotConstants.kCoralStandLength, 90));
  private final MechanismLigament2d intakePivotMechanism = coralStand.append(
      new MechanismLigament2d(
          "Coral Pivot",
          SimulationRobotConstants.kCoralIntakeLength,
          CoralIntakeConstants.PivotSetpoints.kZeroOffsetDegrees));

  public CoralIntake() {
    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */

    rollerMotor.configure(
        Configs.CoralIntake.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    indexerMotor.configure(
        Configs.CoralIntake.indexerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotMotor.configure(
        Configs.CoralIntake.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_coralIntakeSetpoint = CoralIntakeSetpoint.STOW;
    m_coralIntakeState = CoralIntakeState.STOW;

    loaded = false;

    // Display mechanism2d
    SmartDashboard.putData("Coral Intake Mech2d", m_mech2d);

    // Initialize Simulation values
    armMotorSim = new SparkFlexSim(pivotMotor, armMotorModel);
  }

  public double getPosition() {
    return pivotEncoder.getPosition();
  }

  /** Set the arm motor position. This will use closed loop position control. */
  private void moveToSetpoint() {
    pivotController.setReference(pivotCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public boolean atSetpoint() {
    if (Robot.isSimulation()) {
      return true;
    }
    return Math.abs(pivotCurrentTarget - pivotEncoder.getPosition()) <= CoralIntakeConstants.kPivotThreshold;
  }

  private void setCoralIntakeSetpoint(CoralIntakeSetpoint state) {
    m_coralIntakeSetpoint = state;
  }

  private void setCoralIntakeState(CoralIntakeState state) {
    m_coralIntakeState = state;
  }

  private void setPivotPosition(CoralIntakeSetpoint setpoint) {
    setCoralIntakeSetpoint(setpoint);
    switch (m_coralIntakeSetpoint) {
      case STOW:
        pivotCurrentTarget = PivotSetpoints.kStow;
        break;
      case HANDOFF:
        pivotCurrentTarget = PivotSetpoints.kHandoff;
        break;
      case INTAKE:
        pivotCurrentTarget = PivotSetpoints.kIntake;
        break;
      case EXTAKE:
        pivotCurrentTarget = PivotSetpoints.kExtake;
        break;
      case POOP:
        pivotCurrentTarget = PivotSetpoints.kEject;
        break;
      case CLIMB:
        pivotCurrentTarget = PivotSetpoints.kClimb;
        break;
    }
    moveToSetpoint();
  }

  /** Set the pivot motor power in the range of [-1, 1]. */
  private void setRollerPower(double power) {
    rollerMotor.set(power);

    // TODO: Control this separately
    indexerMotor.set(power);
  }

  public Command intake() {
    return intakeReady().until(this::atSetpoint).andThen(
        this.run(() -> {
          setRollerPower(RollerSetpoints.kIntake);
          setCoralIntakeState(CoralIntakeState.INTAKE);
        })).withName("intake");
  }

  public Command intakeReady() {
    return this.run(() -> {
      setPivotPosition(CoralIntakeSetpoint.INTAKE);
      setRollerPower(RollerSetpoints.kStop);
      setCoralIntakeState(CoralIntakeState.INTAKE_READY);
    }).withName("intake ready");
  }

  public Command extakeReady() {
    return this.run(() -> {
      setPivotPosition(CoralIntakeSetpoint.EXTAKE);
      setRollerPower(RollerSetpoints.kStop);
      setCoralIntakeState(CoralIntakeState.EXTAKE_READY);
    }).withName("extake ready");
  }

  public Command extake() {
    return extakeReady().until(this::atSetpoint).andThen(
        this.run(() -> {
          setRollerPower(RollerSetpoints.kExtake);
          setCoralIntakeState(CoralIntakeState.EXTAKE);
        })).withName("extake");
  }

  public Command stow() {
    return this.run(() -> {
      setPivotPosition(CoralIntakeSetpoint.STOW);
      setRollerPower(RollerSetpoints.kStop);
      setCoralIntakeState(CoralIntakeState.STOW);
    }).withName("stow");
  }

  public Command handoffReady() {
    return this.run(() -> {
      setPivotPosition(CoralIntakeSetpoint.HANDOFF);
      setRollerPower(RollerSetpoints.kStop);
      setCoralIntakeState(CoralIntakeState.HANDOFF_READY);
    }).withName("handoff ready");
  }

  public Command handoff() {
    return handoffReady().until(this::atSetpoint).andThen(
        this.run(() -> {
          setRollerPower(RollerSetpoints.kExtake);
          setCoralIntakeState(CoralIntakeState.HANDOFF);
        })).withName("handoff");
  }

  public Command poopReady() {
    return this.run(() -> {
      setPivotPosition(CoralIntakeSetpoint.POOP);
      setRollerPower(RollerSetpoints.kStop);
      setCoralIntakeState(CoralIntakeState.POOP_READY);
    }).withName("poop ready l1");
  }

  public Command poopL1() {
    return poopReady().until(this::atSetpoint).andThen(
        this.run(() -> {
          setRollerPower(RollerSetpoints.kExtake);
          setCoralIntakeState(CoralIntakeState.POOP_SCORE);
        })).withName("poop l1");
  }

  public Command climb(){
    return this.run(() -> {
      setPivotPosition(CoralIntakeSetpoint.CLIMB);
      setRollerPower(RollerSetpoints.kStop);
      setCoralIntakeState(CoralIntakeState.CLIMB);
    }).withName("climb");
  }

  public boolean isLoaded() {
    if (Robot.isSimulation())
      return loaded;
    return !beamBreak.get();
  }

  public void setLoadedTrue() {
    loaded = true;
  }

  public void setLoadedFalse() {
    loaded = false;
  }

  public CoralIntakeSetpoint getSetpoint() {
    return m_coralIntakeSetpoint;
  }

  public CoralIntakeState getState() {
    return m_coralIntakeState;
  }

  /**
   * Command to run when the intake is not actively running. When in the "hold"
   * state, the intake
   * will stay in the "hold" position and run the motor at its "hold" power to
   * hold onto the ball.
   * When in the "stow" state, the intake will stow the arm in the "stow" position
   * and stop the
   * motor.
   */

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("CoralIntake/Pivot/Current Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("CoralIntake/Pivot/Setpoint", pivotCurrentTarget);
    SmartDashboard.putBoolean("CoralIntake/Pivot/at Setpoint?", atSetpoint());
    
    SmartDashboard.putNumber("CoralIntake/Intex/Roller/Applied Output", rollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("CoralIntake/Intex/Indexer/Applied Output", indexerMotor.getAppliedOutput());

    SmartDashboard.putBoolean("CoralIntake/Intex/Beam Break", beamBreak.get());
    SmartDashboard.putBoolean("CoralIntake/Intex/Loaded?", isLoaded());

    SmartDashboard.putString("CoralIntake/State", m_coralIntakeState.toString());
    SmartDashboard.putString("CoralIntake/Current command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "null");

    // Update mechanism2d
    intakePivotMechanism.setAngle(CoralIntakeConstants.PivotSetpoints.kZeroOffsetDegrees + pivotEncoder.getPosition());
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_intakeSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_intakeSim.setInput(armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_intakeSim.update(0.020);

    // Iterate the arm SPARK simulation
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_intakeSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java
  }
}