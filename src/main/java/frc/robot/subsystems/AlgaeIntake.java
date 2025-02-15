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
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.AlgaeIntakeConstants.PivotSetpoints;
import frc.robot.Constants.AlgaeIntakeConstants.RollerSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Robot;
import frc.robot.utils.TunableNumber;

public class AlgaeIntake extends SubsystemBase {
  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so
  // we also need to
  // initialize the closed loop controller and encoder.

  private enum AlgaeIntakeSetpoint {
    STOW,
    INTAKE,
    EXTAKE
  }

  public enum AlgaeIntakeState {
    STOW,
    INTAKE,
    EXTAKE
  }

  // Tunable Number
  private final TunableNumber tunableAngle, tunableP;
  private SparkFlexConfig tunableConfig = Configs.AlgaeIntake.pivotConfig;

  private AlgaeIntakeSetpoint m_algaeIntakeSetpoint;
  private AlgaeIntakeState m_algaeIntakeState;

  private double pivotCurrentTarget = PivotSetpoints.kStow;

  private SparkFlex pivotMotor = new SparkFlex(AlgaeIntakeConstants.kPivotMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private SparkFlex rollerMotor = new SparkFlex(AlgaeIntakeConstants.kRollerMotorCanId, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  // Simulation setup and variables
  private DCMotor armMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim armMotorSim;
  private final SingleJointedArmSim m_intakeSim = new SingleJointedArmSim(
      armMotorModel,
      SimulationRobotConstants.kIntakeReduction,
      SingleJointedArmSim.estimateMOI(
          SimulationRobotConstants.kIntakeLength, SimulationRobotConstants.kIntakeMass),
      SimulationRobotConstants.kIntakeLength,
      SimulationRobotConstants.kIntakeMinAngleRads,
      SimulationRobotConstants.kIntakeMaxAngleRads,
      true,
      SimulationRobotConstants.kIntakeMinAngleRads,
      0.0,
      0.0);

  // Mechanism2d setup for subsytem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Ball Intake Root", 25.2, 0);
  private final MechanismLigament2d intakePivotMechanism = m_mech2dRoot.append(
      new MechanismLigament2d(
          "Intake Pivot",
          SimulationRobotConstants.kIntakeShortBarLength
              * SimulationRobotConstants.kPixelsPerMeter,
          Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)));

  @SuppressWarnings("unused")
  private final MechanismLigament2d intakePivotSecondMechanism = intakePivotMechanism.append(
      new MechanismLigament2d(
          "Intake Pivot Second Bar",
          SimulationRobotConstants.kIntakeLongBarLength
              * SimulationRobotConstants.kPixelsPerMeter,
          Units.radiansToDegrees(SimulationRobotConstants.kIntakeBarAngleRads)));

  public AlgaeIntake() {
    tunableAngle = new TunableNumber("TunableAlgaePivot");
    tunableP = new TunableNumber("TunableAlgaePivot P");
    tunableAngle.setDefault(0);
    tunableP.setDefault(0);
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
    // pivotMotor = new SparkFlex(0, MotorType.kBrushless); //CANID later
    // rollerMotor = new SparkFlex(1, MotorType.kBrushless); //CANID later

    // pivotController = pivotMotor.getClosedLoopController();
    // pivotReference = 0;

    // pivotEncoder = pivotMotor.getAbsoluteEncoder();

    rollerMotor.configure(
        Configs.AlgaeIntake.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotMotor.configure(
        Configs.AlgaeIntake.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_algaeIntakeState = AlgaeIntakeState.STOW;
    m_algaeIntakeSetpoint = AlgaeIntakeSetpoint.STOW;

    // Display mechanism2d
    SmartDashboard.putData("Algae Subsystem", m_mech2d);

    // Initialize Simulation values
    armMotorSim = new SparkFlexSim(pivotMotor, armMotorModel);
  }

  public double getPivotPosition() {
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
    return Math.abs(pivotCurrentTarget - pivotEncoder.getPosition()) <= AlgaeIntakeConstants.kPivotThreshold;
  }

  private void setAlgaeIntakeSetpoint(AlgaeIntakeSetpoint setpoint) {
    m_algaeIntakeSetpoint = setpoint;
  }

  private void setAlgaeIntakeState(AlgaeIntakeState state) {
    m_algaeIntakeState = state;
  }

  private void setPivot(AlgaeIntakeSetpoint setpoint) {
    setAlgaeIntakeSetpoint(setpoint);
    switch (m_algaeIntakeSetpoint) {
      case STOW:
        pivotCurrentTarget = PivotSetpoints.kStow;
        break;
      case INTAKE:
        pivotCurrentTarget = PivotSetpoints.kIntake;
        break;
      case EXTAKE:
        pivotCurrentTarget = PivotSetpoints.kExtake;
        break;
    }
    moveToSetpoint();
  }

  /** Set the pivot motor power in the range of [-1, 1]. */
  private void setRollerPower(double power) {
    rollerMotor.set(power);
  }

  public Command intake() {
    return this.runEnd(() -> {
      setPivot(AlgaeIntakeSetpoint.INTAKE);
      setRollerPower(RollerSetpoints.kIntake);
      setAlgaeIntakeState(AlgaeIntakeState.INTAKE);
    }, () -> {
      setPivot(AlgaeIntakeSetpoint.STOW);
      setRollerPower(RollerSetpoints.kStop);
    });
  }

  public Command extake() {
    return this.runEnd(() -> {
      setPivot(AlgaeIntakeSetpoint.EXTAKE);
      setRollerPower(RollerSetpoints.kExtake);
      setAlgaeIntakeState(AlgaeIntakeState.EXTAKE);
    }, () -> {
      setPivot(AlgaeIntakeSetpoint.STOW);
      setRollerPower(RollerSetpoints.kStop);
    });
  }

  public Command stow() {
    return this.run(() -> {
      setPivot(AlgaeIntakeSetpoint.STOW);
      setRollerPower(RollerSetpoints.kStop);
      setAlgaeIntakeState(AlgaeIntakeState.STOW);
    });
  }

  public AlgaeIntakeSetpoint getSetpoint() {
    return m_algaeIntakeSetpoint;
  }

  public AlgaeIntakeState getState() {
    return m_algaeIntakeState;
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
    SmartDashboard.putNumber("Algae/Arm/Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Algae/Intake/Applied Output", rollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Algae/Arm/Pivot setpoint", pivotCurrentTarget);

    SmartDashboard.putString("Algae Intake State", m_algaeIntakeState.toString());
    SmartDashboard.putBoolean("Algae Pivot at Setpoint?", atSetpoint());

    // Update mechanism2d
    intakePivotMechanism.setAngle(
        Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)
            + Units.rotationsToDegrees(
                pivotEncoder.getPosition() / SimulationRobotConstants.kIntakeReduction));


    if (tunableAngle.hasChanged()) {
      pivotCurrentTarget = tunableAngle.get();
      moveToSetpoint();
    }
    if (tunableP.hasChanged()) {
      tunableConfig.closedLoop.p(tunableP.get());
      pivotMotor.configure(tunableConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
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