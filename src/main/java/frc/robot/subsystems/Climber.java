// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.PivotSetpoints;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;
import frc.robot.utils.TunableNumber;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private enum ClimberSetpoint {
    DEPLOY,
    RETRACT
  }

  public enum ClimberState {
    DEPLOY,
    RETRACT
  }

  // Tunables
  private final TunableNumber tunableAngle, tunableP;
  private SparkFlexConfig tunableConfig = Configs.Climber.pivotConfig;

  private ClimberSetpoint m_climberSetpoint;
  private ClimberState m_climberState;

  private double pivotCurrentTarget = PivotSetpoints.kRetract;

  private SparkFlex pivotMotor = new SparkFlex(ClimberConstants.kPivotMotorCanId, MotorType.kBrushless);
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  private boolean wasResetByLimit = false;


  public Climber() {
    tunableAngle = new TunableNumber("Climber/Tunable Pivot Angle");
    tunableP = new TunableNumber("Climber/Tunable Pivot P");
    tunableAngle.setDefault(0);
    tunableP.setDefault(0);

    pivotMotor.configure(
        Configs.Climber.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_climberSetpoint = ClimberSetpoint.RETRACT;
    m_climberState = ClimberState.RETRACT;

  }

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  /** Set the arm motor position. This will use closed loop position control. */
  private void moveToSetpoint() {
    pivotController.setReference(pivotCurrentTarget, ControlType.kPosition);
  }

  public boolean atSetpoint() {
    if (Robot.isSimulation()) {
      return true;
    }
    return pivotEncoder.getPosition() > ClimberConstants.PivotSetpoints.kMaxAngle;
  }

  public boolean limitSwitchPressed() {
    return pivotMotor.getReverseLimitSwitch().isPressed();
  } 

  private void setClimberSetpoint(ClimberSetpoint setpoint) {
    m_climberSetpoint = setpoint;
  }

  private void setClimberState(ClimberState state) {
    m_climberState = state;
  }

  private void setPivot(ClimberSetpoint setpoint) {
    setClimberSetpoint(setpoint);
    switch (m_climberSetpoint) {
      case DEPLOY:
        pivotCurrentTarget = PivotSetpoints.kDeploy;
        break;
      case RETRACT:
        pivotCurrentTarget = PivotSetpoints.kRetract;
        break;
    }
    moveToSetpoint();
  }

  public Command deploy() {
    return this.runEnd(() -> {
      pivotMotor.set(0.5);
      setClimberState(ClimberState.DEPLOY);
    }, () -> {
      pivotMotor.stopMotor();
    });
  }

  public Command retract() {
    return this.runEnd(() -> {
      pivotMotor.set(-0.5);
      setClimberState(ClimberState.RETRACT);
    }, () -> {
      pivotMotor.stopMotor();
    });
  }
  
  public ClimberSetpoint getSetpoint() {
    return m_climberSetpoint;
  }

  public ClimberState getState() {
    return m_climberState;
  }

  private void zeroClimbOnLimitSwitch() {
    if (!wasResetByLimit && pivotMotor.getReverseLimitSwitch().isPressed()) 
    {
      pivotEncoder.setPosition(PivotSetpoints.kRetract);
      wasResetByLimit = true;
    }
    else if(!pivotMotor.getReverseLimitSwitch().isPressed()) 
    {
      wasResetByLimit = false;  
    }

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber/Pivot/Current Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Climber/Pivot/Setpoint", pivotCurrentTarget);

    SmartDashboard.putString("Climber/Climber State", m_climberState.toString());
    SmartDashboard.putBoolean("Climber/Pivot/at Setpoint?", atSetpoint());

    SmartDashboard.putString("Climber/State", m_climberState.toString());
    SmartDashboard.putString("Climber/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");

    zeroClimbOnLimitSwitch();

    // Tunable If's
    if (tunableAngle.hasChanged()) {
      pivotCurrentTarget = tunableAngle.get();
      moveToSetpoint();
    }
    if (tunableP.hasChanged()) {
      tunableConfig.closedLoop.p(tunableP.get());
      pivotMotor.configure(tunableConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}
