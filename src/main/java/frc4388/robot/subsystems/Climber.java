// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.ClimbConstants;

//! 6.5C Scoring Criteria for Stage

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX climbMotor;

  public Climber(TalonFX climbMotor) {
    this.climbMotor = climbMotor;
    this.climbMotor.setInverted(true);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.7; // An error of 0.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.06; // A velocity of 1 rps results in 0.1 V output

    climbMotor.getConfigurator().apply(slot0Configs);
  }

  public void climbOut() {
    PositionVoltage request = new PositionVoltage(0);
    climbMotor.setControl(request.withPosition(-520)); 
    //climbMotor.set(Constants.ClimbConstants.CLIMB_IN_SPEED);
  }

  public void climbIn() {
    PositionVoltage request = new PositionVoltage(-520);
    climbMotor.setControl(request.withPosition(0)); 
    // climbMotor.set(Constants.ClimbConstants.CLIMB_IN_SPEED);
  }

  public void stopClimb() {
    climbMotor.set(0.d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
