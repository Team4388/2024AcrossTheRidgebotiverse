// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

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
  }

  public void climbOut() {
    climbMotor.set(Constants.ClimbConstants.CLIMB_OUT_SPEED);
  }

  public void climbIn() {
    climbMotor.set(Constants.ClimbConstants.CLIMB_IN_SPEED);
  }

  public void stopClimb() {
    climbMotor.set(0.d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
