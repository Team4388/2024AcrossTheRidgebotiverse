// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeMotor;
  public Intake(CANSparkMax intakeMotor) {
    this.intakeMotor = intakeMotor;
    
  }

  public void spinIntakeMotor() {
    intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
