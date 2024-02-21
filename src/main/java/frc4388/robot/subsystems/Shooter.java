// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class Shooter extends SubsystemBase {
  
  private TalonFX leftShooter;
  private TalonFX rightShooter;

  private double smartDashboardShooterSpeed;

  /** Creates a new Shooter. */
  public Shooter(TalonFX leftTalonFX, TalonFX rightTalonFX) {
    leftShooter  = leftTalonFX;
    rightShooter = rightTalonFX;

    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);
    SmartDashboard.putNumber("Shooter Speed", ShooterConstants.SHOOTER_SPEED);

  }

  public Shooter(TalonFX leftShooter) {
    this.leftShooter = leftShooter;
    leftShooter.setNeutralMode(NeutralModeValue.Coast);
  }

  public void singleSpin() {
    leftShooter.set(1.0);
  }

  public void singleSpin(double speed) {
    leftShooter.set(speed);
  }

  public void spin() {
    spin(ShooterConstants.SHOOTER_SPEED);
  }

  public void spin(double speed) {
    leftShooter.set(-speed);    
    rightShooter.set(-speed);
  }

  public void spin(double leftSpeed, double rightSpeed) {
    leftShooter.set(leftSpeed);    
    rightShooter.set(-rightSpeed);
  }

  public void stop() {
    spin(0.d);
  }

  public void idle() {
      spin(ShooterConstants.SHOOTER_IDLE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  //  SmartDashboard.putNumber("Left Shooter RPM", leftShooter.getRotorVelocity().getValue());
    //SmartDashboard.putNumber("Right Shooter RPM", rightShooter.getRotorVelocity().getValue());
    smartDashboardShooterSpeed = SmartDashboard.getNumber("Shooter Speed", ShooterConstants.SHOOTER_SPEED);

    
  }
}
