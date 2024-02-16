// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class Shooter extends SubsystemBase {
  
  private TalonFX leftShooter;
  private TalonFX rightShooter;

  private Limelight limelight;

  // 0 = Stop
  // 1 = Idle, no limelight
  // 2 = limelight
  // 3 = Shooting
  private int shooterMode;

  
  /** Creates a new Shooter. */
  public Shooter(TalonFX leftTalonFX, TalonFX rightTalonFX, Limelight limelight) {
    leftShooter  = leftTalonFX;
    rightShooter = rightTalonFX;

    this.limelight = limelight;

    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);
  }

  public void spin() {
    shooterMode = 3;
    spin(ShooterConstants.SHOOTER_SPEED);
  }

  public void spin(double speed) {
    leftShooter.set(-speed);    
    rightShooter.set(speed);
  }

  public void stop() {
    shooterMode = 0;
    spin(0.d);
  }

  public void idle() {
    if(limelight.isNearSpeaker()){
      shooterMode = 2;
      spin(ShooterConstants.SHOOTER_IDLE_LIMELIGHT);
    }else{
      shooterMode = 1;
      spin(ShooterConstants.SHOOTER_IDLE);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(limelight.isNearSpeaker() && shooterMode == 0){
      shooterMode = 1;
      spin(ShooterConstants.SHOOTER_IDLE_LIMELIGHT);
    }

    SmartDashboard.putNumber("Shooter Speed mode", shooterMode);
    SmartDashboard.putNumber("Left Shooter RPM", leftShooter.getRotorVelocity().getValue());
    SmartDashboard.putNumber("Right Shooter RPM", rightShooter.getRotorVelocity().getValue());
    
  }
}
