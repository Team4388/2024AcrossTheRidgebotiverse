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

import frc4388.robot.subsystems.Limelight;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix.motorcontrol.NeutralMode;


public class Shooter extends SubsystemBase {
  
  private TalonFX leftShooter;
  private TalonFX rightShooter;

  private Limelight limelight;

  private int spinMode = 0;
  // 0 = Stop / Coast
  // 1 = Idle
  // 2 = Idle Near Speaker
  // 3 = Spin
  // 4 = SingleSpin

  private double smartDashboardShooterSpeed;

  /** Creates a new Shooter. */
  public Shooter(TalonFX leftTalonFX, TalonFX rightTalonFX, Limelight tmplimelight) {
    leftShooter  = leftTalonFX;
    rightShooter = rightTalonFX;

    limelight = tmplimelight;

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
    spinMode = 4;
  }

  public void singleSpin(double speed) {
    leftShooter.set(speed);
    spinMode = 4;
  }

  public void spin() {
    spin(smartDashboardShooterSpeed);
    spinMode = 3;
  }

  public void spin(double speed) {
    leftShooter.set(-speed);    
    rightShooter.set(-speed);
    spinMode = 3;
  }

  public void spin(double leftSpeed, double rightSpeed) {
    leftShooter.set(leftSpeed);    
    rightShooter.set(-rightSpeed);
    spinMode = 3;
  }

  public void stop() {
    spin(0.d);
    spinMode = 0;
  }

  public void idle() {
    spin(ShooterConstants.SHOOTER_IDLE);
    spinMode = 1;
  }

  public int getMode(){
    return spinMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  //  SmartDashboard.putNumber("Left Shooter RPM", leftShooter.getRotorVelocity().getValue());
    //SmartDashboard.putNumber("Right Shooter RPM", rightShooter.getRotorVelocity().getValue());
    //smartDashboardShooterSpeed = SmartDashboard.getNumber("Shooter Speed", ShooterConstants.SHOOTER_SPEED);

    // If the robot is near the speaker, and is stopped, or idled, set to limelight idle speed.
    // Else if the robot is not near the speaker, then set the speed back to idle.
    // if(limelight.isNearSpeaker() && (spinMode == 0 || spinMode == 1)){
    //   leftShooter.set(-ShooterConstants.SHOOTER_IDLE_LIMELIGHT);    
    //   rightShooter.set(-ShooterConstants.SHOOTER_IDLE_LIMELIGHT);
    //   spinMode = 2;
    // }else if(!limelight.isNearSpeaker() && spinMode == 2){
    //   idle();
    // }
  }
}
