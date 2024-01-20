// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class Shooter extends SubsystemBase {
  private WPI_TalonFX leftShooter;
  private WPI_TalonFX rightShooter;

  /** Creates a new Shooter. */
  public Shooter(WPI_TalonFX leftTalonFX, WPI_TalonFX rightTalonFX) {
    leftShooter  = leftTalonFX;
    rightShooter = rightTalonFX;

    leftShooter.setNeutralMode(NeutralMode.Coast);
    rightShooter.setNeutralMode(NeutralMode.Coast);
  }

  public void spin() {
    spin(ShooterConstants.SHOOTER_SPEED);
  }

  public void spin(double speed) {
    leftShooter.set(-speed);    
    rightShooter.set(speed);
  }

  public void stop() {
    spin(0.d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
