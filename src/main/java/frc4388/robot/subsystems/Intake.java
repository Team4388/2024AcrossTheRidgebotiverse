// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.utility.Gains;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor;
  private TalonFX pivotMotor;
  public static Gains armGains = IntakeConstants.ArmPID.INTAKE_GAINS;

  /** Creates a new Intake. */
  public Intake(TalonFX intakeMotor, TalonFX pivotMotor) {
    this.intakeMotor = intakeMotor;
    this.pivotMotor = pivotMotor;

    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    pivotMotor.getConfigurator().apply(new TalonFXConfiguration());

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1.3; // An error of 0.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.21; // A velocity of 1 rps results in 0.1 V output

    pivotMotor.getConfigurator().apply(slot0Configs);
  }

  // ! Talon Methods
  public void PIDIn() {
    PIDPosition(0);
  }

  public void PIDOut() {
    PIDPosition(-53);
  }

  public void PIDPosition(double pos) {
    PositionVoltage request = new PositionVoltage(pos);
    pivotMotor.setControl(request);
  }

  public void handoff() {
    intakeMotor.set(-IntakeConstants.INTAKE_OUT_SPEED_UNPRESSED);
  }

  public void spinIntakeMotor() {
    intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  public void spinIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public boolean getIntakeLimitSwitchState() {
    return intakeMotor.getForwardLimit().getValue().value == 0;
  }

  public void setPivotEncoderPosition(double val) {
    pivotMotor.setPosition(val);
  }
  
  public void stopIntakeMotors() {
    intakeMotor.set(0);
  }

  public void stopArmMotor() {
    pivotMotor.set(0);
  }

  public void stop() {
    intakeMotor.set(0);
    pivotMotor.set(0);
  }

  public double getArmPos() {
    return pivotMotor.getPosition().getValue();
  }

  public void resetArmPosition() {
    if (getIntakeLimitSwitchState()) {
     // talonPivot.setPosition(0);
    }
  }

  public void ampPosition() {
    PIDPosition(-59); //TODO: Find actual value
  } 

  public void ampOuttake(double speed) {
    spinIntakeMotor(speed);
  }

  @Override
  public void periodic() {
    resetArmPosition();
  }
}
