// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.commands.PID;
import frc4388.utility.Gains;

public class Intake extends SubsystemBase {
  
  private CANSparkMax intakeMotor;
  private CANSparkMax pivot;
  private SparkPIDController m_spedController;
  private static Gains armGains = IntakeConstants.ArmPID.INTAKE_GAINS;
  private SparkLimitSwitch forwardLimit;
  private SparkLimitSwitch reverseLimit;
  private SparkLimitSwitch intakeforwardLimit;  
  private SparkLimitSwitch intakereverseLimit;

  private Shooter shooter;



  /** Creates a new Intake. */
  public Intake(CANSparkMax intakeMotor, CANSparkMax pivot) {
    this.intakeMotor = intakeMotor;
    this.pivot = pivot;

    pivot.restoreFactoryDefaults();
    //pivot.setInverted(true);

    forwardLimit = pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    reverseLimit = pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    forwardLimit.enableLimitSwitch(true);
    reverseLimit.enableLimitSwitch(true);

    intakeMotor.restoreFactoryDefaults();

    
    intakeforwardLimit = intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    intakereverseLimit = intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    intakeforwardLimit.enableLimitSwitch(true);
    intakereverseLimit.enableLimitSwitch(false);

    //Arm PID
    m_spedController = pivot.getPIDController();
    m_spedController.setP(armGains.kP);
    m_spedController.setI(armGains.kI);
    m_spedController.setD(armGains.kD);
  }

  //hanoff
  public void spinIntakeMotor() {
    intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  //Rotate robot in for handoff
  public void rotateArmIn() {
    //pivot.set(IntakeConstants.PIVOT_SPEED);
    pivot.set(IntakeConstants.PIVOT_SPEED);
  }

  //Rotates robot out for intake
  public void rotateArmOut() {
    pivot.set(-IntakeConstants.PIVOT_SPEED);

  }

  public void pidIn() {
    m_spedController.setReference(8000, CANSparkMax.ControlType.kVelocity);
    //SmartDashboard.putNumber("Velocity Output", pivot.getEncoder().getVelocity());
  }

  public void pidOut() {
    m_spedController.setReference(-8000, CANSparkMax.ControlType.kVelocity);

  }

  
  public void handoff() {
    intakeMotor.set(-IntakeConstants.INTAKE_OUT_SPEED);
  }

 

  public void stopIntakeMotors() {
    intakeMotor.set(0);
  }

  public void stopArmMotor() {
    pivot.set(0);
  }

  public RelativeEncoder getEncoder() {
    return pivot.getEncoder();
  }
  public void setVoltage(double voltage) {
   pivot.setVoltage(voltage);
  }

  public double getVelocity() {
    return pivot.getEncoder().getVelocity();
  }

  public void resetPostion() {
    pivot.getEncoder().setPosition(0);
  }

  public void resetPosition1() {
    if(forwardLimit.isPressed() == true) {
      resetPostion();
    }
  }

  public double getPos() {
    return pivot.getEncoder().getPosition();
  }

  public void rotateArm() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Vel Output", getVelocity());
    SmartDashboard.putNumber("Position", getPos());
    resetPosition1();
  }
}
