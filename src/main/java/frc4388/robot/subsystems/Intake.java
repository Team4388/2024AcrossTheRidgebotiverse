// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.commands.PID;
import frc4388.utility.Gains;
import frc4388.utility.configurable.ConfigurableDouble;
import frc4388.utility.controller.DeadbandedXboxController;

public class Intake extends SubsystemBase {
  
  //NEO
  private CANSparkMax intakeMotor;
  private CANSparkMax pivot;
  private SparkPIDController m_spedController;
  private SparkLimitSwitch forwardLimit;
  private SparkLimitSwitch reverseLimit;
  private SparkLimitSwitch intakeforwardLimit;  
  private SparkLimitSwitch intakereverseLimit;

  //Talon
  private TalonFX talonIntake;
  private TalonFX talonPivot;
  private CANcoder encoder;

  private DeadbandedXboxController m_driverXbox;
  private DeadbandedXboxController m_operatorXbox;

  private boolean r;

  private HardwareLimitSwitchConfigs l;

  TalonFXConfiguration doodooController = new TalonFXConfiguration();

  public static Gains armGains = IntakeConstants.ArmPID.INTAKE_GAINS;
  private ConfigurableDouble outtakeSpeed = new ConfigurableDouble("Outtake Speed", IntakeConstants.INTAKE_OUT_SPEED_UNPRESSED);
  private BooleanSupplier sup = () -> true;
  private BooleanSupplier dup = () -> false;

  private double smartDashboardOuttakeValue;

  /** Creates a new Intake. */
  //For NEO
  // public Intake(CANSparkMax intakeMotor, CANSparkMax pivot) {
  //   this.intakeMotor = intakeMotor;
  //   this.pivot = pivot;

  //   pivot.restoreFactoryDefaults();
  //   //pivot.setInverted(true);

  //   forwardLimit = pivot.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  //   reverseLimit = pivot.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  //   forwardLimit.enableLimitSwitch(true);
  //   reverseLimit.enableLimitSwitch(true);

  //   intakeMotor.restoreFactoryDefaults();
    
  //   intakeforwardLimit = intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  //   intakereverseLimit = intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  //   intakeforwardLimit.enableLimitSwitch(true);
  //   intakereverseLimit.enableLimitSwitch(false);

  //   //Arm PID
  //   m_spedController = pivot.getPIDController();
  //   m_spedController.setP(armGains.kP);
  //   m_spedController.setI(armGains.kI);
  //   m_spedController.setD(armGains.kD);

  //   SmartDashboard.putNumber("Outtake Speed", IntakeConstants.INTAKE_OUT_SPEED_UNPRESSED);
  // }

  //For Talon
  public Intake(TalonFX talonIntake, TalonFX talonPivot, DeadbandedXboxController driverXbox, DeadbandedXboxController operatorXbox) {
    this.talonIntake = talonIntake;
    this.talonPivot = talonPivot;
    
    this.m_driverXbox = driverXbox;
    this.m_operatorXbox = operatorXbox;

    talonIntake.getConfigurator().apply(new TalonFXConfiguration());
    talonPivot.getConfigurator().apply(new TalonFXConfiguration());

    talonIntake.setNeutralMode(NeutralModeValue.Brake);
    talonPivot.setNeutralMode(NeutralModeValue.Brake);

    // talonPivot.getConfigurator().apply(new HardwareLimitSwitchConfigs());
    // talonIntake.getConfigurator().apply(new HardwareLimitSwitchConfigs());

  
    
    // doodooController.Slot0.kP = armGains.kP;
    // doodooController.Slot1.kI = armGains.kI;
    // doodooController.Slot2.kD = armGains.kD;

    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.7; // An error of 0.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.06; // A velocity of 1 rps results in 0.1 V output

    talonPivot.getConfigurator().apply(slot0Configs);

    
  }

  // ! Talon Methods
  public void talonPIDIn() {
    PositionVoltage request = new PositionVoltage(-59);
    talonPivot.setControl(request.withPosition(0)); 
  }

  public void talonPIDOut() {
    PositionVoltage request = new PositionVoltage(0);
    talonPivot.setControl(request.withPosition(-59)); 
  }

  public void talonPIDPosition(double out2) {
    PositionVoltage request = new PositionVoltage(out2);
    talonPivot.setControl(request);
  }

  public void talonHandoff() {
    talonIntake.set(-outtakeSpeed.get());
  }

  public void talonSpinIntakeMotor() {
    talonIntake.set(IntakeConstants.INTAKE_SPEED);
  }

  public void talonSpinIntakeMotor(double speed) {
    talonIntake.set(speed);
  }

  public boolean getTalonIntakeLimitSwitchState() {
    if(r = talonIntake.getForwardLimit().getValue().value == 0) {
      return true;
    }
    return false;
  }

  public void talonSetPivotEncoderPosition(int val) {
    talonPivot.setPosition(val);
  }
  
  public void talonStopIntakeMotors() {
    talonIntake.set(0);
  }

  public void talonStopArmMotor() {
    talonPivot.set(0);
  }

  public double getArmPos() {
    return talonPivot.getPosition().getValue();
  }

  public void resetArmPosition() {
    if(getTalonIntakeLimitSwitchState()){
     // talonPivot.setPosition(0);
    }
  }

  public void ampPosition() {
    PositionVoltage request = new PositionVoltage(-0);
    talonPivot.setControl(request.withPosition(-59)); //TODO: Find actual value
  } 

  public void ampShoot(double speed) {
    talonSpinIntakeMotor(speed);
  }


  // ! NEO Methods
  //hanoff
  // public void spinIntakeMotor() {
  //   intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  // }

  // //Rotate robot in for handoff
  // public void rotateArmIn() {
  //   pivot.set(IntakeConstants.PIVOT_SPEED);
  // }

  // //Rotates robot out for intake
  // public void rotateArmOut() {
  //   pivot.set(-IntakeConstants.PIVOT_SPEED);

  // }

  // public void pidIn() {
  //   m_spedController.setReference(2.5, CANSparkMax.ControlType.kPosition);
  //   //SmartDashboard.putNumber("Velocity Output", pivot.getEncoder().getVelocity());
  // }
  
  // public void pidOut() {
  //   m_spedController.setReference(-53, CANSparkMax.ControlType.kPosition);
  // }

  // public void limitNote() {
  //   if (intakeforwardLimit.isPressed()) {
  //     rotateArmIn2();
  //   } else {
  //     spinIntakeMotor();
  //   }
  // }

  // public void rotateArmOut2() {
  //   if(reverseLimit.isPressed()){
  //     stopArmMotor();
  //   } else {
  //     pidOut();
  //   }
  // }

  // public void rotateArmIn2() {
  //   if(forwardLimit.isPressed()){
  //     stopArmMotor();
  //   } else {
  //     pidIn();
  //   }
  // }
  
  // public void handoff() {
  //   intakeMotor.set(-IntakeConstants.INTAKE_OUT_SPEED_UNPRESSED);
  // }

  // public void handoff2() {
  //   if(intakeforwardLimit.isPressed()) {
  //     intakeMotor.set(-smartDashboardOuttakeValue);
  //   } else {
  //     intakeMotor.set(-smartDashboardOuttakeValue);
  //   }
  // }

  // public void stopIntakeMotors() {
  //   intakeMotor.set(0);
  // }

  // public void stopArmMotor() {
  //   pivot.set(0);
  // }

  // public RelativeEncoder getEncoder() {
  //   return pivot.getEncoder();
  // }

  // public boolean getForwardLimitSwitchState() {
  //   return forwardLimit.isPressed();
  // }

  // public boolean getReverseLimitSwitchState() {
  //   return reverseLimit.isPressed();
  // }

  // public boolean getIntakeLimitSwtichState() {
  //   return intakeforwardLimit.isPressed();
  // }

  // public void setVoltage(double voltage) {
  //  pivot.setVoltage(voltage);
  // }

  // public double getVelocity() {
  //   return pivot.getEncoder().getVelocity();
  // }

  // public void setPivotEncoderPosition(int val) {
  //   pivot.getEncoder().setPosition(val);
  // }

  // public void resetPosition() {
  //   if(forwardLimit.isPressed()) {
  //     setPivotEncoderPosition(0);
  //   }
  // }

  // public double getPos() {
  //   return pivot.getEncoder().getPosition();
  // }

  // public double getIntakeVelocity() {
  //   return intakeMotor.getEncoder().getVelocity();
  // }

  // public void rotateArm() {

  // }

  // public BooleanSupplier getArmFowardLimitState() {
  //   if(forwardLimit.isPressed()) {
  //     return sup;
  //   } else {
  //     return dup;
  //   }
  // }

  // public void changeIntakeNeutralState() {
  //   if(forwardLimit.isPressed()) {
  //     intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
  //   }
  // }

  private boolean isRumble = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Vel Output", getVelocity());
    // SmartDashboard.putNumber("Position", getPos());
    // resetPosition();
    // changeIntakeNeutralState();

    resetArmPosition();

    SmartDashboard.putNumber("Pivot Position", getArmPos());

    smartDashboardOuttakeValue = SmartDashboard.getNumber("Outtake Speed", IntakeConstants.INTAKE_OUT_SPEED_UNPRESSED);

    // System.out.println(getTalonIntakeLimitSwitchState());

    if(getTalonIntakeLimitSwitchState() && !isRumble){
      m_driverXbox.rumble(1);
      m_operatorXbox.rumble(1);
      isRumble = true;
    }else if(!getTalonIntakeLimitSwitchState() && isRumble){
      isRumble = false;
    }
  }
}
