/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.Constants.ShooterConstants;
import frc4388.robot.Constants.IntakeConstants;
import frc4388.robot.subsystems.SwerveModule;
import frc4388.utility.RobotGyro;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
    private WPI_Pigeon2 m_pigeon2 = new WPI_Pigeon2(14);
    public RobotGyro gyro = new RobotGyro(m_pigeon2);

    public SwerveModule leftFront;
    public SwerveModule rightFront;
    public SwerveModule leftBack;
    public SwerveModule rightBack;

    public RobotMap() {
        configureLEDMotorControllers();
        configureDriveMotorControllers();
    }

    /* LED Subsystem */
    public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);


    /* Swreve Drive Subsystem */
    public final WPI_TalonFX leftFrontWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_WHEEL_ID);
    public final WPI_TalonFX leftFrontSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_STEER_ID);
    public final CANCoder leftFrontEncoder = new CANCoder(SwerveDriveConstants.IDs.LEFT_FRONT_ENCODER_ID);


    public final WPI_TalonFX rightFrontWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_WHEEL_ID);
    public final WPI_TalonFX rightFrontSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_STEER_ID);
    public final CANCoder rightFrontEncoder = new CANCoder(SwerveDriveConstants.IDs.RIGHT_FRONT_ENCODER_ID);
        
    public final WPI_TalonFX leftBackWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_WHEEL_ID);
    public final WPI_TalonFX leftBackSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_STEER_ID);
    public final CANCoder leftBackEncoder = new CANCoder(SwerveDriveConstants.IDs.LEFT_BACK_ENCODER_ID);

    public final WPI_TalonFX rightBackWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_WHEEL_ID);
    public final WPI_TalonFX rightBackSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_STEER_ID);
    public final CANCoder rightBackEncoder = new CANCoder(SwerveDriveConstants.IDs.RIGHT_BACK_ENCODER_ID);

    /* Shooter Subsystem */
    public final TalonFX leftShooter = new TalonFX(ShooterConstants.LEFT_SHOOTER_ID);
    public final TalonFX rightShooter = new TalonFX(ShooterConstants.RIGHT_SHOOTER_ID);

    /* Intake Subsystem */
    public final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    void configureLEDMotorControllers() {    
    }

    void configureDriveMotorControllers() {
        // config factory default
        leftFrontWheel.configFactoryDefault();
        leftFrontSteer.configFactoryDefault();
        
        rightFrontWheel.configFactoryDefault();
        rightFrontSteer.configFactoryDefault();
        
        leftBackWheel.configFactoryDefault();
        leftBackSteer.configFactoryDefault();
        
        rightBackWheel.configFactoryDefault();
        rightBackSteer.configFactoryDefault();

        // config open loop ramp
        leftFrontWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        leftFrontSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        
        rightFrontWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        rightFrontSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        
        leftBackWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        leftBackSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        
        rightBackWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        rightBackSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);

        // config closed loop ramp
        leftFrontWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        leftFrontSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        
        rightFrontWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        rightFrontSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        
        leftBackWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        leftBackSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        
        rightBackWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
        rightBackSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);

        // config neutral deadband
        leftFrontSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
        leftFrontWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);

        rightFrontWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
        rightFrontSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);

        leftBackWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
        leftBackSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);

        rightBackWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
        rightBackSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);

        // set neutral mode
        leftFrontWheel.setNeutralMode(NeutralMode.Brake);
        rightFrontWheel.setNeutralMode(NeutralMode.Brake);
        leftBackWheel.setNeutralMode(NeutralMode.Brake);
        rightBackWheel.setNeutralMode(NeutralMode.Brake);

        leftFrontSteer.setNeutralMode(NeutralMode.Brake);
        rightFrontSteer.setNeutralMode(NeutralMode.Brake);
        leftBackSteer.setNeutralMode(NeutralMode.Brake);
        rightBackSteer.setNeutralMode(NeutralMode.Brake);

        // initialize SwerveModules
        this.leftFront = new SwerveModule(leftFrontWheel, leftFrontSteer, leftFrontEncoder, SwerveDriveConstants.DefaultSwerveRotOffsets.FRONT_LEFT_ROT_OFFSET);
        this.rightFront = new SwerveModule(rightFrontWheel, rightFrontSteer, rightFrontEncoder, SwerveDriveConstants.DefaultSwerveRotOffsets.FRONT_RIGHT_ROT_OFFSET);
        this.leftBack = new SwerveModule(leftBackWheel, leftBackSteer, leftBackEncoder, SwerveDriveConstants.DefaultSwerveRotOffsets.BACK_LEFT_ROT_OFFSET);
        this.rightBack = new SwerveModule(rightBackWheel, rightBackSteer, rightBackEncoder, SwerveDriveConstants.DefaultSwerveRotOffsets.BACK_RIGHT_ROT_OFFSET);
    }
}
