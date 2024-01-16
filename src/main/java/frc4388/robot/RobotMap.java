/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveModule;
import frc4388.utility.RobotGyro;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
    private Pigeon2 m_pigeon2 = new Pigeon2(14);
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
    public final TalonFX leftFrontWheel = new TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_WHEEL_ID);
    public final TalonFX leftFrontSteer = new TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_STEER_ID);
    public final CANcoder leftFrontEncoder = new CANcoder(SwerveDriveConstants.IDs.LEFT_FRONT_ENCODER_ID);


    public final TalonFX rightFrontWheel = new TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_WHEEL_ID);
    public final TalonFX rightFrontSteer = new TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_STEER_ID);
    public final CANcoder rightFrontEncoder = new CANcoder(SwerveDriveConstants.IDs.RIGHT_FRONT_ENCODER_ID);
        
    public final TalonFX leftBackWheel = new TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_WHEEL_ID);
    public final TalonFX leftBackSteer = new TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_STEER_ID);
    public final CANcoder leftBackEncoder = new CANcoder(SwerveDriveConstants.IDs.LEFT_BACK_ENCODER_ID);

    public final TalonFX rightBackWheel = new TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_WHEEL_ID);
    public final TalonFX rightBackSteer = new TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_STEER_ID);
    public final CANcoder rightBackEncoder = new CANcoder(SwerveDriveConstants.IDs.RIGHT_BACK_ENCODER_ID);

    void configureLEDMotorControllers() {    
    }

    void configureDriveMotorControllers() {
        // config factory default
        TalonFXConfiguration defualtMotorConfig = new TalonFXConfiguration();
        
        leftFrontWheel.getConfigurator().apply(defualtMotorConfig);
        leftFrontSteer.getConfigurator().apply(defualtMotorConfig);
        
        rightFrontWheel.getConfigurator().apply(defualtMotorConfig);
        rightFrontSteer.getConfigurator().apply(defualtMotorConfig);
        
        leftBackWheel.getConfigurator().apply(defualtMotorConfig);
        leftBackSteer.getConfigurator().apply(defualtMotorConfig);
        
        rightBackWheel.getConfigurator().apply(defualtMotorConfig);
        rightBackSteer.getConfigurator().apply(defualtMotorConfig);

        // set neutral mode
        leftFrontWheel.setNeutralMode(NeutralModeValue.Brake);
        rightFrontWheel.setNeutralMode(NeutralModeValue.Brake);
        leftBackWheel.setNeutralMode(NeutralModeValue.Brake);
        rightBackWheel.setNeutralMode(NeutralModeValue.Brake);

        leftFrontSteer.setNeutralMode(NeutralModeValue.Brake);
        rightFrontSteer.setNeutralMode(NeutralModeValue.Brake);
        leftBackSteer.setNeutralMode(NeutralModeValue.Brake);
        rightBackSteer.setNeutralMode(NeutralModeValue.Brake);

        // initialize SwerveModules
        this.leftFront = new SwerveModule(leftFrontWheel, leftFrontSteer, leftFrontEncoder, -181.230469);
        this.rightFront = new SwerveModule(rightFrontWheel, rightFrontSteer, rightFrontEncoder, -270.615234);
        this.leftBack = new SwerveModule(leftBackWheel, leftBackSteer, leftBackEncoder, -240.029297);
        this.rightBack = new SwerveModule(rightBackWheel, rightBackSteer, rightBackEncoder, -40.869142);
    }
}
