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
        this.leftFront = new SwerveModule(leftFrontWheel, leftFrontSteer, leftFrontEncoder, -181.230469);
        this.rightFront = new SwerveModule(rightFrontWheel, rightFrontSteer, rightFrontEncoder, -270.615234);
        this.leftBack = new SwerveModule(leftBackWheel, leftBackSteer, leftBackEncoder, -240.029297);
        this.rightBack = new SwerveModule(rightBackWheel, rightBackSteer, rightBackEncoder, -40.869142);
    }
}
