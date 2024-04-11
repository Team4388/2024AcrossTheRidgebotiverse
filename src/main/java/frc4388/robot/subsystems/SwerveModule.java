// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;
import frc4388.utility.configurable.ConfigurableDouble;

public class SwerveModule extends SubsystemBase {
    // private WPI_TalonFX tal;
    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private CANcoder encoder;
    private int selfid;
    // private ConfigurableDouble offsetGetter;
    private static int swerveId = 0;
    public static Gains swerveGains = SwerveDriveConstants.PIDConstants.SWERVE_GAINS;
  
    /** Creates a new SwerveModule. */
    public SwerveModule(TalonFX driveMotor, TalonFX angleMotor, CANcoder encoder, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        // this.offsetGetter = new ConfigurableDouble("Swerve id " + swerveId, offset);
        this.selfid = swerveId;
         swerveId++;
        // TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        TalonFXConfiguration cfg = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(swerveGains.kP)
                .withKI(swerveGains.kI)
                .withKD(swerveGains.kD))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(encoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
        // angleConfig.slot0.kP = swerveGains.kP;
        // angleConfig.slot0.kI = swerveGains.kI;
        // angleConfig.slot0.kD = swerveGains.kD;

        // // use the CANcoder as the remote sensor for the primary TalonFX PID

        // new Slot0Configs().
        
        // angleConfig.remoteFilter0.remoteSensorDeviceID = encoder.getDeviceID();
        // angleConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        // angleConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.getConfigurator().apply(cfg);
        // angleMotor.configAllSettings(angleConfig);
        
        //encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        CANcoderConfiguration coder_cfg = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(offset));
        encoder.getConfigurator().apply(coder_cfg);
        reset(0);
        // encoder.configMagnetOffset(offset);
        // driveMotor.
        driveMotor.setPosition(0);
        driveMotor.getConfigurator().apply(new Slot0Configs().withKP(0.2));
        // driveMotor.setSelectedSensorPosition(0);
        // driveMotor.config_kP(0, 0.2);
    }
    
    @Override
    public void periodic() {
        //encoder.configMagnetOffset(offsetGetter.get());
        //SmartDashboard.putString("Error Code: " + selfid, getstuff());
        // SmartDashboard.putNumber("Angular Position: " + selfid, getAngle().getDegrees());
        // SmartDashboard.putNumber("Angular Velocity: " + selfid, getAngularVel());
        // SmartDashboard.putNumber("Drive Position: " + selfid, getDrivePos());
        // SmartDashboard.putNumber("Drive Velocity: " + selfid, getDriveVel());
    }
    /**
     * Get the drive motor of the SwerveModule
     * @return the drive motor of the SwerveModule
     */
    public TalonFX getDriveMotor() {
        return this.driveMotor;
    }

    /**
     * Get the angle motor of the SwerveModule
     * @return the angle motor of the SwerveModule
     */
    public TalonFX getAngleMotor() {
        return this.angleMotor;
    }

    /**
     * Get the CANcoder of the SwerveModule
     * @return the CANcoder of the SwerveModule
     */
    public CANcoder getEncoder() {
        return this.encoder;
    }

    /**
     * Get the angle of a SwerveModule as a Rotation2d
     * @return the angle of a SwerveModule as a Rotation2d
     */
    public Rotation2d getAngle() {
        // * Note: This assumes that the CANCoders are setup with the default feedback coefficient and the sensor value reports degrees.
        return Rotation2d.fromDegrees(encoder.getPosition().getValue() * 360);
        // return Rotation2d.fromDegrees(tal.get());
    }
    
    public double getAngularVel() {
        return this.angleMotor.getVelocity().getValue() * 360;
        // return this.tal.getSelectedSensorVelocity();
        // return this.angleMotor.getSelectedSensorVelocity();
    }

    public double getDrivePos() {
        return this.driveMotor.getPosition().getValue(); // TODO: with drive test, might have to multiply or divide by 2
        // return this.tal.getSelectedSensorPosition() / SwerveDriveConstants.Conversions.TICKS_PER_MOTOR_REV;
    }

    public double getDriveVel() {
        return this.driveMotor.getVelocity().getValue() * 360;
        // return this.driveMotor.getSelectedSensorVelocity(0);
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public void rotateToAngle(double angle) {
        angleMotor.setControl(new PositionVoltage(angle));
        // angleMotor.set(TalonFXControlMode.Position, angle);
    }

    /**
     * Get state of swerve module
     * @return speed in m/s and angle in degrees
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Units.inchesToMeters(driveMotor.getVelocity().getValue() * SwerveDriveConstants.Conversions.INCHES_PER_TICK) * SwerveDriveConstants.Conversions.TICK_TIME_TO_SECONDS,
            // Units.inchesToMeters(driveMotor.getSelectedSensorVelocity() * SwerveDriveConstants.Conversions.INCHES_PER_TICK) * SwerveDriveConstants.Conversions.TICK_TIME_TO_SECONDS, 
            getAngle()
        );
    }

    /**
     * Returns the current position of the SwerveModule
     * @return The current position of the SwerveModule in meters traveled by the driveMotor and the angle of the angleMotor.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(Units.inchesToMeters(driveMotor.getPosition().getValue() * SwerveDriveConstants.Conversions.INCHES_PER_TICK), getAngle());
    }

    /**
     * Set the speed and rotation of the SwerveModule from a SwerveModuleState object
     * @param desiredState a SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = this.getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // calculate the difference between our current rotational position and our new rotational position
        // Rotation2d rotationDelta = state.angle.minus(currentRotation);

        // rotationDelta.getDegrees();

        // calculate the new absolute position of the SwerveModule based on the difference in rotation
        // double deltaTicks = (rotationDelta.getDegrees() / 360.) * SwerveDriveConstants.Conversions.CANCODER_TICKS_PER_ROTATION;

        // convert the CANCoder from its position reading to ticks
        // (new CANCoder(13)).getPosition()

        // double currentTicks = encoder.getPosition().getValue() / encoder.configGetFeedbackCoefficient();
        
        // double currentTicks = encoder.getPosition().getValue() / 0.087890625;

        // angleMotor.setControl(new PositionVoltage(currentTicks + deltaTicks));
        System.out.println(desiredState.angle.getDegrees());
        angleMotor.setControl(new PositionVoltage(desiredState.angle.getDegrees()/360));
        // angleMotor.setControl(new PositionVoltage(0));
        // angleMotor.set(TalonFXControlMode.Position, currentTicks + deltaTicks);

        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);

        // driveMotor.set((feetPerSecond / SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    }

    public void reset(double position) {
        encoder.setPosition(encoder.getAbsolutePosition().getValue());
        // (new CANCoder(13)).setPositionToAbsolute();
    }

    public double getCurrent() {
        // angleMotor.getSupplyVoltage()
        // return angleMotor.getMotorVoltage().getValue() + driveMotor.getMotorVoltage().getValue();
        return angleMotor.getSupplyCurrent().getValue() + driveMotor.getSupplyCurrent().getValue();
    }

    public double getVoltage() {
        return Math.abs(angleMotor.getMotorVoltage().getValue()) + Math.abs(driveMotor.getMotorVoltage().getValue());
        // return (Math.abs((new WPI_TalonFX(1).getMotorOutputVoltage()) + Math.abs(driveMotor.getMotorOutputVoltage()));
    }

    // public String getstuff() {
    //     encoder.getPosition();
    //     return "" + encoder.getLastError().value;
    // }
}
