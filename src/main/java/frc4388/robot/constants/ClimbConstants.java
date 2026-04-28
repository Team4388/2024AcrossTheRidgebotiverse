package frc4388.robot.constants;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc4388.utility.status.CanDevice;

public class ClimbConstants {
    public static final CanDevice CLIMB_MOTOR_ID = new CanDevice("Climb Motor", 19);
    public static final double CLIMB_IN_SPEED = -1.0;
    public static final double CLIMB_OUT_SPEED = 0.87;

    public static final TalonFXConfiguration CLIMB_MOTOR_CONFIG = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive)
        );
}
