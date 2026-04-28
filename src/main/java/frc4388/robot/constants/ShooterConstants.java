package frc4388.robot.constants;

import frc4388.utility.status.CanDevice;

public class ShooterConstants {
    // public static final int LEFT_SHOOTER_ID = 15; // final
    // public static final int RIGHT_SHOOTER_ID = 16; // final

    public static final CanDevice LEFT_SHOOTER_ID = new CanDevice("Left Shooter Motor", 45);    
    public static final CanDevice RIGHT_SHOOTER_ID = new CanDevice("Right Shooter Motor", 46);


    
    public static final double SHOOTER_SPEED = 0.50; // final
    public static final double SHOOTER_IDLE = 0.20; // final
    public static final double SHOOTER_IDLE_LIMELIGHT = 0.20;
}
