package frc4388.robot.constants;

import frc4388.utility.status.CanDevice;
import frc4388.utility.structs.Gains;

public class IntakeConstants {
    // public static final int INTAKE_MOTOR_ID = 18;
    // public static final int PIVOT_MOTOR_ID = 17;
    public static final CanDevice INTAKE_MOTOR_ID = new CanDevice("Intake Motor", 18);
    public static final CanDevice PIVOT_MOTOR_ID = new CanDevice("Pivot Motor", 17);

    
    public static final double INTAKE_SPEED = 0.75;
    public static final double INTAKE_OUT_SPEED_UNPRESSED =  1.0;       
    public static final double INTAKE_OUT_SPEED_PRESSED = 0.5;        
    public static final double HANDOFF_SPEED = 0.4;
    public static final double PIVOT_SPEED = 0.2;

    public static final class ArmPID {
        public static final Gains INTAKE_GAINS = new Gains(0.05, 0, 0, 0, 0, 1.0);
    }
}
