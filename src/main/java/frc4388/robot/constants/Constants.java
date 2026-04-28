/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc4388.utility.compute.Trim;
import frc4388.utility.structs.Gains;
import frc4388.utility.structs.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final CANBus RIO_CANBUS = CANBus.roboRIO();
    // public static final CANBus CANIVORE_CANBUS = new CANBus("drivetrain");

    // public static final class LiDARConstants {
    //     public static final int REEF_LIDAR_DIO_CHANNEL = 7;
    //     public static final int REVERSE_LIDAR_DIO_CHANNEL = 4;

    //     public static final int HUMAN_PLAYER_STATION_DISTANCE = 40;

    //     public static final int LIDAR_DETECT_DISTANCE = 100; // Min distance to detect pole
    //     public static final int LIDAR_MICROS_TO_CM = 10;
    //     public static final int SECONDS_TO_MICROS = 1000000;
    // }

    public static final class AutoConstants {
        // public static final Gains XY_GAINS = new Gains(5,0.6,0.0);
        public static final Gains ROT_GAINS = new Gains(8,0,0.0);
        // public static final ConfigurableDouble P_XY_GAINS = new ConfigurableDouble("P_XY_GAINS", XY_GAINS.kP);
        // public static final ConfigurableDouble I_XY_GAINS = new ConfigurableDouble("I_XY_GAINS", XY_GAINS.kI);
        // public static final ConfigurableDouble D_XY_GAINS = new ConfigurableDouble("D_XY_GAINS", XY_GAINS.kD);
       // public static final Gains XY_GAINS = new Gains(3,0.3,0.0);

        // public static final Gains ROT_GAINS = new Gains(0.05,0,0.007);
        // public static final Gains ROT_GAINS = new Gains(0.05,0,0.0);

        public static final Trim X_OFFSET_TRIM =        new Trim("X Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE,0.5, 0);
        // public static final Trim Y_OFFSET_TRIM =        new Trim("Y Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE, 0.5, 1.5);
        public static final Trim Y_OFFSET_TRIM =        new Trim("Y Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE, 0.5, 0);
         
        public static final double XY_TOLERANCE = 0.07; // Meters
        public static final double ROT_TOLERANCE = 10; // Degrees

        public static final double MIN_XY_PID_OUTPUT = 0.0;
        public static final double MIN_ROT_PID_OUTPUT = 0.0;

        public static final double VELOCITY_THRESHHOLD = 0.01;

        
        public static final double STOP_VELOCITY = 0.;
    }

   

    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 8;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_RAINBOW;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
        public static final int BUTTONBOX_ID = 2;
        public static final int XBOX_PROGRAMMER_ID = 3;
        public static final double LEFT_AXIS_DEADBAND = 0.1;

    }



    // Logging constants
    public static class SimConstants {
        public static final Mode simMode = Mode.SIM;
        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
    }
}