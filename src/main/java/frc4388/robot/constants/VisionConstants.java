package frc4388.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String LEFT_CAMERA_NAME = "CAMERA_LEFT";
    public static final String RIGHT_CAMERA_NAME = "CAMERA_RIGHT";

    public static final Transform3d LEFT_CAMERA_POS = new Transform3d(
        new Translation3d(Units.inchesToMeters(-13-9.134), -Units.inchesToMeters(10.75), Units.inchesToMeters(9.5)
        ), new Rotation3d(0,25.*(Math.PI/180.),Math.PI));
    public static final Transform3d RIGHT_CAMERA_POS = new Transform3d(
        new Translation3d(Units.inchesToMeters(-13-9.134), Units.inchesToMeters(10.75), Units.inchesToMeters(9.5)), 
        new Rotation3d(0,25.*(Math.PI/180.),Math.PI)
    );
    
    public static final double MIN_ESTIMATION_DISTANCE = 5; // Meters

    // Photonvision thing
    // The standard deviations of our vision estimated poses, which affect correction rate
    // X, Y, Theta
    // https://www.chiefdelphi.com/t/photonvision-finding-standard-deviations-for-swervedriveposeestimator/467802/2
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 1);
}