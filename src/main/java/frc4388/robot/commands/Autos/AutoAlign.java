package frc4388.robot.commands.Autos;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.Constants.AutoAlignConstants;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoAlign extends Command {
    private SwerveDrive swerve;
    private Limelight limelight;
    private Pose2d pose;

    private boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    public AutoAlign(SwerveDrive swerve, Limelight limelight) {
        this.swerve = swerve;
    }

    private Translation2d calcMoveStick(){
        //TODO - DO THE MATH!
        pose = limelight.getPose();

        double curX = pose.getX();
        double curY = pose.getY();
        double curYaw = pose.getRotation().getDegrees();

        // This is not math
        double stickX = curX * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;
        double stickY = curY * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;

        return new Translation2d(stickX, stickY);
    }

    private Translation2d calcRotStick(){
        //TODO - DO THE MATH!
        pose = limelight.getPose();

        double curX = pose.getX();
        double curY = pose.getY();
        double curYaw = pose.getRotation().getDegrees();

        // This is not math
        double stickX = curX * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;
        double stickY = curY * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;

        return new Translation2d(stickX, stickY);
    }

    public void execute() {
        Translation2d moveStick = calcMoveStick();
        Translation2d rotStick = calcRotStick();

        // This would greatly benifit from having feild Relative implemented.
        swerve.driveWithInput(moveStick, rotStick, false);
    }
}
