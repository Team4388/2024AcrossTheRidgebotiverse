package frc4388.robot.commands.Autos;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.Constants.AutoAlignConstants;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlign extends Command {
    private SwerveDrive swerve;
    private Limelight limelight;
    private Pose2d pose;

    private Optional<Alliance> alliance;

    private boolean isFinished;
    private boolean isReverseFinished;
    
    private boolean reverseAfterFinish;
    private Translation2d[] moveStickReplayArr;
    private Translation2d[] rotStickReplayArr;
    private int replayIndex;

    public AutoAlign(SwerveDrive swerve, Limelight limelight, boolean reverseAfterFinish) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.reverseAfterFinish = reverseAfterFinish;
    }

    private Translation2d calcMoveStick(){
        //TODO - DO THE MATH!

        final double curX = pose.getX();
        final double curY = pose.getY();
        final double curYaw = pose.getRotation().getDegrees();

        final boolean isred = alliance.get() == DriverStation.Alliance.Red;

        

        // This is not math
        double stickX = curX * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;
        double stickY = curY * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;

        return new Translation2d(stickX, stickY);
    }

    private Translation2d calcRotStick(){
        //TODO - DO THE MATH!

        final double curX = pose.getX();
        final double curY = pose.getY();
        final double curYaw = pose.getRotation().getDegrees();

        final boolean isred = alliance.get() == DriverStation.Alliance.Red;

        // This is not math
        double stickX = curX * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;
        double stickY = curY * AutoAlignConstants.MoveSpeed * AutoAlignConstants.RotSpeed;

        return new Translation2d(stickX, stickY);
    }

    // Called when the command is initially scheduled.
	@Override 
	public final void initialize() {
        if(reverseAfterFinish){
            isReverseFinished = false;
            replayIndex = 0;
            moveStickReplayArr = new Translation2d[]{};
            rotStickReplayArr = new Translation2d[]{};
        }
	}

    // Called every time the scheduler runs while the command is scheduled.
	@Override
    public void execute() {
        // Update limelight pos
        pose = limelight.getPose();

        // These must be 0, or it will error
        Translation2d moveStick = new Translation2d(0, 0);
        Translation2d rotStick = new Translation2d(0, 0);

        // Regular replay
        if(!isFinished){
            moveStick = calcMoveStick();
            rotStick = calcRotStick();

            // This is a way of appending...
            if(reverseAfterFinish){
                moveStickReplayArr[moveStickReplayArr.length] = moveStick;
                rotStickReplayArr[rotStickReplayArr.length] = rotStick;
            }

            // if(isFinished != limelight.isNearSpeaker() && isReverseFinished){
            //     replayIndex
            // }
            isFinished = limelight.isNearSpeaker();

        // If reverseAfterFinish, then loop back over and replay
        }else if(reverseAfterFinish && !isReverseFinished){
            // Get reverse direction
            moveStick = moveStickReplayArr[replayIndex-moveStickReplayArr.length-1];
            rotStick = rotStickReplayArr[replayIndex-rotStickReplayArr.length-1];

            // Invert sticks
            moveStick = new Translation2d(moveStick.getX()*-1, moveStick.getY()*-1);
            rotStick = new Translation2d(rotStick.getX()*-1, rotStick.getY()*-1);

            replayIndex++;
        }

        // This would greatly benifit from having feild Relative implemented.
        swerve.driveWithInput(moveStick, rotStick, false);
    }

    // Returns true when the command should end.
	@Override
	public final boolean isFinished() {
        return isFinished && (isReverseFinished || !reverseAfterFinish);
	}
}
