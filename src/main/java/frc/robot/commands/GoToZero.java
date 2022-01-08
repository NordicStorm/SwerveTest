package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.paths.MultiPartPath;

public class GoToZero extends CommandBase{
    @Override
    public void initialize() {
        MultiPartPath path = new MultiPartPath(Robot.drivetrain);
        path.changeDrivetrainConfigProperty("endOfTrajectoryPositionTolerance", 0.1);
        path.setHeading(0);
        //path.setHeadingFollowMovement(0);
        path.addWaypoint(0, 0);
        path.stop();
        path.finalizePath().schedule();
    }
}
