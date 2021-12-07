package frc.robot.commands.paths;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FullStopPiece extends CommandBase implements CommandPathPiece {
    MultiPartPath path;
    long endTime = 0;
    long waitTime;

    public FullStopPiece(MultiPartPath path, long waitTime) {
        this.path = path;
        this.waitTime = waitTime;
    }

    @Override
    public void initialize() {
        endTime = System.currentTimeMillis() + waitTime;
    }

    @Override
    public void execute() {
        path.drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        var speeds = path.drivetrain.getSpeeds();
        return System.currentTimeMillis() >= endTime
                && speeds.vxMetersPerSecond <= path.drivetrainConfig.stopVelocityTolerance
                && speeds.vyMetersPerSecond <= path.drivetrainConfig.stopVelocityTolerance
                && speeds.omegaRadiansPerSecond <= path.drivetrainConfig.stopAngularVelocityTolerance;
    }

    @Override
    public boolean interruptsTrajectory() {
        return true;
    }

    @Override
    public double getRequestedStartSpeed() {
        return 0;
    }

}
