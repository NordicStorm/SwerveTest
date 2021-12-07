package frc.robot.commands.paths;

/**
 * Velocity units are meters per second. Acceleration is meters per second per
 * second. Angular velocity and acceleration are radians per second.
 */
public class DrivetrainConfig {

    public double maxVelocity = 0;
    public double maxAcceleration = 0;
    public double maxAngularAcceleration = 0;
    public double maxAnglularVelocity = 0;

    public double positionCorrectionP = 0;
    public double positionCorrectionI = 0;
    public double positionCorrectionD = 0;

    public double rotationCorrectionP = 0;
    public double rotationCorrectionI = 0;
    public double rotationCorrectionD = 0;

    public double endOfTrajectoryPositionTolerance = 999;
    public double endOfTrajectoryAngleTolerance = 999;

    //meters per second
    public double stopVelocityTolerance = 0.01;

    //radians per second
    public double stopAngularVelocityTolerance = 0.01;

	public DrivetrainConfig makeClone() {
        var c = new DrivetrainConfig();
        c.maxVelocity = maxVelocity;
        c.maxAcceleration = maxAcceleration;
        c.maxAngularAcceleration = maxAngularAcceleration;
        c.maxAnglularVelocity = maxAnglularVelocity;
        c.positionCorrectionP = positionCorrectionP;
        c.positionCorrectionI = positionCorrectionI;
        c.positionCorrectionD = positionCorrectionD;

        c.rotationCorrectionP = rotationCorrectionP;
        c.rotationCorrectionI = rotationCorrectionI;
        c.rotationCorrectionD = rotationCorrectionD;

        c.endOfTrajectoryPositionTolerance = endOfTrajectoryPositionTolerance;
        c.endOfTrajectoryAngleTolerance = endOfTrajectoryAngleTolerance;
		return c;
	}
}
