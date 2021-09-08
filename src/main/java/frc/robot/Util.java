package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class Util {
    public static SwerveModuleState stateFromModule(SwerveModule module){
        return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
    }
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    public static double applyDeadzone(double value, double deadzone){
        if(Math.abs(value)<deadzone){
            return 0;
        }else{
            return Math.copySign(map(Math.abs(value), deadzone, 1, 0, 1), value);
        }
    }
}
