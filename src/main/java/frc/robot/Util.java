package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class Util {
    public static SwerveModuleState stateFromModule(SwerveModule module){
        return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
    }
}
