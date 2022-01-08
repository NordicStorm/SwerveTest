
package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Robot;
import frc.robot.commands.paths.MultiPartPath;

/**
 *
 */
public class AutonomousCommand extends AutoWithInit {


    public AutonomousCommand() {


    }

    @Override
    public void initializeCommands() {
// !PATHWEAVER_INFO: {"trackWidth":0.71,"gameName":"Slalom Path","outputDir":"C:\\Users\\Nordic Storm 3018\\FRC\\SwerveTest\\src\\main\\java\\frc\\robot\\commands\\AutonomousCommand.java"}
        MultiPartPath path;
        double startX = Units.feetToMeters(2.5);
        double startY = Units.feetToMeters(2.5);
        Robot.drivetrain.resetPose(startX, startY, 0);
        path = new MultiPartPath(Robot.drivetrain);
        path.setHeading(0);
        
        
        //path.addWaypoint(0.814, 0.715);
        path.addWaypoint(1.558, 0.688);
        path.addWaypoint(2.046, 0.932);
        path.addWaypoint(2.619, 2.309);
        path.addWaypoint(3.052, 2.313);
        path.addWaypoint(4.516, 2.335);
        path.addWaypoint(6.690, 2.190);
        path.addWaypoint(6.928, 1.106);
        path.addWaypoint(7.494, 0.563);
        path.addWaypoint(8.515, 1.038);
        path.addWaypoint(8.351, 2.263);
        path.addWaypoint(7.481, 2.250);
        path.addWaypoint(6.960, 1.051);
        path.addWaypoint(5.332, 0.566);
        path.addWaypoint(2.926, 0.802);
        path.addWaypoint(2.566, 1.604);
        path.addWaypoint(2.197, 2.243);
        path.addWaypoint(1.701, 2.305);
        path.addWaypoint(0.781, 2.381);
        
        path.stop();
        addCommands(path.finalizePath());
        
        
    }

    
}