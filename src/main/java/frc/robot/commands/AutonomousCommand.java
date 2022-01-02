
package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.paths.MultiPartPath;

/**
 *
 */
public class AutonomousCommand extends AutoWithInit {


    public AutonomousCommand() {


    }

    @Override
    public void initializeCommands() {
// !PATHWEAVER_INFO: {"trackWidth":0.71,"gameName":"Destination: Deep Space","outputDir":"E:\\Documents\\FRCStuff\\SwerveTest\\src\\main\\java\\frc\\robot\\commands\\AutonomousCommand.java"}
        MultiPartPath path;
        path.addWaypoint(9.572, 0.950);
        path.addWaypoint(6.495, 1.835);
        path.addWaypoint(4.807, 1.917);
        path.addWaypoint(1.798, 0.882);
        path.addWaypoint(3.118, 3.373);
        path.addWaypoint(5.964, 5.931);
    }

    
}