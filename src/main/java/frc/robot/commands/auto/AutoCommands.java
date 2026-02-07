package frc.robot.commands.auto;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommands extends Command {

    

    public static Command moveTest() {
        return Commands.sequence(
            new PathPlannerAuto("Test with vision")
        );
    }
        

   
}
