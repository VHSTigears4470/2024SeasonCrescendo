package frc.robot.commands.AutoGroups;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos{

    private Autos(){
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static CommandBase LoadAuto(SwerveAutoBuilder autoBuilder, String name, double maxVelocity, double maxAcceleration){
        // Check if files exists
        if(!new File(Filesystem.getDeployDirectory(), "pathplanner/" + name + ".path").isFile()){
            return null;
        }
        else{
            // Creates trajectory for path planner
            List<PathPlannerTrajectory> trajectoryList = PathPlanner.loadPathGroup(name, new PathConstraints(maxVelocity, maxAcceleration));
            return Commands.sequence(autoBuilder.fullAuto(trajectoryList));
        }
    }
}