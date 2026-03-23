package frc.robot.util;

import java.util.Arrays;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * @author Anvay Mathur
 */
public class AutoPath extends SequentialCommandGroup {
	private final String autoName;
	private final Supplier<Pose2d> startingPose;

    /**
     * Follows the path given by the path name, returns null if the path doesn't exist
     * @param pathName The name of the file of the path to run
     * @return The command to follow the path or null if path not found
     */
	public static Command followPath(String pathName){
		try{
			return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
		} catch (Exception e){
			System.out.println("Path Error " + e);
			return null;
		}
	}

    /**
     * Runs the commands given sequentially, with the given auto name and starting position
     * @param autoName The name of the auto
     * @param startingPose The starting pose of the auto
     * @param commands The commands to run sequentially
     */
    public AutoPath(String autoName, Pose2d startingPose, Command... commands){
		super(
			Stream.concat(
				Stream.of(Commands.runOnce(() ->{
				AutoBuilder.resetOdom(getFlippedStartingPose(startingPose));
			})),
				Arrays.stream(commands)
			).toArray(Command[]::new)
		);
		this.autoName = autoName;
		this.startingPose = () -> getFlippedStartingPose(startingPose);
        Logger.recordOutput("AutoName", this.getName());
    }

    /**
     * @return The name of the auto
     */
	public String getName(){
		return autoName;
	}

    /**
     * @return the starting pose of the auto
     */
	public Pose2d getStartingPose(){
		return startingPose.get();
	}

	private static Pose2d getFlippedStartingPose(Pose2d startingPose){
		Pose2d pose = startingPose;
		if (AutoBuilder.shouldFlip())
			pose = FlippingUtil.flipFieldPose(pose);
		return pose;
	}

}
