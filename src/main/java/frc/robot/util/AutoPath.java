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

public class AutoPath extends SequentialCommandGroup {
	private final String autoName;
	private final Supplier<Pose2d> startingPose;

	public static Command followPath(String pathName){
		try{
			return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
		} catch (Exception e){
			System.out.println("Path Error " + e);
			return null;
		}
	}

    public AutoPath(String autoName, Pose2d startingPose, Command... commands){
		super(
			Stream.concat(
				Stream.of(Commands.runOnce(() ->{
				AutoBuilder.resetOdom(getFlippedStartingPose(startingPose));
			})),
				Arrays.stream(commands)
			).toArray(Command[]::new)
		);
		Logger.recordOutput("AutoName", this.getName());
		this.autoName = autoName;
		this.startingPose = () -> getFlippedStartingPose(startingPose);
    }

	public String getName(){
		return autoName;
	}

	public Pose2d getStartingPose(){
		return startingPose.get();
	}

	private static Pose2d getFlippedStartingPose(Pose2d startingPose){
		Pose2d pose = startingPose;
		
		if (AutoBuilder.shouldFlip())
			pose = FlippingUtil.flipFieldPose(pose);
			// pose = new Pose2d(FlippingUtil.flipFieldPosition(startingPose.getTranslation()), startingPose.getRotation());
		return pose;
	}

}
