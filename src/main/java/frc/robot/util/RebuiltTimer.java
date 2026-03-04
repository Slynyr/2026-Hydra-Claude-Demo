package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.util.Color;
import java.util.HashMap;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;

/**
 * Timer class to keep track of hub state + additional information in 2026 game Rebuilt
 * @author Anvay Mathur
 */
public class RebuiltTimer {

	public enum HubState {
		RED,
		BLUE,
		BOTH
	}

	public enum MatchState{
		AUTO,
		TRANSITION,
		SHIFT1,
		SHIFT2,
		SHIFT3,
		SHIFT4,
		ENDGAME
	}

	public enum AutoWinner {
		RED,
		BLUE,
		ERROR
	}

	public enum GameState{
		ACQUIRE,
		LAUNCH,
		PASS,
		IDLE,
		TRAVEL,
		CLIMB,
		UNCLIMB
	}

	private HubState activeHub;
	public MatchState currentShift;
	private boolean redIsActiveFirst;
	public AutoWinner autoWinner;
	private Timer timerFMS;
	private long timer;
	private int fuel;
	public Time timeInShift;
	public boolean autoNotifSent;
	public Color autoWinnerColor;

	private HashMap<GameState, Double> gameStrat; 
	private GameState[] gameStrategy;

//  TODO: Get accurate values
	private static final double            FUEL_PER_SECOND =	7;
	private static final LinearVelocity    ROBOT_SPEED     =	MetersPerSecond.of(4);
    private static final Time              CLIMB_TIME		=	Seconds.of(3);
	public static final Color				AUTO_ERROR		= 	new Color("#FFFF00");
	private static final Color				AUTO_BLUE		= 	new Color("#0000FF");
	private static final Color				AUTO_RED		= 	new Color("#FF0000");

	public RebuiltTimer() {
		this.activeHub = HubState.BOTH;
		this.currentShift = MatchState.AUTO;
		this.redIsActiveFirst = false;
		this.autoWinner = AutoWinner.ERROR;
		this.timerFMS = new Timer();
		this.timer = -1;
		this.fuel = 0;
		this.gameStrat = new HashMap<>();
		this.autoNotifSent = false;
		SmartDashboard.putBoolean("Timer/Red is active first?", redIsActiveFirst);
		autoWinnerColor = AUTO_ERROR;
	}

	/**
	 * Start timer (not needed if using {@link DriverStation#getMatchTime()})
	 */
	public void start(){
		timer = System.currentTimeMillis();
		timerFMS.start();
	}

	public void periodic(Drive drive){
		if (autoWinner == AutoWinner.RED) this.autoWinnerColor = AUTO_RED;
		else if (autoWinner == AutoWinner.BLUE) this.autoWinnerColor = AUTO_BLUE;
		else this.autoWinnerColor = AUTO_ERROR;

		SmartDashboard.putNumber("Timer/Time In shift", getTimeInShift().in(Seconds));

		SmartDashboard.putString("Timer/Current Shift", currentShift.toString());

		SmartDashboard.putString("Timer/AutoWinner", autoWinnerColor.toHexString());

		SmartDashboard.putString("Timer/IsHubActive", isHubActive()
				? new Color("#00FF00").toHexString()
				: new Color("#FF0000").toHexString());

		SmartDashboard.putNumber("Timer/Fuel", getFuel());

		SmartDashboard.putNumber("Timer/Time Left To Acquire",
				timeToAcquire(
						drive::getPose
				).in(Seconds));

		SmartDashboard.putNumber("Timer/Time to score", scoreTime());

		SmartDashboard.putNumber("Timer/Time to travel",
				timeToPose(
						drive::getPose,
						() -> getClosestScoringPosition(drive::getPose)
				).in(Seconds));
	}

	/**
	 * Gets the game specific message about who won auto from {@link DriverStation#getGameSpecificMessage()}
	 * and returns the alliance that won auto. If no message is received or message is corrupted, returns error state.
	 */
	public void getAutoWinner(){

		String gameData = DriverStation.getGameSpecificMessage();

		if (!gameData.isEmpty()){
			switch (gameData.charAt(0)){
				case 'B'-> {
					this.autoWinner = AutoWinner.BLUE;
					redIsActiveFirst = true;
				} case 'R'->{
					this.autoWinner = AutoWinner.RED;
					redIsActiveFirst = false;
				} default -> this.autoWinner = AutoWinner.ERROR;
			}
		}

		if ((DriverStation.isTeleop() && gameData.isEmpty()) || this.autoWinner == AutoWinner.ERROR){
			Elastic.Notification autoErrorNotif = 
				new Elastic.Notification(
					Elastic.NotificationLevel.ERROR, 
					"AUTO Winner Not Detected", 
					"The AUTO winner was not detected, please manually input the first active hub"
				);            
			if (!autoNotifSent) Elastic.sendNotification(autoErrorNotif);
			autoNotifSent = true;
			this.autoWinner = AutoWinner.ERROR;
			redIsActiveFirst = SmartDashboard.getBoolean("Timer/Red is active first?", redIsActiveFirst);
			if (redIsActiveFirst) this.autoWinner = AutoWinner.BLUE;
			else this.autoWinner = AutoWinner.RED;
		}
	}
//    get match time
//          Manual match Time or Timer class, or Fms match time

	/**
	 * Counts down the match time from 160 seconds
	 * If using {@link DriverStation#getMatchTime()} auto counts down from 20 and teleop from 140
	 * @return Match time
	 */
	public double getMatchTime(){
		// return Timer.getMatchTime();
		return DriverStation.getMatchTime();
		// return 160 - timerFMS.get();
		// return 160 - (System.currentTimeMillis() - timer);
	}

	// Tracks when the shifts change
	/**
	 * Tracks the shift state using the match time from {@link #getMatchTime()} 
	 */
	public void trackShift(){
		double matchTime = getMatchTime();
		// Transition Time
		if (DriverStation.isAutonomous()){
			// Auto
			currentShift = MatchState.AUTO;

		} else if (matchTime > 130){
			// Transition
			activeHub = HubState.BOTH;
			currentShift = MatchState.TRANSITION;

		} else if (matchTime > 105){
			// Shift 1
			if (redIsActiveFirst)
				activeHub = HubState.RED;
			else 
				activeHub = HubState.BLUE;

			currentShift = MatchState.SHIFT1;

		} else if (matchTime > 80){
			// Shift 2
			if (redIsActiveFirst)
				activeHub = HubState.BLUE;
			else
				activeHub = HubState.RED;

			currentShift = MatchState.SHIFT2;
		} else if(matchTime > 55){
			// Shift 3
			if (redIsActiveFirst)
				activeHub = HubState.RED;
			else 
				activeHub = HubState.BLUE;

			currentShift = MatchState.SHIFT3;

		} else if (matchTime > 30){
			// Shift 4
			if (redIsActiveFirst)
				activeHub = HubState.BLUE;
			else
				activeHub = HubState.RED;
			currentShift = MatchState.SHIFT4;

		} else {
			// Endgame
			activeHub = HubState.BOTH;
			currentShift = MatchState.ENDGAME;
		}
	}

//    Get time to current shift
//          Track shift time
// Track how much time is left in the current shift in seconds. counts down from shift start to shift end
	/**
	 * Tracks how much time is left in the current shift, counting down from shift start time to shift end time.
	 * @return time left in current shift
	 */
	public Time getTimeInShift(){
		double matchTime = getMatchTime();

		if (DriverStation.isAutonomous())
			this.timeInShift = Seconds.of(matchTime);
		else if (currentShift == MatchState.TRANSITION)
			this.timeInShift = Seconds.of(matchTime - 130);
		else if (currentShift == MatchState.SHIFT1)
			this.timeInShift = Seconds.of(matchTime - 105);
		else if (currentShift == MatchState.SHIFT2)
			this.timeInShift = Seconds.of(matchTime - 80);
		else if (currentShift == MatchState.SHIFT3)
			this.timeInShift = Seconds.of(matchTime - 55);
		else if (currentShift == MatchState.SHIFT4)
			this.timeInShift = Seconds.of(matchTime - 30);
		else if(currentShift == MatchState.ENDGAME)
			this.timeInShift = Seconds.of(matchTime);
		else 
			return Seconds.of(0.0);
		
		return this.timeInShift;
	}

//    Get time to next shift
//         Track shift time

//    Active? Inactive?
//          Hub state tracking, track shift time
	/**
	 * Checks to see if the active hub is the same as the current alliance
	 * @return if the hub is active
	 */
	public boolean isHubActive(){
		if (activeHub == HubState.BOTH)
			return true;
		if (DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Red)
				return  activeHub == HubState.RED;
			else if (DriverStation.getAlliance().get() == Alliance.Blue)
				return activeHub == HubState.BLUE;
			else return false;
		} else return false;
	}

//    Get time to get back to 
//          Calculate distance relative to time
	/**
	 * time it takes to get from robot pose to target pose, each axis is added individually.
	 * So it imagines you only move in straight lines horizontally and vertically. 
	 * Uses {@link #ROBOT_SPEED} to calculate the time
	 * @param robot robot pose
	 * @param target target pose
	 * @return time it takes to get from robot pose to target pose
	 */
	public Time timeToPose(Supplier<Pose2d> robot, Supplier<Pose2d> target){
		Pose2d robotPose = robot.get();
		Pose2d targetPose = target.get();

		Distance xDiff = targetPose.getMeasureX().minus(robotPose.getMeasureX());
		Distance yDiff = targetPose.getMeasureY().minus(robotPose.getMeasureY());

		xDiff = Meters.of(xDiff.abs(Meters));
		yDiff = Meters.of(yDiff.abs(Meters));

		Time xTime = xDiff.div(ROBOT_SPEED);
		Time yTime = yDiff.div(ROBOT_SPEED);


		return xTime.plus(yTime);
	}

	/**
	 * Returns 1 of 2 constant positions based on left or right side of the field
	 * Flips if on red alliance
	 * @param robot the current position of the robot
	 * @return 1 of 2 constant scoring positions based on place on field
	 */
	public Pose2d getClosestScoringPosition(Supplier<Pose2d> robot){
		Pose2d robotPose = robot.get();

		final Pose2d LEFT = new Pose2d(Meters.of(3.209), Meters.of(5.5), Rotation2d.kZero);
		final Pose2d RIGHT = new Pose2d(Meters.of(3.209), Meters.of(2.6), Rotation2d.kZero);

		Pose2d targetPose;

		if (AutoBuilder.shouldFlip()){ 
			if (!Constants.kField.LEFT_HALF.contains(robotPose.getTranslation())) targetPose = LEFT;
			else  targetPose = RIGHT;
			targetPose = FlippingUtil.flipFieldPose(targetPose);
		} else {
			if (Constants.kField.LEFT_HALF.contains(robotPose.getTranslation())) targetPose = LEFT;
			else  targetPose = RIGHT;
		}
		return targetPose;
	}

//    Time left to acquire
//         Time left in hub  - Time to shoot - time to get back 
	/**
	 * The amount of time left to acquire in the shift based on how long it takes to score and how long it takes to get to scoring position
     * if hub not active, doesn't account for time to score fuel
	 * @param robotPose the current position of the robot
	 * @return the time left in the shift available for acquiring
	 */
	public Time timeToAcquire(Supplier<Pose2d> robotPose){
        if (this.currentShift == MatchState.ENDGAME) return this.timeInShift.minus(Seconds.of(scoreTime())).minus(timeToPose(robotPose, () -> getClosestScoringPosition(robotPose))).minus(CLIMB_TIME);
        if (isHubActive())	return this.timeInShift.minus(Seconds.of(scoreTime())).minus(timeToPose(robotPose, () -> getClosestScoringPosition(robotPose)));
        else                return this.timeInShift.minus(timeToPose(robotPose, () -> getClosestScoringPosition(robotPose)));
	}

//    Time needed/left to shoot
//         Fuel in robot / (fuel/second)
	/**
	 * Time needed to score based on the amount of {@link #fuel} and the set {@link #FUEL_PER_SECOND}
	 * @return time needed to score
	 */
	public double scoreTime(){
		return this.fuel / FUEL_PER_SECOND;
	}

//    Fuel Gauge
	/**
	 * Set the amount of fuel the robot has {@link #fuel}
	 * @param fuel the amount of fuel to set
	 */
	public void setFuel(int fuel){
		this.fuel = Math.max(0, fuel);
	}

	/**
	 * increments the amount of fuel the robot has {@link #fuel}
	 * @param fuel the amount of fuel to increment by
	 */
	public void incrementFuel(int fuel){
		this.fuel = Math.max(0, this.fuel + fuel);
	}

	/**
	 * Gets the amount of {@link #fuel} in the robot
	 * @return the amount of fuel
	 */
	public int getFuel(){
		return this.fuel;
	}

	//    Next "Phase"

	//    Next Action

//    Time left to Feed
//      time in shift - (time needed to get back + time needed to shoot) and something else

//    Button: Current Action Done

//    Idle time

//    Map out match strategy
	/**
	 * Set the match strategy as a hashmap of {@link GameState} and time
	 * @param gameStrat hashmap of {@link GameState} and time
	 */
	public void setGameStrategy(HashMap<GameState,Double> gameStrat){
		this.gameStrat = gameStrat;
	}

	/**
	 * Gets the match strategy as a hashmap of {@link GameState} and time (double)
	 * @return the current game strategy
	 */
	public HashMap<GameState,Double> getGameStrat(){
		return this.gameStrat;
	}

	/**
	 * Sets the game strategy as an array of {@link GameState} in order of events
	 * @param gameStrat the game strategy
	 */
	public void setGameStrategy(GameState[] gameStrat){
		this.gameStrategy = gameStrat;
	}

	/**
	 * Gets the current game strategy as an array of {@link GameState} in order of events
	 * @return the set game strategy
	 */
	public GameState[] getGameStrategy(){
		return this.gameStrategy;
	}   
	

	
}
