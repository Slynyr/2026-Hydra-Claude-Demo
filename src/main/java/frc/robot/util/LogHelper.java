package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class LogHelper {
    /**
     * Logs an "event" by setting the value of a given field to the current time, allowing the event to be plotted in a
     * line graph to see when the event was called/propagated.
     *
     * @param name name of event field (e.g. Subsystem/MyEventName)
     */
    public static void logEvent(String name) {
        Logger.recordOutput("Event/" + name, Timer.getFPGATimestamp());
    }
}
