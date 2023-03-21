package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The Base Runnymede Command implements command helpers to aid with logging
 */
public class RunnymedeCommandBase extends CommandBase {

    private long    startTime     = 0;
    private String  finishReason  = "";

    List<Subsystem> subsystemList = new ArrayList<>();

    public void logCommandStart() {

        logCommandStart(new Subsystem[0]);
    }

    public void logCommandStart(String msg) {

        logCommandStart(msg, new Subsystem[0]);
    }

    public void logCommandStart(Subsystem... subsystemList) {

        logCommandStart(null, subsystemList);
    }

    public void logCommandStart(String commandParms, Subsystem... subsystemList) {

        // Capture the subsystem list associated with this command
        if (subsystemList != null && subsystemList.length > 0) {
            this.subsystemList.addAll(Arrays.asList(subsystemList));
        }
        else {
            this.subsystemList.addAll(getRequirements());
        }

        logCommandState("STARTING", commandParms);

        startTime = System.currentTimeMillis();
    }

    public void logCommandEnd(boolean interrupted) {

        logCommandEnd(interrupted, null);
    }

    public void logCommandEnd(boolean interrupted, String endMsg) {

        String state = "ENDED";

        if (interrupted) {
            state = "INTERUPTED";
        }

        logCommandState(state, endMsg);
    }

    public void logStateTransition(String transition) {
        logCommandState(transition, null);
    }

    public void logStateTransition(String transition, String msg) {
        logCommandState(transition, msg);
    }

    public void logCommandState(String state, String msg) {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName())
            .append(" : ").append(state);

        if (startTime != 0) {
            sb.append(" at ").append(System.currentTimeMillis() - startTime).append("ms");
        }

        if (!finishReason.isEmpty()) {
            sb.append(" : ").append(finishReason);
        }

        if (msg != null) {
            sb.append(" : ").append(msg);
        }

        // Print the subsystems as passed in on the command start
        for (Subsystem subsystem : subsystemList) {
            sb.append("\n   ").append(subsystem.toString());
        }

        System.out.println(sb.toString());
    }

    public void setFinishReason(String finishReason) {
        this.finishReason = finishReason;
    }


}
