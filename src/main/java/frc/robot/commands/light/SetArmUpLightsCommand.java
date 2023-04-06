package frc.robot.commands.light;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;


public class SetArmUpLightsCommand extends CommandBase {

    private final LightSubsystem lightSubsystem;


    public SetArmUpLightsCommand(LightSubsystem lightSubsystem) {

        this.lightSubsystem = lightSubsystem;

        addRequirements(lightSubsystem);
    }

    double       executeStartTime;
    double       startTime;
    double       period;
    double       endTime    = 5000;
    LightPattern lightsOn   = new LightPattern();
    LightPattern lightsGlow = new LightPattern();

    @Override
    public void initialize() {

        startTime        = System.currentTimeMillis();
        executeStartTime = System.currentTimeMillis();
        period           = 500;
        lightsOn         = new LightPattern();
        lightsGlow       = new LightPattern();


        lightsOn.append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP)
            .append(Patterns.ARM_UP);

        lightsGlow.append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW)
            .append(Patterns.DEFAULT_GLOW);

    }

    @Override
    public void execute() {

        if (System.currentTimeMillis() - executeStartTime < period) {
            lightSubsystem.setPattern(0, lightsOn);

        }
        else if (System.currentTimeMillis() - executeStartTime > period) {
            lightSubsystem.setPattern(0, lightsGlow);
        }

        if (System.currentTimeMillis() - executeStartTime >= period * 2) {
            executeStartTime = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {

        // This command ends when the piece is dropped
        return false;

    }

    @Override
    public void end(boolean interrupted) {

    }

}
