package frc.robot.commands.light;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;


public class DefaultLightCommand extends CommandBase {

    private final LightSubsystem lightSubsystem;


    public DefaultLightCommand(LightSubsystem lightSubsystem) {

        this.lightSubsystem = lightSubsystem;

        addRequirements(lightSubsystem);
    }

    LightPattern lightsGlow = new LightPattern();

    @Override
    public void initialize() {

        lightsGlow = new LightPattern();

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

        lightSubsystem.setPattern(0, lightsGlow);


    }

    @Override
    public void execute() {

        // lightSubsystem.setPattern(0, lightsGlow);

    }

    @Override
    public void end(boolean interrupted) {

        lightSubsystem.off();

    }

}
