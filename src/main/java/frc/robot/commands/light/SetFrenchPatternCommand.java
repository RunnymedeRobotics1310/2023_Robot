package frc.robot.commands.light;

import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.LightSubsystem;

public class SetFrenchPatternCommand extends RunnymedeCommandBase {

    private final LightSubsystem lightSubsystem;


    public SetFrenchPatternCommand(LightSubsystem lightSubsystem) {

        this.lightSubsystem = lightSubsystem;

        addRequirements(lightSubsystem);
    }

    @Override
    public void initialize() {

        LightPattern p = new LightPattern();
        p.append(Patterns.FRENCH)
            .append(Patterns.FRENCH);

        this.lightSubsystem.setPattern(0, p);

    }

    @Override
    public void end(boolean interrupted) {
        lightSubsystem.off();
    }

}
