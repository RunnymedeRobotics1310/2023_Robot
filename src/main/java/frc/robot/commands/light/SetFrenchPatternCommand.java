package frc.robot.commands.light;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class SetFrenchPatternCommand extends CommandBase {

    private final LightSubsystem lightSubsystem;


    public SetFrenchPatternCommand(LightSubsystem lightSubsystem) {

        this.lightSubsystem = lightSubsystem;

        addRequirements(lightSubsystem);
    }

    @Override
    public void initialize() {
        lightSubsystem.setPattern(0, Patterns.FRENCH);
    }

    @Override
    public void end(boolean interrupted) {
        lightSubsystem.off();
    }

}
