package frc.robot.commands.light;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class SetAllOrangeCommand extends CommandBase {

    private final LightSubsystem lightSubsystem;


    public SetAllOrangeCommand(LightSubsystem lightSubsystem) {

        this.lightSubsystem = lightSubsystem;

        addRequirements(lightSubsystem);
    }

    @Override
    public void initialize() {

        lightSubsystem.setPattern(0, Patterns.createSolidPattern(149, Color.kOrange));
    }

    @Override
    public void end(boolean interrupted) {
        lightSubsystem.off();
    }

}
