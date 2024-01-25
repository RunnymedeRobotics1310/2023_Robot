package frc.robot.commands.light;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;


public class SetHoldingCubeLightsCommand extends Command {

    private final LightSubsystem lightSubsystem;


    public SetHoldingCubeLightsCommand(LightSubsystem lightSubsystem) {

        this.lightSubsystem = lightSubsystem;

        addRequirements(lightSubsystem);
    }


    LightPattern lightsOn = new LightPattern();

    @Override
    public void initialize() {


        lightsOn = new LightPattern();

        lightsOn.append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP)
            .append(Patterns.CUBE_PICKUP);
    }

    @Override
    public void execute() {

        lightSubsystem.setPattern(0, lightsOn);



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
