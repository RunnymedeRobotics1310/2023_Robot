package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem,
            SendableChooser<String> autoChooser) {

        String selectedAuto = autoChooser.getSelected();

        // The selector should always return one of the auto
        // patterns, if not, then do nothing
        if (selectedAuto == null) {
            System.out.println("*** ERROR - NULL auto selected ***");
            addCommands(new InstantCommand());
            return;
        }

        // Placeholder for auto commands
        switch (selectedAuto) {

            case AutoConstants.AUTO_PATTERN_DO_NOTHING:
                // Do nothing
                System.out.println("Do nothing auto selected");
                addCommands(new InstantCommand());
                break;

            case AutoConstants.AUTO_PATTERN_SHOOT:
                // Do nothing
                System.out.println("Do shoot selected");
                addCommands(new ShootCommand(shooterSubsystem, intakeSubsystem));
                break;

            default:
                // How did we get here?
                System.out.println("Auto selection(" + selectedAuto + ") was not programmed!");
                addCommands(new InstantCommand());
        }
    }

}
