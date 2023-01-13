package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
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
        System.out.println("Selected auto command: "+selectedAuto);
        switch (selectedAuto) {

        case AutoConstants.AUTO_PATTERN_DO_NOTHING:
            // Do nothing
            addCommands(new InstantCommand());
            break;

        case AutoConstants.AUTO_PATTERN_SHOOT:
            // Do nothing
            addCommands(
                    new ShootCommand(
                            ShooterConstants.SHOOT_HIGH_SPEED,
                            shooterSubsystem, intakeSubsystem));
            break;

        case AutoConstants.AUTO_PATTERN_MID_DIRECT_CHARGE:
            addCommands(new MidDirectChargeCommand());
            break;

        default:
            // How did we get here?
            System.out.println("Auto selection(" + selectedAuto + ") was not programmed!");
            addCommands(new InstantCommand());
        }
    }

}
