package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.*;

abstract class BaseArmCommand extends CommandBase {


    private final ArmSubsystem armSubsystem;

    protected BaseArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    protected final void printArmState() {
        System.out.println("BaseArmCommand: armSubsystem.getHeldGamePiece: " + armSubsystem.getHeldGamePiece());
        System.out.println("BaseArmCommand: Arm height: " + armSubsystem.getArmLiftEncoder());
        System.out.println("BaseArmCommand: Arm extent: " + armSubsystem.getArmExtendEncoder());
        System.out.println("BaseArmCommand: Pincher: " + armSubsystem.getPincherEncoder());
    }

    protected final void stopArmMotors() {
        armSubsystem.setArmExtendSpeed(0);
        armSubsystem.setPincherSpeed(0);
        armSubsystem.setArmLiftSpeed(0);
    }

    /**
     * Start moving the motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should be moving
     * @return true if moving, false if not
     */
    protected final boolean moveArmLiftToEncoderCount(double targetCount, double speed) {
        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getArmLiftEncoder() - targetCount;
        if (Math.abs(gap) > ARM_LIFT_MOTOR_TOLERANCE) {
            armSubsystem.setArmLiftSpeed(gap > 0 ? -absSpd : absSpd);
            return true;
        }
        else {
            armSubsystem.setArmLiftSpeed(0);
            return false;
        }
    }

    /**
     * Start moving the motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should be moving
     * @return true if moving, false if not
     */
    protected final boolean moveArmExtendToEncoderCount(double targetCount, double speed) {
        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getArmExtendEncoder() - targetCount;
        if (Math.abs(gap) > PINCHER_MOTOR_TOLERANCE) {
            armSubsystem.setArmExtendSpeed(gap > 0 ? -absSpd : absSpd);
            return true;
        }
        else {
            armSubsystem.setArmExtendSpeed(0);
            return false;
        }
    }

    /**
     * Start moving the motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should be moving
     * @return true if moving, false if not
     */
    protected final boolean movePincherToEncoderCount(double targetCount, double speed) {
        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getPincherEncoder() - targetCount;
        if (Math.abs(gap) > PINCHER_MOTOR_TOLERANCE) {
            armSubsystem.setPincherSpeed(gap > 0 ? -absSpd : absSpd);
            return true;
        }
        else {
            armSubsystem.setPincherSpeed(0);
            return false;
        }
    }
}
