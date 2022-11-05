package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    // The motors on the top part of the intake thing.
    private final VictorSPX intakeMotor = new VictorSPX(IntakeConstants.INTAKE_MOTOR_ADDRESS);

    private double intakeMotorSetpoint = 0;

    private DoubleSolenoid intakeRollerPiston = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.INTAKE_ROLLER_PISTON_ADDRESS,
            IntakeConstants.INTAKE_ROLLER_PISTON_ADDRESS + 1);

    Value intakeRollerValue = Value.kOff;

    private DoubleSolenoid intakeHoodPiston = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.INTAKE_HOOD_PISTON_ADDRESS,
            IntakeConstants.INTAKE_HOOD_PISTON_ADDRESS + 1);

    Value intakeHoodValue = Value.kOff;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_REVERSED);

        // Default to intake retracted
        setHoodPiston(false);
        setRollerPiston(false);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return 0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return 0;
    }

    public void setHoodPiston(boolean deploy) {

        if (deploy) {
            intakeHoodValue = Value.kReverse;
        } else {
            intakeHoodValue = Value.kForward;
        }

        intakeHoodPiston.set(intakeHoodValue);
    }

    public void setRollerPiston(boolean deploy) {

        if (deploy) {
            intakeRollerValue = Value.kReverse;
        } else {
            intakeRollerValue = Value.kForward;
        }

        intakeRollerPiston.set(intakeRollerValue);
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return 0;
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
    }

    /**
     * Set the speed on the intake motor
     * 
     * @param speed in the range -1.0 to 1.0, 0 = stopped;
     */
    public void setMotorSpeed(double speed) {
        intakeMotorSetpoint = speed;
        intakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Motor", intakeMotorSetpoint);
        SmartDashboard.putString("Intake Hood", intakeHoodValue.toString());
        SmartDashboard.putString("Intake Roller", intakeRollerValue.toString());
    }
}
