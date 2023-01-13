package frc.robot.subsystems.position;

public class Speed {
    public static final Speed FAST = new Speed(1.5);
    public static final Speed MEDIUM = new Speed(0.75);
    public static final Speed SLOW = new Speed(0.05);
    private final double metresPerSecond;
    public Speed(double metresPerSecond) {
        this.metresPerSecond = metresPerSecond;
    }
}
