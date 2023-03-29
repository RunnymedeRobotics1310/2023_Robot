package frc.robot;

import org.junit.Test;

import static frc.robot.RunnymedeUtils.calculateFastestSpeed;
import static org.junit.Assert.assertEquals;

public class RunnymedeUtilsTests {

    @Test
    public void testCalculateFastestSpeedShortDistance() {
        // configuration
        double startSpeed = 0.3;
        double endSpeed = 0.3;
        double maxSpeed = 1.0;
        double accelDist = 40;
        double decelDist = 70;

        // distance to travel
        double goal = 40;

        test(0, goal, goal, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 5, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 10, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.4, goal, 15, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 20, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 25, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 30, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 35, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0, goal, 40, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0, goal, 45, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
    }

    @Test
    public void testCalculateFastestSpeedMidDistance() {
        // configuration
        double startSpeed = 0.3;
        double endSpeed = 0.3;
        double maxSpeed = 1.0;
        double accelDist = 40;
        double decelDist = 70;

        // distance to travel
        double goal = 180;
        test(0, goal, goal, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 5, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 10, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.4, goal, 15, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.5, goal, 20, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.625, goal, 25, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.75, goal, 30, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.875, goal, 35, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 40, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 45, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 50, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 55, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 65, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 70, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 75, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 80, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 85, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 90, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 95, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 100, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 105, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 110, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 115, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.85, goal, 120, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.78, goal, 125, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.71, goal, 130, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.64, goal, 135, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.57, goal, 140, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.5, goal, 145, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.42, goal, 150, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.4, goal, 155, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.3, goal, 160, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.3, goal, 165, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.3, goal, 170, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.3, goal, 175, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0, goal, 180, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0, goal, 185, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
    }

    @Test
    public void testCalculateFastestSpeedSlowToStop() {
        // configuration
        double startSpeed = 0.15;
        double endSpeed = 0.0;
        double maxSpeed = 1.0;
        double accelDist = 40;
        double decelDist = 70;

        // distance to travel
        double goal = 180;

        test(0, goal, goal, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.15, goal, 5, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.3, goal, 10, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.4, goal, 15, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.5, goal, 20, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.625, goal, 25, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.75, goal, 30, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0.875, goal, 35, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 40, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 45, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 50, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 55, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 65, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 70, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 75, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 80, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 85, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 90, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 95, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 100, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 105, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 110, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(1, goal, 115, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.85, goal, 120, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.78, goal, 125, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.71, goal, 130, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.64, goal, 135, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.57, goal, 140, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.5, goal, 145, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.42, goal, 150, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.4, goal, 155, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.3, goal, 160, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.21, goal, 165, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.14, goal, 170, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(.07, goal, 175, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0, goal, 180, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
        test(0, goal, 185, startSpeed, maxSpeed, endSpeed, accelDist, decelDist);
    }

    private void test(double expected,
        double goal, double travelled, double start, double max, double end, double accel, double decel) {
        double speed = calculateFastestSpeed(goal, travelled, start, max, end, accel, decel);
//        System.out.println("Speed: "+speed);
        String msg = "Goal: "+goal+", travelled: "+travelled+", startSpeed: "+start+", maxSpeed: "+max+", endSpeed: "+end+", accelDist: "+accel+", decelDist: "+decel;
        assertEquals(msg, expected, speed, 0.075);
    }
}
