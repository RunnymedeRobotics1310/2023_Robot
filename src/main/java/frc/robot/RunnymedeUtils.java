package frc.robot;

public class RunnymedeUtils {

    /**
     * Get the fastest possible speed to travel to the <code>distanceToTravel</code>,
     * given the specified parameters.
     * <p>
     * The distance can be provided in any unit - encoder counts, centimetres, etc.
     * The speed can also be provided in any unit.
     * <p>
     * The returned speed will accelerate from the <code>startSpeed</code> to the
     * <code>maxSpeed</code> over the <code>accelerationDistance</code>, then run at
     * <code>maxSpeed</code>, then decelerate to the <code>endSpeed</code> over the
     * <code>decelerationDistance</code>.
     * <p>
     * If the distance is too short to reach cruising speed, then the <code>maxSpeed</code>,
     * <code>accelerationDistance</code> and <code>decelerationDistance</code> will all
     * be reduced proportionately so that the speed can be maximized while preserving
     * the acceleration and deceleration behaviour.
     * <p>
     * When the <code>distanceTravelled</code> reaches or exceeds the <code>distanceToTravel</code>
     * the journey is complete and a speed of 0 is returned.
     * <p>
     * Note that the <code>distanceTravelled</code> must be specified, so calling
     * code needs to keep track of it (for example, by asking a motor
     * encoder what the distance travelled is).
     * @param distanceToTravel The distance that should be travelled. This must be
     * a non-negative number
     * @param distanceTravelled The distance travelled so far. This value is used to
     * calculate the output speed based on the parameters provided. This must be a
     * non-negative number
     * @param startSpeed The speed at the beginning of the acceleration period. This
     * number must be a non-negative number.
     * @param maxSpeed The target maximum speed to be reached during the cruise stage.
     * This number must be greater than startSpeed
     * @param endSpeed The speed at the end of the deceleration period. This number
     * must be a non-negative number, and less than or equal to maxSpeed.
     * @param accelerationDistance the distance (in the same unit as the other distance
     * parameters) over which the speed will increase up to the <code>maxSpeed</code>.
     * <p>
     * If the sum of <code>accelerationDistance</code> and <code>decelerationDistance</code>
     * exceeds the <code>distanceToTravel</code>, the acceleration will be proportionately
     * scaled down to ensure approximately the same acceleration curve.
     * @param decelerationDistance the distance (in the same unit as the other distance
     * parameters) over which the speed will decrease down to the <code>minSpeed</code>
     * <p>
     * If the sum of <code>accelerationDistance</code> and <code>decelerationDistance</code>
     * exceeds the <code>distanceToTravel</code>, the acceleration will be proportionately
     * scaled down to ensure approximately the same acceleration curve.
     * @throws IllegalArgumentException if the input parameters are invalid
     */
    public static double calculateFastestSpeed(final double distanceToTravel, final double distanceTravelled,
        final double startSpeed, double maxSpeed, final double endSpeed,
        double accelerationDistance, double decelerationDistance) throws IllegalArgumentException {

        if (distanceToTravel < 0) throw new IllegalArgumentException("Distance to travel "+distanceToTravel+" must not be negative");
        if (distanceTravelled < 0) throw new IllegalArgumentException("Distance travelled "+distanceTravelled+" must not be negative");
        if (maxSpeed < startSpeed) throw new IllegalArgumentException("Start speed "+startSpeed+" must not exceed max speed "+maxSpeed);
        if (endSpeed > maxSpeed) throw new IllegalArgumentException("End speed "+endSpeed+" must not exceed max speed "+maxSpeed);

        final boolean LOG_ENABLED = true;

        if (distanceTravelled >= distanceToTravel) {
            return 0;
        }

        // ensure that we have enough space to accelerate all the way to max and then slow down safely
        double ratio = distanceToTravel / (accelerationDistance + decelerationDistance);
        if (ratio < 1) {

            maxSpeed = maxSpeed * ratio;
            accelerationDistance = accelerationDistance * ratio;
            decelerationDistance = decelerationDistance * ratio;

            if (LOG_ENABLED) {
                System.out.println("Short distance - reducing max speed to "+maxSpeed);
            }
        }

        double remaining = distanceToTravel - distanceTravelled;
        final double speed;

        if (distanceTravelled < accelerationDistance) {

            // speeding up
            double pctDriven = (distanceTravelled/accelerationDistance);
            speed = Math.max(startSpeed, pctDriven*maxSpeed);
            if (LOG_ENABLED) {
                System.out.println("Accelerating ("+Math.round(pctDriven*100)+"% done). Speed: "+(Math.round(speed*100)/100d)+" dist/total: "+(Math.round(distanceTravelled))+"/"+distanceToTravel);
            }

        }
        else if (remaining < decelerationDistance) {

            // slowing down
            double pctToGo = (remaining/decelerationDistance);
            speed = Math.max(endSpeed, pctToGo*maxSpeed);
            if (LOG_ENABLED) {
                System.out.println("Decelerating ("+Math.round(pctToGo*100)+"% done). Speed: "+(Math.round(speed*100)/100d)+" dist/total: "+(Math.round(distanceTravelled))+"/"+distanceToTravel);
            }

        }
        else {

            // cruising
            speed = maxSpeed;

        }

        return speed;
    }
}
