package frc.robot.commands.light;

import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kBlueViolet;
import static edu.wpi.first.wpilibj.util.Color.kLightPink;
import static edu.wpi.first.wpilibj.util.Color.kOrangeRed;
import static edu.wpi.first.wpilibj.util.Color.kRed;
import static edu.wpi.first.wpilibj.util.Color.kWhite;
import static edu.wpi.first.wpilibj.util.Color.kYellow;

import edu.wpi.first.wpilibj.util.Color;

public class Patterns {
    public static final LightPattern FRENCH       = new LightPattern(kBlue, kRed, kWhite);
    public static final LightPattern RUNNYMEDE    = new LightPattern(kRed, kBlack);
    public static final LightPattern CONE_PICKUP  = new LightPattern(kYellow, kYellow, kYellow, kYellow, kYellow, kYellow,
        kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow, kYellow);

    public static final LightPattern CUBE_PICKUP  = new LightPattern(kBlueViolet, kBlueViolet, kBlueViolet, kBlueViolet,
        kBlueViolet, kBlueViolet, kBlueViolet, kBlueViolet, kBlueViolet, kBlueViolet, kBlueViolet, kBlueViolet, kBlueViolet,
        kBlueViolet, kBlueViolet);

    public static final LightPattern ARM_UP       = new LightPattern(kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed,
        kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed, kOrangeRed);

    public static final LightPattern GYRO_ZERO    = new LightPattern(kLightPink);
    public static final LightPattern DEFAULT_GLOW = new LightPattern(kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed,
        kRed, kRed, kRed, kRed, kRed);

    public static final LightPattern OFF          = new LightPattern(kBlack);

    public static LightPattern createSolidPattern(int length, Color color) {

        LightPattern pattern = new LightPattern();
        for (int i = 0; i < length; i++) {
            pattern.append(color);
        }
        return pattern;
    }

}
