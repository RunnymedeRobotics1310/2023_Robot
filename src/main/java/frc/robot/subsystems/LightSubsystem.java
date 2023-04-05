package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.light.LightPattern;

public class LightSubsystem extends SubsystemBase {

    private static final int           LIGHT_STRIP_LENGTH = 150;
    private final AddressableLED       ledStrip           = new AddressableLED(0);
    private final AddressableLEDBuffer buffer;

    public LightSubsystem() {
        System.out.println("Initializing LightSubsystem with ledStripBuffer of length " + LIGHT_STRIP_LENGTH);

        // Length is expensive to set, so only set it once, then just update data
        ledStrip.setLength(LIGHT_STRIP_LENGTH);
        buffer = new AddressableLEDBuffer(LIGHT_STRIP_LENGTH);

        // Set the data
        ledStrip.setData(buffer);
        ledStrip.start();
    }

    public void off() {
        for (int i = 0; i < LIGHT_STRIP_LENGTH; i++) {
            buffer.setLED(i, Color.kBlack);
        }
        safelySetLights();
    }

    public void setPattern(int sectionStart, LightPattern pattern) {
        List<Color> colors = pattern.getLights();
        for (int i = 0; i < colors.size(); i++) {
            Color color = colors.get(i);
            buffer.setLED(sectionStart + i, color);
        }
        safelySetLights();
    }

    private void safelySetLights() {

        /*
         * To stay within the RIO bounds, we would need to
         * draw < 2A or 12W total power for the LEDs the way
         * they are configured.
         *
         * The lights probably draw about 20mA/colour
         *
         * @ full on, so RED=20mA, BLUE=20mA, WHITE=60mA.
         * Less if the LEDs are dimly lit. Single colour,
         * we can safely light 100 LEDs, Full white, about 33 LEDs.
         */
        double               MAX_MILLIAMPS = 2000;

        // don't destroy the desired levels by overwriting buffer
        AddressableLEDBuffer dimmedBuffer  = null;

        double               ratio;
        do {
            AddressableLEDBuffer workingBuffer = dimmedBuffer == null ? buffer : dimmedBuffer;
            double               milliamps     = 0;
            for (int i = 0; i < workingBuffer.getLength(); i++) {
                Color color = workingBuffer.getLED(i);
                milliamps += color.red * 20;
                milliamps += color.blue * 20;
                milliamps += color.green * 20;
            }
            ratio = milliamps / MAX_MILLIAMPS;
            if (ratio >= 1) {
                if (dimmedBuffer == null) {
                    dimmedBuffer = new AddressableLEDBuffer(buffer.getLength());
                }
                for (int i = 0; i < workingBuffer.getLength(); i++) {
                    Color  color       = workingBuffer.getLED(i);
                    double r           = color.red / ratio;
                    double g           = color.green / ratio;
                    double b           = color.blue / ratio;
                    Color  dimmerColor = new Color(Math.max(0, r), Math.max(0, g), Math.max(0, b));
                    dimmedBuffer.setLED(i, dimmerColor);
                }
            }

        }
        while (ratio > 1);


        if (dimmedBuffer != null) {
            System.out.println("Dimming lights to reduce current draw");
            ledStrip.setData(dimmedBuffer);
        }
        else {
            ledStrip.setData(buffer);
        }
    }
}

