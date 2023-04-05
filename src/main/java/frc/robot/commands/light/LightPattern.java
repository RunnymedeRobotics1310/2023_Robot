package frc.robot.commands.light;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class LightPattern {
  private final List<Color> lights = new ArrayList<>();

  public LightPattern(Color... color) {
    lights.addAll(Arrays.asList(color));
  }

  public LightPattern append(Color color) {
    lights.add(color);
    return this;
  }

  public List<Color> getLights() {
    return Collections.unmodifiableList(lights);
  }

}
