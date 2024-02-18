package frc.utils.lights;

public enum LightType {
  RGB,
  RGBW;

  public static LightType getType(String typeName) {
      return switch (typeName.toLowerCase()) {
          case "rgb" -> RGB;
          case "rgbw" -> RGBW;
          default -> RGB;
      };
  }

}

