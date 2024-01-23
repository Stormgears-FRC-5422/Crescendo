package frc.robot.joysticks;

public class CrescendoJoystickFactory {
    public static CrescendoJoystick instance;

    public static CrescendoJoystick getInstance(String joystickType) throws IllegalJoystickTypeException {
        if (instance == null) {
            System.out.printf("Initializing %s", joystickType);
            switch (joystickType) {
                case "XboxController" -> instance = new CrescendoXboxController(0);
                case "LogitechController" -> instance = new CrescendoLogitechController(0);
                default -> throw new IllegalJoystickTypeException("Illegal Joystick Type: " + joystickType + " ---!");
            }
        }
        return instance;
    }
}
