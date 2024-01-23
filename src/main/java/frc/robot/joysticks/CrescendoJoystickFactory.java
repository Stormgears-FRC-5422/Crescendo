package frc.robot.joysticks;

public class CrescendoJoystickFactory {
    public static CrescendoJoystick instance;

    public static CrescendoJoystick getInstance(String joystickType, int port) throws IllegalJoystickTypeException {
        if (instance == null) {
            System.out.printf("Initializing %s", joystickType);
            switch (joystickType) {
                case "XboxController" -> instance = new CrescendoXboxController(port);
                case "LogitechController" -> instance = new CrescendoLogitechController(port);
                default -> throw new IllegalJoystickTypeException("Illegal Joystick Type: " + joystickType + " ---!");
            }
        }
        return instance;
    }
}
