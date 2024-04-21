package frc.robot.joysticks;

public class CrescendoJoystickFactory {
    public static CrescendoJoystick instance;

    public static CrescendoJoystick getInstance(String joystickType, int port) throws IllegalJoystickTypeException {
        System.out.println("Initializing " + joystickType + " as Joystick");
        switch (joystickType.toLowerCase()) {
            case "xboxcontroller" -> instance = new CrescendoXboxController(port);
            case "logitechcontroller" -> instance = new CrescendoLogitechController(port);
            case "dummy" -> instance = new CrescendoDummyController(port);
            default -> throw new IllegalJoystickTypeException("Illegal Joystick Type: " + joystickType + " ---!");
        }
        return instance;
    }
}
