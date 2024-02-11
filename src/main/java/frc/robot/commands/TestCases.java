package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Drive;
import frc.robot.Constants.ButtonBoard;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TestCases extends Command {
    private DrivetrainBase drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier omegaSupplier;
    private final BooleanSupplier robotRelativeSupplier;
    private final BooleanSupplier turboSupplier;

    private boolean m_flipJoystick = false;

    private double startTime, currentTime, lapse;
    private double[] testDurations = {5.0, 4.0, 3.0, 2.0};
    private double[] cummulativeTestDuration = new double[testDurations.length];
    private double currentCum = 0.0;
    private int currentTestCaseNumber;

    // private ChassisSpeeds speeds;

    public TestCases(DrivetrainBase drivetrain,
                         CrescendoJoystick joystick) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        omegaSupplier = joystick::getOmegaSpeed;
        robotRelativeSupplier = joystick::getRobotRelative;
        turboSupplier = joystick::getTurbo;

    }

    @Override
    public void initialize() {
        System.out.println("Starting TestCases for automated test");
        m_flipJoystick = ButtonBoard.flipJoystickForRed && !RobotState.getInstance().isAllianceBlue();
        System.out.println("Joystick is " + (m_flipJoystick ? "" : "NOT")+ " flipped for alliance");

        if (!turboSupplier.getAsBoolean()) {
            drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
        } else {
            drivetrain.setDriveSpeedScale(Drive.driveSpeedScale);
        }

        startTime = Timer.getFPGATimestamp();
        System.out.print(cummulativeTestDuration);
        getCummulativeTestTime();
        System.out.print(cummulativeTestDuration);

    }
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds speeds;
        speeds = new ChassisSpeeds(0.0, 0.0,
                0.0);
        drivetrain.percentOutputDrive(speeds, true);
        System.out.println("Ending Joystick Drive. Interrupted = " + interrupted);
    }

    @Override
    public void execute() {
        boolean fieldRelative = !robotRelativeSupplier.getAsBoolean();
        double x = txSupplier.getAsDouble();
        double y = tySupplier.getAsDouble();
        double omega = omegaSupplier.getAsDouble();
        ChassisSpeeds speeds;

        // // When on the red alliance, we want to have "forward" mean "move in the -X direction" and so on.
        // // But only for field relative driving. Robot relative driving is always the same
        // if (m_flipJoystick && fieldRelative) {
        //     speeds = new ChassisSpeeds(-x, -y, omega);
        // } else {
        //     speeds = new ChassisSpeeds(x, y, omega);
        // }

        currentTime = Timer.getFPGATimestamp();
        lapse = currentTime - startTime;
        System.out.println("Time lapse: ");
        System.out.println(lapse);
        currentTestCaseNumber = getTestCaseNumber(currentTime);

        System.out.println("Test mode running test case number:" + currentTestCaseNumber);

        switch (currentTestCaseNumber) {
            case 0:
                // ShuffleboardConstants.getInstance().drivetrainTab.add("Drive direction",
                //     robotRelativeSupplier.getAsBoolean()? "Field Orientation": "Robot Orientation");

                // if (!turboSupplier.getAsBoolean()) {
                //     drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
                // } else {
                //     drivetrain.setDriveSpeedScale(Drive.driveSpeedScale);
                // }

                // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(txSupplier.getAsDouble(),
                //     tySupplier.getAsDouble(),
                //     omegaSupplier.getAsDouble());

                // drivetrain.percentOutputDrive(chassisSpeeds, robotRelativeSupplier.getAsBoolean());
                System.out.println("Test case 0");
                speeds = new ChassisSpeeds(0.5, 0.0, 0.0);
                drivetrain.percentOutputDrive(speeds, true);
                break;

            case 1:
                System.out.println("Test case 1");
                speeds = new ChassisSpeeds(0.0, 0.5, 0.0);
                drivetrain.percentOutputDrive(speeds, true);
                break;

            case 2:
                System.out.println("Test case 2");
                speeds = new ChassisSpeeds(0.0, 0.0, 0.5);
                drivetrain.percentOutputDrive(speeds, true);
                break;

            default:
                System.out.println("All test case complete. Stop robot movement.");
                speeds = new ChassisSpeeds(0.0, 0.0,
                        0.0);
                drivetrain.percentOutputDrive(speeds, true);
                break;
        }


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void getCummulativeTestTime(){
        for (int i = 0; i< testDurations.length; i++) {
            cummulativeTestDuration[i] = currentCum + testDurations[i];
            currentCum = cummulativeTestDuration[i];
        }
    }

    private int getTestCaseNumber(double time){
        for (int i = 0; i < cummulativeTestDuration.length - 1; i++){
            if (time >= cummulativeTestDuration[i] && time < cummulativeTestDuration[i+1]) {
                return i;
            }
        }
        return cummulativeTestDuration.length;
    }

}









