// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Toggles;
import frc.robot.Constants.Drive;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.JoyStickDrive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainFactory;
import frc.robot.subsystems.drive.IllegalDriveTypeException;
import frc.utils.joysticks.StormXboxController;


public class RobotContainer {

  NavX navX;

  DrivetrainBase drivetrainBase;

  StormXboxController xboxController;

  public RobotContainer() throws IllegalDriveTypeException {
    if (Toggles.useDrive) {

      drivetrainBase = DrivetrainFactory.getInstance(Drive.driveType);

      if (Toggles.useController){

        xboxController = new StormXboxController(0);
        JoyStickDrive driveWithJoystick = new JoyStickDrive(
                drivetrainBase,
                xboxController::getWpiXSpeed,
                xboxController::getWpiYSpeed,
                xboxController::getOmegaSpeed,
                () -> xboxController.getLeftTrigger() > 0.2,
                () -> xboxController.getRightTrigger() > 0.2
        );
        drivetrainBase.setDefaultCommand(driveWithJoystick);
      }
    }

    if (Toggles.useNavX) {
      navX = new NavX();
    }






    configureBindings();
  }

  private void configureBindings() {

  }


//  public Command getAutonomousCommand() {
//
//  }
}
