// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.teleop.manualDrive;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  private Controller controller = Controller.getInstance();
  private CommandXboxController driverCommandController = new CommandXboxController(0);
  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
      new manualDrive(
        () -> -controller.getDriverLY(), // x-axis up-positive
        () -> -controller.getDriverLX(), // y-axis left-positive 
        () -> -controller.getDriverRX(), // rotation CCW
        () -> controller.getDriverLT(),
        () -> controller.getDriverRT()
      )
    );

    configureBindings();
  }

  private void configureBindings() {
    driverCommandController.button(8).debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          swerveSubsystem.resetYaw();
        }
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
