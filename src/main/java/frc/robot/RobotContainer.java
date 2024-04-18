// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.ControllerConstants.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerID);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final CannonSubsystem m_CannonSubsystem = new CannonSubsystem();
  private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveTeleopCommand(() -> m_driverController.getLeftY(), () -> m_driverController.getLeftX(), () -> m_driverController.getRightX()));

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().and(m_driverController.leftBumper()).onTrue(m_CannonSubsystem.loadLeftTankTimedCommand(0.2)); //Load left barrel low
    m_driverController.b().and(m_driverController.leftBumper()).onTrue(m_CannonSubsystem.loadLeftTankTimedCommand(0.6)); //Load left barrel mid
    m_driverController.y().and(m_driverController.leftBumper()).onTrue(m_CannonSubsystem.loadLeftTankTimedCommand(1.4)); //Load left barrel high

    m_driverController.a().and(m_driverController.rightBumper()).onTrue(m_CannonSubsystem.loadRightTankTimedCommand(0.2)); //Load right barrel low
    m_driverController.b().and(m_driverController.rightBumper()).onTrue(m_CannonSubsystem.loadRightTankTimedCommand(0.6)); //Load right barrel mid
    m_driverController.y().and(m_driverController.rightBumper()).onTrue(m_CannonSubsystem.loadRightTankTimedCommand(1.4)); //Load right barrel high

    m_driverController.leftTrigger().onTrue(m_CannonSubsystem.shootLeftCannonCommand());
    m_driverController.rightTrigger().onTrue(m_CannonSubsystem.shootRightCannonCommand());

    m_driverController.povUp().whileTrue(m_PivotSubsystem.pivotCommand(0.5));
    m_driverController.povDown().whileTrue(m_PivotSubsystem.pivotCommand(-0.5));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
