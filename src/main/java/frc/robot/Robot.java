// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Constants.VisionConstants.*;

//do not touch unless you know what you're doing
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Thread visionThread; 

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData(CommandScheduler.getInstance());
    DriverStation.silenceJoystickConnectionWarning(true);

    visionThread =
    new Thread(
        () -> {
          UsbCamera camera = CameraServer.startAutomaticCapture();
          camera.setResolution(kResolutionWidth, kResolutionHeight);
          /*
          CvSink cvSink = CameraServer.getVideo();
          CvSource outputStream = CameraServer.putVideo("Rectangle", kResolutionWidth, kResolutionHeight);

          Mat mat = new Mat();

          Point upperLeftCorner = new Point(kResolutionWidth - (kResolutionWidth * (kCrosshairWidth/2 + 0.5)), kResolutionHeight - (kResolutionHeight * (kCrosshairHeight/2 + 0.5)));
          Point lowerRightCorner = new Point(kResolutionWidth * (kCrosshairWidth/2 + 0.5), kResolutionHeight * (kCrosshairHeight/2 + 0.5));

          //Thread.interrupted() allows for the thread to stop when code redeploys/resets
          while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
              outputStream.notifyError(cvSink.getError());
              continue;
            }
            Imgproc.rectangle(mat, upperLeftCorner, lowerRightCorner, new Scalar(0, 0, 0), 7);
            Imgproc.rectangle(mat, upperLeftCorner, lowerRightCorner, new Scalar(70, 255, 70), 4);
            outputStream.putFrame(mat);
          } */
        });
    visionThread.setDaemon(true);
    visionThread.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
