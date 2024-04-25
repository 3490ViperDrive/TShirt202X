package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MecanumWheel;
import frc.robot.Constants.ControllerConstants;
import frc.robot.MecanumWheel.ControlType;

import static frc.robot.Constants.DrivetrainConstants.*;

public class DriveSubsystem extends SubsystemBase {
    // final CANSparkMax frontLeftSparkMax;
    // final CANSparkMax frontRightSparkMax;
    // final CANSparkMax backLeftSparkMax;
    // final CANSparkMax backRightSparkMax;

    final MecanumWheel frontLeftWheel;
    final MecanumWheel frontRightWheel;
    final MecanumWheel backLeftWheel;
    final MecanumWheel backRightWheel;

    final MecanumDriveKinematics mecanumKinematics;


    public DriveSubsystem() {
        // frontLeftSparkMax = new CANSparkMax(kFrontLeftMotorControllerID, MotorType.kBrushless);
        // frontRightSparkMax = new CANSparkMax(kFrontRightMotorControllerID, MotorType.kBrushless);
        // backLeftSparkMax = new CANSparkMax(kBackLeftMotorControllerID, MotorType.kBrushless);
        // backRightSparkMax = new CANSparkMax(kBackRightMotorControllerID, MotorType.kBrushless);

        frontLeftWheel = new MecanumWheel(kFrontLeftMotorControllerID, false, "Front Left");
        frontRightWheel = new MecanumWheel(kFrontRightMotorControllerID, true, "Front Left");
        backLeftWheel = new MecanumWheel(kBackLeftMotorControllerID, false, "Front Left");
        backRightWheel = new MecanumWheel(kBackRightMotorControllerID, true, "Front Left");

        mecanumKinematics = new MecanumDriveKinematics(
            new Translation2d(kTrackWidth/2, kTrackWidth/2),
            new Translation2d(kTrackWidth/2, -kTrackWidth/2),
            new Translation2d(-kTrackWidth/2, kTrackWidth/2),
            new Translation2d(-kTrackWidth/2, -kTrackWidth/2));

        // Timer.delay(1); //Delay motor config for a second to give the CANbus some time

        // frontLeftSparkMax.setInverted(false);
        // backLeftSparkMax.setInverted(false);
        // frontRightSparkMax.setInverted(true);
        // backRightSparkMax.setInverted(true);
        
        // configureMotorController(frontLeftSparkMax);
        // configureMotorController(backLeftSparkMax);
        // configureMotorController(frontRightSparkMax);
        // configureMotorController(backRightSparkMax);

        //+x is forwards
        //TODO determine if/why using proper distances causes strafing issues (it may just be the weight of the robot)
        /*
        mecanumKinematics = new MecanumDriveKinematics(
            new Translation2d(kWheelbase/2, kTrackWidth/2),
            new Translation2d(kWheelbase/2, -kTrackWidth/2),
            new Translation2d(-kWheelbase/2, kTrackWidth/2),
            new Translation2d(-kWheelbase/2, -kTrackWidth/2)); */
    }

    void configureMotorController(CANSparkMax motorController) {
        motorController.setIdleMode(IdleMode.kCoast);
        //Config current limits?
        //Config velocity PID?
    }

    @Override
    public void periodic() {

    }

    public Command driveTeleopCommand (DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRot) {
        return runEnd(() -> driveCartesian(filterLeftStick(xSpeed.getAsDouble(), ySpeed.getAsDouble()), filterAxis(zRot.getAsDouble())), () -> stopMotors())
        .withName("Drive Teleop");
    }

    Translation2d filterLeftStick(double xSpeed, double ySpeed) {
        //Convert to polar coordinates and saturate
        Translation2d xy = new Translation2d(xSpeed, ySpeed);
        //Convert to polar coordinates, apply filters, return
        Translation2d xy2 = new Translation2d(filterAxis(Math.min(1, xy.getNorm())), 0).rotateBy(xy.getAngle());
        SmartDashboard.putNumber("Left Stick X Filtered", xy2.getX());
        SmartDashboard.putNumber("Left Stick Y Filtered", xy2.getY());
        return xy2;
    }

    double filterAxis(double value) {
        //Square and deadband input
        return MathUtil.applyDeadband(Math.abs(value) * Math.abs(value) * Math.signum(value), ControllerConstants.kInputDeadband);
    }

    void driveCartesian(Translation2d translation, double zRot) {
        driveCartesian(translation.getX(), translation.getY(), zRot);
    }

    void driveCartesian(double xSpeed, double ySpeed, double zRot) {
        //Make ChassisSpeeds obj
        //Convert to WheelSpeeds
        MecanumDriveWheelSpeeds wheelSpeeds = mecanumKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ySpeed, zRot));
        wheelSpeeds.desaturate(1);
        //Command motor controllers
        // frontLeftSparkMax.set(wheelSpeeds.frontLeftMetersPerSecond);
        // frontRightSparkMax.set(wheelSpeeds.frontRightMetersPerSecond);
        // backLeftSparkMax.set(wheelSpeeds.rearLeftMetersPerSecond);
        // backRightSparkMax.set(wheelSpeeds.rearRightMetersPerSecond);
        frontLeftWheel.setVelocity(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kOpenLoop);
        frontRightWheel.setVelocity(wheelSpeeds.frontRightMetersPerSecond, ControlType.kOpenLoop);
        backLeftWheel.setVelocity(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kOpenLoop);
        backRightWheel.setVelocity(wheelSpeeds.rearRightMetersPerSecond, ControlType.kOpenLoop);

        SmartDashboard.putNumber("front left wheel", wheelSpeeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("front right wheel", wheelSpeeds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("rear left wheel", wheelSpeeds.rearLeftMetersPerSecond);
        SmartDashboard.putNumber("rear right wheel", wheelSpeeds.rearRightMetersPerSecond);
    }

    void stopMotors() {
        // frontLeftSparkMax.set(0);
        // frontRightSparkMax.set(0);
        // backLeftSparkMax.set(0);
        // backRightSparkMax.set(0);
        frontLeftWheel.stopWheel();
        frontRightWheel.stopWheel();
        backLeftWheel.stopWheel();
        backRightWheel.stopWheel();
    }
}
