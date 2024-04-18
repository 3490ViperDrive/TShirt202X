package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CannonConstants.*;

public class CannonSubsystem extends SubsystemBase {
    final TalonSRX leftShooterSolenoidSRX;
    final TalonSRX rightShooterSolenoidSRX;
    final TalonSRX leftPrimerSolenoidSRX;
    final TalonSRX rightPrimerSolenoidSRX;

    CannonState leftCannonState;
    CannonState rightCannonState;

    public enum CannonState {
        kUnknown,
        kPrimed,
        kEmpty;
    }

    public CannonSubsystem() {
        leftShooterSolenoidSRX = new TalonSRX(kLeftShooterSolenoidID);
        rightShooterSolenoidSRX = new TalonSRX(kRightShooterSolenoidID);
        leftPrimerSolenoidSRX = new TalonSRX(kLeftPrimerSolenoidID);
        rightPrimerSolenoidSRX = new TalonSRX(kRightPrimerSolenoidID);
        configureSolenoidSRX(leftShooterSolenoidSRX, kShooterContinuousCurrentLimit, kShooterPeakCurrentLimit);
        configureSolenoidSRX(rightShooterSolenoidSRX, kShooterContinuousCurrentLimit, kShooterPeakCurrentLimit);
        configureSolenoidSRX(leftPrimerSolenoidSRX, kPrimerContinuousCurrentLimit, kPrimerPeakCurrentLimit);
        configureSolenoidSRX(rightPrimerSolenoidSRX, kPrimerContinuousCurrentLimit, kPrimerPeakCurrentLimit);

        leftCannonState = CannonState.kUnknown;
        rightCannonState = CannonState.kUnknown;

        SmartDashboard.putNumber("left cannon fill", -1);
        SmartDashboard.putNumber("right cannon fill", -1);
    }

    private void configureSolenoidSRX(TalonSRX motorController, int continuousCurrentLimit, int peakCurrentLimit) {
        motorController.configContinuousCurrentLimit(continuousCurrentLimit);
        motorController.configPeakCurrentLimit(peakCurrentLimit);
        motorController.enableCurrentLimit(true);
    }

    @Override
    public void periodic() {
        
    }

    private Command activateSolenoidTimedCommand(TalonSRX motorController, double duration) {
        return new StartEndCommand(
            () -> motorController.set(TalonSRXControlMode.PercentOutput, 1), () -> motorController.set(TalonSRXControlMode.PercentOutput, 0), this //TODO should this command require the subsystem
        ).withTimeout(duration);
    }

    public Command loadLeftTankTimedCommand(double duration) {
        return activateSolenoidTimedCommand(leftPrimerSolenoidSRX, duration)
                .unless(() -> {return leftCannonState == CannonState.kPrimed;})
                .finallyDo((interrupted) -> {
                    if (leftCannonState != CannonState.kPrimed) {
                        SmartDashboard.putNumber("left cannon fill", duration);
                    }
                    leftCannonState = CannonState.kPrimed;
                })
                .withName("Load Left Tank");
    }

    public Command loadRightTankTimedCommand(double duration) {
        return activateSolenoidTimedCommand(rightPrimerSolenoidSRX, duration)
                .unless(() -> {return rightCannonState == CannonState.kPrimed;})
                .finallyDo((interrupted) -> {
                    if (rightCannonState != CannonState.kPrimed) {
                        SmartDashboard.putNumber("right cannon fill", duration);
                    }
                    rightCannonState = CannonState.kPrimed;
                })
                .withName("Load Right Tank");
    }

    public Command shootLeftCannonCommand() {
        return activateSolenoidTimedCommand(leftShooterSolenoidSRX, 0.5)
                .finallyDo((interrupted) -> {
                    leftCannonState = CannonState.kEmpty;
                    SmartDashboard.putNumber("left cannon fill", 0);
                })
                .withName("Shoot Left Cannon");
    }

    public Command shootRightCannonCommand() {
        return activateSolenoidTimedCommand(rightShooterSolenoidSRX, 0.5)
                .finallyDo((interrupted) -> {
                    rightCannonState = CannonState.kEmpty;
                    SmartDashboard.putNumber("right cannon fill", 0);
                })
                .withName("Shoot Right Cannon");
    }
}
