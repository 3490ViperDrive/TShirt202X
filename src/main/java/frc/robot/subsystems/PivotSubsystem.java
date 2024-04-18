package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.PivotConstants.*;

public class PivotSubsystem extends SubsystemBase {
    final TalonSRX motorController;
    final DigitalInput lowerLimitSwitch;
    
    public PivotSubsystem() {
        lowerLimitSwitch = new DigitalInput(kLowerLimitSwitchID);
        motorController = new TalonSRX(kMotorControllerID);
        motorController.configPeakCurrentLimit(kPeakCurrentLimit);
        motorController.configContinuousCurrentLimit(kContinuousCurrentLimit);
        motorController.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("lower limit switch", lowerLimitSwitch.get());
    }

    //[0, 1]
    public Command pivotCommand(double speed) {
        return new StartEndCommand(() -> runPivot(speed),  () -> motorController.set(ControlMode.PercentOutput, 0), this)
        .withName("Pivot Move");
    }

    private double mapToPercentOutput(double value) {
        return Math.max(0, Math.min(1, Math.abs(value))) * Math.signum(value);
    }

    private void runPivot(double speed) {
        if (speed < 0 && !lowerLimitSwitch.get()) {
            motorController.set(ControlMode.PercentOutput, 0);
        } else {
            motorController.set(ControlMode.PercentOutput, mapToPercentOutput(speed));
        }
    }
}
