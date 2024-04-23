import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class MecanumWheel {

    final int controllerID;
    final CANSparkMax motorController;
    final SparkPIDController PIDController;
    final RelativeEncoder encoder;
    final String name;

    public static enum ControlType {
        kOpenLoop,
        kClosedLoop
    }

    public MecanumWheel(int controllerID, boolean inverted, double kP, double kD, String name) {
        this.controllerID = controllerID;
        this.name = name;

        motorController = new CANSparkMax(controllerID, MotorType.kBrushless);
        encoder = motorController.getEncoder();
        PIDController = motorController.getPIDController();

        //Configure SparkMAX
        motorController.restoreFactoryDefaults();
        motorController.setIdleMode(IdleMode.kCoast);
        motorController.setInverted(inverted);

        PIDController.setP(kP);
        PIDController.setD(kD);
    }

    public void stopWheel() {
        motorController.stopMotor();
    }

    /**
     * Sets the velocity of the NEO motor of this mecanum wheel.
     * @param speed speed of the wheel (normalized, so 1 is full speed forwards and -1 is full speed backwards)
     * @param controlType what strategy should be used to reach said speed.
     */
    public void setVelocity(double speed, ControlType controlType) {
        switch (controlType) {
            case kClosedLoop:
                PIDController.setReference(MathUtil.clamp(speed, -1, 1) * 12, CANSparkBase.ControlType.kVoltage);
            break;
            case kOpenLoop:
            default:
                motorController.set(speed);
            break;
        }
        SmartDashboard.putNumber(name + " Velocity (RPM)", encoder.getVelocity());
        SmartDashboard.putNumber(name + " Velocity (Porportion of Max RPM)", encoder.getVelocity() / kNEOMaxRPM);
    }
    
}
