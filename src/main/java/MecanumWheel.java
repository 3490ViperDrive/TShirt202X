import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class MecanumWheel {

    final int controllerID;
    final CANSparkMax motorController;
    final RelativeEncoder encoder;

    double kP;
    double kD;
    String name;


    public MecanumWheel(int controllerID, boolean inverted, double kP, double kD, String name) {
        this.kP = kP;
        this.kD = kD;
        this.controllerID = controllerID;
        this.name = name;

        motorController = new CANSparkMax(controllerID, MotorType.kBrushless);
        encoder = motorController.getEncoder();

        //Configure SparkMAX
        motorController.setIdleMode(IdleMode.kCoast);
        motorController.setInverted(inverted);

        //Configure encoder
        encoder.setVelocityConversionFactor(1/kNEOMaxRPM);
    }

    public void stopWheel() {
        motorController.stopMotor();
    }

    public void setSpeedDutyCycle(double speed) {
        motorController.set(speed);
    }
    
}
