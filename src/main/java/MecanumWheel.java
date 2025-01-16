import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

public class MecanumWheel {

    final int controllerID;
    final SparkMax motorController;
    final RelativeEncoder encoder;

    double kP;
    double kD;
    String name;


    public MecanumWheel(int controllerID, boolean inverted, double kP, double kD, String name) {
        this.kP = kP;
        this.kD = kD;
        this.controllerID = controllerID;
        this.name = name;

        motorController = new SparkMax(controllerID, MotorType.kBrushless);
        encoder = motorController.getEncoder();

        //todo fix these lines for 2025, this class is currently unused so its fine
        //Configure SparkMAX
        //motorController.setIdleMode(IdleMode.kCoast);
        //motorController.setInverted(inverted);

        //Configure encoder
        //encoder.setVelocityConversionFactor(1/kNEOMaxRPM);
    }

    public void stopWheel() {
        motorController.stopMotor();
    }

    public void setSpeedDutyCycle(double speed) {
        motorController.set(speed);
    }
    
}
