package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;

public class Climber implements Subsystem {
    private final CANSparkMax climbMotor = new CANSparkMax(ClimbConstants.CLIMB_MOTOR_PORT, MotorType.kBrushless);

    private final Servo dropServo = new Servo(ClimbConstants.DROP_SERVO_PORT);

    public Climber() {
        dropServo.set(0.0);
    }

    public void releaseClimber() {
        dropServo.set(0.5);
    }

    public void raiseClimber() {
        climbMotor.set(0.5);
    }

    public void lowerClimber() {
        climbMotor.set(-0.5);
    }

    @Override
    public void periodic() {}
}
