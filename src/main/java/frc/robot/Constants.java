package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
//  CONSTANTS FOR COMPETITION ROBOT

	public static final class GlobalConstants {
		// Robot loop time
		public static final double LOOPER_TIME = 0.02;

        public static final int PCM_ID = 14;
	}

    public static final class AutoConstants {
        // Holonomic controller PID constants - x
        public static final double AUTO_X_VELOCITY_KP = 1.0;
        public static final double AUTO_X_VELOCITY_KI = 0.0;
        public static final double AUTO_X_VELOCITY_KD = 0.0;

        // Holonomic controller PID constants - y
        public static final double AUTO_Y_VELOCITY_KP = 1.0;
        public static final double AUTO_Y_VELOCITY_KI = 0.0;
        public static final double AUTO_Y_VELOCITY_KD = 0.0;

        // Holonomic controller PID constants - angular velocity
        public static final double AUTO_ROTATION_KP = 0.0;
        public static final double AUTO_ROTATION_KI = 0.0;
        public static final double AUTO_ROTATION_KD = 0.0;

        // Max translational velocity, acceleration in m/s
        public static final double AUTO_MAX_VELOCITY = Units.feetToMeters(2.5);
        public static final double AUTO_MAX_ACCELERATION = Units.feetToMeters(1.0);
    }

	public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER_INCHES = 4.0;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);

        public static final double DRIVE_GEAR_RATIO = 1.0 / 6.75;
        public static final double STEER_GEAR_RATIO = 1.0 / 12.8;

        public static final double DRIVE_ENCODER_METERS_PER_ROTATION = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double STEER_ENCODER_RADIANS_PER_ROTATION = STEER_GEAR_RATIO * 2.0 * Math.PI;
        public static final double DRIVE_ENCODER_METERS_PER_SECOND = DRIVE_ENCODER_METERS_PER_ROTATION / 60.0;
        public static final double STEER_ENCODER_RADIANS_PER_SECOND = STEER_ENCODER_RADIANS_PER_ROTATION / 60.0;
        
        public static final double STEER_KP = 0.65;
        public static final double STEER_KI = 0.0;
        public static final double STEER_KD = 0.01;
    }

    public static final class DriveConstants {
		// Left-to-right distance between wheel centers in meters
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.5);
		// Front-to-back distance between wheel centers in meters
        public static final double WHEEL_BASE = Units.inchesToMeters(21.5);
        
		// Creates kinematics object for swerve subsystem
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );

        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 4;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 6;

        public static final int FRONT_LEFT_STEER_MOTOR_ID = 1;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 3;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 5;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 7;

        public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = true;

        public static final boolean FRONT_LEFT_STEER_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_STEER_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_STEER_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_STEER_ENCODER_REVERSED = false;

        public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 10;
        public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 11;
        public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 12;
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 13;

        public static final boolean FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;

		// Offsets for swerve modules. Should be equal to absolute encoder reading when wheel is facing straight forward
        public static final double FRONT_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 3.5450;
        public static final double FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 1.6874;
        public static final double BACK_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 6.0700;
        public static final double BACK_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 0.6259;

        public static final double DRIVE_MAX_PHYSICAL_VELOCITY = Units.feetToMeters(14.6);

        public static final double DRIVE_MAX_VELOCITY = Units.feetToMeters(14.0);
        public static final double DRIVE_MAX_ROTATIONAL_VELOCITY = 1.0 * 2.0 * Math.PI; // rad/s

        public static final double DRIVE_MAX_VELOCITY_SLOW = Units.feetToMeters(3.0);
        public static final double DRIVE_MAX_ROTATIONAL_VELOCITY_SLOW = 0.25 * 2.0 * Math.PI; // rad/s

        public static final double DRIVE_ROTATION_KP = 11.0;
        public static final double DRIVE_ROTATION_KI = 0.2;
        public static final double DRIVE_ROTATION_KD = 2500.0;

        public static final double DRIVE_MAX_ACCELERATION = 10.0; // m/s^2
        public static final double DRIVE_MAX_ROTATIONAL_ACCELERATION = 25.0; // rad/s^2

        public static final double CHARGE_STATION_PITCH_DEADBAND = 1.5; // degrees
    }

    public static final class ExtensionConstants {
        public static final int LOWER_EXTENSION_PCM_PORT_FORWARD = 1;
        public static final int LOWER_EXTENSION_PCM_PORT_REVERSE = 2;
        public static final int UPPER_EXTENSION_PCM_PORT_FORWARD = 3;
        public static final int UPPER_EXTENSION_PCM_PORT_REVERSE = 4;

        public static final int ELBOW_MOTOR_ID = 16;

        public static final double ELBOW_MOTOR_TO_LOWER_SHAFT_RATIO = 44.0;
        public static final double ELBOW_CHAIN_GEAR_RATIO = 18.0 / 48.0;

        public static final double ELBOW_MAX_VELOCITY = 0.5;
        public static final double ELBOW_MAX_ACCELERATION = 1.0;

        public static final int ELBOW_ABSOLUTE_ENCODER_PORT = 0;
        public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = 0.8514;

        public static final int ELBOW_ENCODER_PORT_A = 1;
        public static final int ELBOW_ENCODER_PORT_B = 2;

        public static final int STRING_POTENTIOMETER_PORT = 1;

        public static final double ELBOW_KP = 5.8;
        public static final double ELBOW_KI = 0.0;
        public static final double ELBOW_KD = 0.5;

        public static final double ELBOW_KS = 0.0;
        public static final double ELBOW_KG = 0.11;
        public static final double ELBOW_KV = 0.0;
        public static final double ELBOW_KA = 0.0;
    }

    public static final class GripperConstants{
        public static final int GRIPPER_PCM_PORT = 0;
        public static final int DISTANCE_SENSOR_PORT = 0;
        public static final int LIMIT_SWITCH_PORT = 3;

        public static final double DISTANCE_VOLTAGE_THRESHOLD_CUBE = 2.3;
        public static final double DISTANCE_VOLTAGE_THRESHOLD_CONE = 2.3;
    }

	public static final class OIConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;

        public static final int OPERATOR_CONTROLLER_PORT = 1;

		// Controller axis IDs        
        public static final int DRIVER_X_AXIS = 0;

        public static final int DRIVER_Y_AXIS = 1;

        public static final int DRIVER_ROT_AXIS = 4;
        
		// Deadband for drive axes
        public static final double DRIVE_DEADBAND = 0.01;

		// Threshold for trigger being pressed
		public static final double TRIGGER_THRESHOLD = 0.2;
	}

    public static final class LEDConstants {
        public static final int LED_PWM_PORT = 0;
    }

    public static final class ActionConstants {
        public static final boolean WILL_CANCEL = true;

        public static final boolean WILL_NOT_CANCEL = false;
    }

     public static final class LimelightConstants {
         public static final String[] LIMELIGHT_NAMES = new String[] {
             "limelight-front",
             "limelight-back"
         };
     }

//  CONSTANTS FOR PRACTICE ROBOT

	// public static final class GlobalConstants {
	// 	// Robot loop time
	// 	public static final double LOOPER_TIME = 0.02;

    //     public static final int PCM_ID = 14;
	// }

    // public static final class AutoConstants {
    //     // Holonomic controller PID constants - x
    //     public static final double AUTO_X_VELOCITY_KP = 0.05;
    //     public static final double AUTO_X_VELOCITY_KI = 0.0;
    //     public static final double AUTO_X_VELOCITY_KD = 0.1;

    //     // Holonomic controller PID constants - y
    //     public static final double AUTO_Y_VELOCITY_KP = 0.05;
    //     public static final double AUTO_Y_VELOCITY_KI = 0.0;
    //     public static final double AUTO_Y_VELOCITY_KD = 0.1;

    //     // Holonomic controller PID constants - theta velocity (omega)
    //     public static final double AUTO_OMEGA_KP = 8.0;
    //     public static final double AUTO_OMEGA_KI = 0.0;
    //     public static final double AUTO_OMEGA_KD = 0.2;

    //     // Max translational velocity, acceleration in m/s
    //     public static final double AUTO_MAX_VELOCITY = Units.feetToMeters(2.5);
    //     public static final double AUTO_MAX_ACCELERATION = Units.feetToMeters(1.0);

    //     // Max rotational velocity, acceleration in rad/s
    //     public static final TrapezoidProfile.Constraints AUTO_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(6.28, 3.14);
    // }

	// public static final class ModuleConstants {
    //     public static final double WHEEL_DIAMETER_INCHES = 4.0;
    //     public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);

    //     public static final double DRIVE_GEAR_RATIO = 1.0 / 6.75;
    //     public static final double STEER_GEAR_RATIO = 1.0 / 12.8;

    //     public static final double DRIVE_ENCODER_METERS_PER_ROTATION = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    //     public static final double STEER_ENCODER_RADIANS_PER_ROTATION = STEER_GEAR_RATIO * 2.0 * Math.PI;
    //     public static final double DRIVE_ENCODER_METERS_PER_SECOND = DRIVE_ENCODER_METERS_PER_ROTATION / 60.0;
    //     public static final double STEER_ENCODER_RADIANS_PER_SECOND = STEER_ENCODER_RADIANS_PER_ROTATION / 60.0;
        
    //     public static final double STEER_KP = 0.7;
    //     public static final double STEER_KD = 0.25;
    // }

    // public static final class DriveConstants {
	// 	// Left-to-right distance between wheel centers in meters
    //     public static final double TRACK_WIDTH = Units.inchesToMeters(19.5);
	// 	// Front-to-back distance between wheel centers in meters
    //     public static final double WHEEL_BASE = Units.inchesToMeters(21.5);
        
	// 	// Creates kinematics object for swerve subsystem
    //     public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
    //             new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    //             new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    //             new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    //             new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    //     );

    //     public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 8;
    //     public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    //     public static final int BACK_LEFT_DRIVE_MOTOR_ID = 4;
    //     public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 6;

    //     public static final int FRONT_LEFT_STEER_MOTOR_ID = 1;
    //     public static final int FRONT_RIGHT_STEER_MOTOR_ID = 3;
    //     public static final int BACK_LEFT_STEER_MOTOR_ID = 5;
    //     public static final int BACK_RIGHT_STEER_MOTOR_ID = 7;

    //     public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    //     public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true;
    //     public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
    //     public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;

    //     public static final boolean FRONT_LEFT_STEER_ENCODER_REVERSED = false;
    //     public static final boolean FRONT_RIGHT_STEER_ENCODER_REVERSED = false;
    //     public static final boolean BACK_LEFT_STEER_ENCODER_REVERSED = false;
    //     public static final boolean BACK_RIGHT_STEER_ENCODER_REVERSED = false;

    //     public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 10;
    //     public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 11;
    //     public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 12;
    //     public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 13;

    //     public static final boolean FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
    //     public static final boolean FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
    //     public static final boolean BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
    //     public static final boolean BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;

	// 	// Offsets for swerve modules. Should be equal to absolute encoder reading when wheel is facing straight forward
    //     public static final double FRONT_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 1.4404;
    //     public static final double FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 1.5585;
    //     public static final double BACK_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 3.6202;
    //     public static final double BACK_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 2.1844;

    //     public static final double DRIVE_MAX_PHYSICAL_VELOCITY = Units.feetToMeters(14.6);

    //     public static final double DRIVE_MAX_VELOCITY = Units.feetToMeters(12.0);
    //     public static final double DRIVE_MAX_ROTATIONAL_VELOCITY = 1.0 * 2.0 * Math.PI; // rad/s

    //     public static final double DRIVE_MAX_VELOCITY_SLOW = Units.feetToMeters(3.0);
    //     public static final double DRIVE_MAX_ROTATIONAL_VELOCITY_SLOW = 0.4  * 2.0 * Math.PI; // rad/s

    //     public static final double DRIVE_ROTATION_KP = 5.0;
    //     public static final double DRIVE_ROTATION_KI = 0.0;
    //     public static final double DRIVE_ROTATION_KD = 0.0;

    //     public static final double DRIVE_MAX_ACCELERATION = 10.0; // m/s^2
    //     public static final double DRIVE_MAX_ROTATIONAL_ACCELERATION = 7.0; // rad/s^2

    //     public static final double CHARGE_STATION_PITCH_DEADBAND = 0.75; // degrees
    // }

    // public static final class ExtensionConstants {
    //     public static final int LOWER_EXTENSION_PCM_PORT_FORWARD = 1;
    //     public static final int LOWER_EXTENSION_PCM_PORT_REVERSE = 2;
    //     public static final int UPPER_EXTENSION_PCM_PORT_FORWARD = 3;
    //     public static final int UPPER_EXTENSION_PCM_PORT_REVERSE = 4;

    //     public static final int ELBOW_MOTOR_ID = 16;

    //     public static final double ELBOW_MOTOR_TO_LOWER_SHAFT_RATIO = 44.0;
    //     public static final double ELBOW_CHAIN_GEAR_RATIO = 18.0 / 48.0;

    //     public static final double ELBOW_MAX_VELOCITY = 0.5;
    //     public static final double ELBOW_MAX_ACCELERATION = 1.0;

    //     public static final int ELBOW_ABSOLUTE_ENCODER_PORT = 0;
    //     public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = 0.318;

    //     public static final int ELBOW_ENCODER_PORT_A = 1;
    //     public static final int ELBOW_ENCODER_PORT_B = 2;

    //     public static final int STRING_POTENTIOMETER_PORT = 1;

    //     public static final double ELBOW_KP = 5.8;
    //     public static final double ELBOW_KI = 0.0;
    //     public static final double ELBOW_KD = 0.5;

    //     public static final double ELBOW_KS = 0.0;
    //     public static final double ELBOW_KG = 0.11;
    //     public static final double ELBOW_KV = 0.0;
    //     public static final double ELBOW_KA = 0.0;
    // }

    // public static final class GripperConstants{
    //     public static final int GRIPPER_PCM_PORT = 0;
    //     public static final int DISTANCE_SENSOR_PORT = 0;
    //     public static final int LIMIT_SWITCH_PORT = 3;

    //     public static final double DISTANCE_VOLTAGE_THRESHOLD_CUBE = 2.3;
    //     public static final double DISTANCE_VOLTAGE_THRESHOLD_CONE = 2.3;
    // }

	// public static final class OIConstants {
	// 	public static final int DRIVER_CONTROLLER_PORT = 0;

    //     public static final int OPERATOR_CONTROLLER_PORT = 1;

	// 	// Controller axis IDs        
    //     public static final int DRIVER_X_AXIS = 0;

    //     public static final int DRIVER_Y_AXIS = 1;

    //     public static final int DRIVER_ROT_AXIS = 4;
        
	// 	// Deadband for drive axes
    //     public static final double DRIVE_DEADBAND = 0.025;

	// 	// Threshold for trigger being pressed
	// 	public static final double TRIGGER_THRESHOLD = 0.2;
	// }

    // public static final class LEDConstants {
    //     public static final int LED_PWM_PORT = 0;
    // }

    // public static final class ActionConstants {
    //     public static final boolean WILL_CANCEL = true;

    //     public static final boolean WILL_NOT_CANCEL = false;
    // }

    // public static final class LimelightConstants {
    //     public static final String[] LIMELIGHT_NAMES = new String[] {
    //     };
    // }
}