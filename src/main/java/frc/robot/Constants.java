package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class Constants {
    public class DEVICE_IDs {
        public class DRIVEBASE{
            public static final int FRONT_LEFT_DRIVE_ID = 1;
            public static final int FRONT_LEFT_TURN_ID = 2;
            public static final int FRONT_LEFT_CANCODER_ID = 3;
        
            public static final int FRONT_RIGHT_DRIVE_ID = 4;
            public static final int FRONT_RIGHT_TURN_ID = 5;
            public static final int FRONT_RIGHT_CANCODER_ID = 6;
        
            public static final int REAR_LEFT_DRIVE_ID = 7;
            public static final int REAR_LEFT_TURN_ID = 8;
            public static final int REAR_LEFT_CANCODER_ID = 9;
        
            public static final int REAR_RIGHT_DRIVE_ID = 10;
            public static final int REAR_RIGHT_TURN_ID = 11;
            public static final int REAR_RIGHT_CANCODER_ID = 12;

            public static Port GYRO_PORT = SPI.Port.kMXP;
        }

        public class LED{
            public static final int port = 0;
        }
        
        public static final int GAMEPAD_DRIVER = 0;
        public static final int GAMEPAD_OPERATOR = 1;
    }

    public class KEY_BINDS{
        
    }

    public class TUNED_CONSTANTS {
        public class DRIVEBASE{
            public static final double DRIVE_PIDF0_P = 0;
            public static final double DRIVE_PIDF0_I = 0;
            public static final double DRIVE_PIDF0_D = 0;

            public static final double DRIVE_FEED_FORWARD_KV = 0; //速度
            public static final double DRIVE_FEED_FORWARD_KS = 0; //摩擦

            public static final double TURN_PIDF0_P = 0.31;
            public static final double TURN_PIDF0_I = 0;
            public static final double TURN_PIDF0_D = 0;
            public static final double TURN_PIDF0_F = 0;
        }
    }
    
    public class PHYSICAL_CONSTANTS {
        public static final double NOMINAL_VOLTAGE = 12;
        
        public class DRIVEBASE {
            public class LENGTHS{
                public static double TRACK_WIDTH_INCH = 21.75;
                public static double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCH);
                public static double TRACK_RADIUS_METERS = Math.sqrt((TRACK_WIDTH_METERS * TRACK_WIDTH_METERS) + (TRACK_WIDTH_METERS * TRACK_WIDTH_METERS));

                public static double DRIVE_WHEEL_DIAMETER_INCHES = 4;
                public static double DRIVE_WHEEL_DIAMETER_METERS = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER_INCHES);
                public static double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER_METERS * Math.PI;
            }

            public class GEARS{
                public static final double DRIVE_GEAR_RATIO = 6.12;
                public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
            }      

            public class LIMITING{
                public static final int DRIVE_CURRENT_LIMIT = 40;
                public static final int TURN_CURRENT_LIMIT = 30;

                public static final double DRIVE_LOOP_RAMP_RATE = 0.25;
            }

            public class CANCODER{
                public static final double FRONT_LEFT_OFFSET = -0.3;
                public static final double FRONT_RIGHT_OFFSET = 0.468;
                public static final double REAR_LEFT_OFFSET = 0.307;
                public static final double REAR_RIGHT_OFFSET = -0.210;
            }

            public static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d( (LENGTHS.TRACK_WIDTH_METERS / 2.0),  (LENGTHS.TRACK_WIDTH_METERS / 2.0)  ),  // Front Left   Translation2D (1,1)   -> (  1 , -1 )
                new Translation2d( (LENGTHS.TRACK_WIDTH_METERS / 2.0),  -(LENGTHS.TRACK_WIDTH_METERS / 2.0) ),  // Front Right  Translation2D (1,-1)  -> (  1 ,  1 )
                new Translation2d( -(LENGTHS.TRACK_WIDTH_METERS / 2.0), (LENGTHS.TRACK_WIDTH_METERS / 2.0)  ),  // Rear  Left   Translation2D (-1,1)  -> ( -1 , -1 )
                new Translation2d( -(LENGTHS.TRACK_WIDTH_METERS / 2.0), -(LENGTHS.TRACK_WIDTH_METERS / 2.0) )   // Rear  Right  Translation2D (-1,-1) -> ( -1 ,  1 )
            ); 

            public static final double MAX_ANGULAR_SPEED_RAD = Math.PI * 2; // 360deg/s
            public static final double MAX_WHEEL_SPEED_METERS = 4.87;
        }

        public static final double GYRO_REVERSE_FACTOR = -1;
    }

    // PathPlanner
    public static final double LOOP_TIME_S = 0.02;
    public static final int LOOP_TIME_MS = (int) (LOOP_TIME_S * 1000);
}