package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PivotConstants{
    public static final int PIVOT_MOTOR_PORT = 2; //test
    public static final int PIVOT_LIMIT = 9;
    public static final int JOYSTICK_PORT = 0;

    public static final double subWooferEnc = 20;
    public static final double ampEnc = 40;
    public static final double wingEnc = 55;

  }

  public static class SwerveConstants {
    //SDS L2 
    public static final boolean ROTATION_ENCODER_DIRECTION = false; 

    /* * * MEASUREMENTS * * */
    //FIXME REPLACE WITH VALUES OF ACTUAL BASE 
    public static final double WHEEL_DIAMETER = 4 * 2.5 / 100;
    public static final double TRACK_WIDTH = 0.635;
    public static final double WHEEL_BASE = 0.635;
    
    public static final double DRIVE_GEAR_RATIO = 8.14 / 1;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;
    
    public static final double VOLTAGE = 7.2;

    /* * * SWERVE DRIVE KINEMATICS * * */
    // ORDER IS ALWAYS FL, BL, FR, BR 
    //pos x is out in front, pos y is to the left 
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      
      // front left
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // front right
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      // back right
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
      

      /* //front left 
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), 

      //back left 
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),

      //front right 
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), 

      //back right 
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2) */

    );

    /* * * FRONT LEFT * * */
    //FIXME FILL IN VALUES FOR FRONT LEFT 
    public static class FrontLeft {
      public static final int DRIVE_PORT = 1;
      public static final int ROTATION_PORT = 5;
      public static final int ABSOLUTE_ENCODER_PORT = 9;
      public static final double OFFSET = 80.95;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK LEFT * * */
    //FIXME FILL IN VALUES FOR BACK LEFT 
    public static class BackLeft {
      public static final int DRIVE_PORT = 2;
      public static final int ROTATION_PORT = 6;
      public static final int ABSOLUTE_ENCODER_PORT = 10;
      public static final double OFFSET = -101.60 + 6;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK RIGHT * * */
    //FIXME FILL IN VALUES FOR BACK RIGHT 
    public static class BackRight {
      public static final int DRIVE_PORT = 3;
      public static final int ROTATION_PORT = 7;
      public static final int ABSOLUTE_ENCODER_PORT = 11;
      public static final double OFFSET = -28.92 + 6;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * FRONT RIGHT * * */
    //FIXME FILL IN VALUES FOR FRONT RIGHT 
    public static class FrontRight {
      public static final int DRIVE_PORT = 4;
      public static final int ROTATION_PORT = 8;
      public static final int ABSOLUTE_ENCODER_PORT = 12;
      public static final double OFFSET = -25.31 - 2;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }
    
    /* * * CONVERSIONS FOR ENCODERS * * */
    //velocity in meters per sec instead of RPM 
    public static final double DRIVE_ENCODER_POSITION_CONVERSION = ((2 * Math.PI * (WHEEL_DIAMETER/2))) / DRIVE_GEAR_RATIO; //drive enc rotation
    //velocity in meters instead of rotations 
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed 
 
    /* * * PID VALUES FOR TURNING MOTOR PID * * */
    public static final double KP_TURNING = 0.0048;
    public static final double KI_TURNING = 0.0002;
    public static final double KD_TURNING = 0.0001;

    /* * * MAX * * */
    public static final double MAX_SPEED = 3.6576; //12.0 ft/s 
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    // public static class AutonomousConstants {
    //   public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
    //     new PIDConstants(5.0, 0.0, 0.00), //FIXME translation PID constants 
    //     new PIDConstants(5.0, 0.0, 0.0), //FIXME rotation PID constants 
    //     1.5, //FIXME max module speed in m/s 
    //     (Math.hypot(WHEEL_BASE, TRACK_WIDTH)) / 2, //FIXME drive base radius in meters. dist from robot center to furthest module 
    //     new ReplanningConfig() //default path replanning config 
    //     );
    // }
    
  }
}