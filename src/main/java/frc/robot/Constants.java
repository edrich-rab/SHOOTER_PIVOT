// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// I LOVE VIETNAM
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PivotConstants{
    public static final int PIVOT_MOTOR_PORT = 8;
    public static final int PIVOT_LIMIT = 9;
    public static final int JOYSTICK_PORT = 0;

    public static final double subWooferEnc = 10;
    public static final double ampEnc = 10;
    public static final double wingEnc = 10;

  }
}
