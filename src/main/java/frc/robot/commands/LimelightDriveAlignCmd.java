package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.SwerveConstants;

public class LimelightDriveAlignCmd extends Command {
  private SwerveSubsystem swerveSubs;
  private PivotSubsystem pivotSubs; 

  private DoubleSupplier xSupplier, ySupplier, zSupplier; 
  private boolean fieldOriented; 

  private double pipeline;
  private double idealDist;

  PIDController testDrivePid;
  /* * * CONSTRUCTOR * * */
  /* 
   * @param swerveSubs the swerve subsystem 
   * @param xSupplier value input for strafe on x-axis 
   * @param ySupplier value input for strafe on y-axis 
   * @param zSupplier value input for rotation 
   * @param fieldOriented whether or not we want the bot to run in field oriented 
   */
  public LimelightDriveAlignCmd(SwerveSubsystem swerveSubs, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldOriented, double pipeline, double idealDist) {
    this.swerveSubs = swerveSubs; 
    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier; 
    this.zSupplier = zSupplier; 
    this.fieldOriented = fieldOriented; 

    this.pipeline = pipeline;
    this.idealDist = idealDist;

    testDrivePid = new PIDController(0.001, 0, 0);
    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SwerveModuleState[] states; 
    /* * * ALTERING VALUES * *   */
    //Joystick values -> double 
    double xSpeed = xSupplier.getAsDouble(); 
    double ySpeed = ySupplier.getAsDouble(); 
    double zSpeed = zSupplier.getAsDouble(); 

    SmartDashboard.putNumber("z speed", zSpeed);


    //apply deadzone to speed values 
    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 
    zSpeed = deadzone(zSpeed); 

    //square the speed values to make for smoother acceleration 
    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 
    zSpeed = modifyAxis(zSpeed); 

    double driveSpeed = testDrivePid.calculate(LimelightHelpers.getTY("limelight"), idealDist);

    /* * * SETTING SWERVE STATES * * */ 
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeed, ySpeed, zSpeed, swerveSubs.getRotation2d())
      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(driveSpeed, ySpeed, zSpeed)
      );
    }

    swerveSubs.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

/* * * ADDED METHODS * * */
public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
}

private static double modifyAxis(double num) {
  // Square the axis
  num = Math.copySign(num * num, num);

  return num;
}


}