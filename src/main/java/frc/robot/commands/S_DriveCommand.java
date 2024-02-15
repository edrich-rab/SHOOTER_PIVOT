package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class S_DriveCommand extends CommandBase {
  private SwerveSubsystem swerveSubs; 
  private PivotSubsystem pivotSubs;
  PIDController drivePid;

  private DoubleSupplier xSupplier, ySupplier, zSupplier; 
  //private boolean fieldOriented; 
  private double idealDist;

  public S_DriveCommand(SwerveSubsystem swerveSubs, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, double idealDistance) {
    this.swerveSubs = swerveSubs; 
    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier; 
    this.zSupplier = zSupplier; 
    //this.fieldOriented = fieldOriented; 
    idealDist = idealDistance;

    //FIXME tune the pid!!!!!
    drivePid = new PIDController(SwerveConstants.KP_DRIVE, SwerveConstants.KI_DRIVE, SwerveConstants.KD_DRIVE);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // swerveSubs.resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("DONE", "not done");
    SwerveModuleState[] states; 
    /* * * ALTERING VALUES * * */
    //Joystick values -> double 
    double xSpeed = xSupplier.getAsDouble(); 
    double ySpeed = ySupplier.getAsDouble(); 
    double zSpeed = zSupplier.getAsDouble(); 

    //apply deadzone to speed values 
    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 
    zSpeed = deadzone(zSpeed); 

    //square the speed values to make for smoother acceleration 
    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 
    zSpeed = modifyAxis(zSpeed); 

    double forwardDriveSpeed = drivePid.calculate(swerveSubs.getDistanceFromTarget(), idealDist);

    swerveSubs.drive(xSpeed, -forwardDriveSpeed, zSpeed, true);
    SmartDashboard.putNumber("pid output speed", forwardDriveSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("DONE", "DONE");
    return false;
  }

  /* * * ADDED METHODS * * */
  public double deadzone(double num){
    return Math.abs(num) > 0.2 ? num : 0;
  }

  private static double modifyAxis(double num) {
    // Square the axis
    num = num * num * Math.signum(num);

    return num;
  }

  //waialua wrap method from Conversions.java 
  public static double wrap(double angle1, double angle2) {
    double difference = angle1 - angle2;
    if (difference > 180) {
        difference -= 360;
        //difference = -difference;
    } else if (difference < -180) {
        difference += 360;
        //difference = -difference;
    }
    return difference;
  }


}