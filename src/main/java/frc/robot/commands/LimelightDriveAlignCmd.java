package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightDriveAlignCmd extends Command {
  private SwerveSubsystem swerveSubs; 
  //private PivotSubsystem pivotSubs;
  PIDController drivePid;
  Optional<Alliance> ally;

  //private boolean fieldOriented;

  //private DoubleSupplier xSupplier, ySupplier, zSupplier; 
  //private boolean fieldOriented; 
  private double idealDist;

  public LimelightDriveAlignCmd(SwerveSubsystem swerveSubs, double idealDistance) {
    this.swerveSubs = swerveSubs; 
    // this.xSupplier = xSupplier; 
    // this.ySupplier = ySupplier; 
    // this.zSupplier = zSupplier; 
    //this.fieldOriented = fieldOriented; 
    idealDist = idealDistance;
    ally = DriverStation.getAlliance();

    //FIXME tune the pid!!!!!
    drivePid = new PIDController(0.05, 0, 0);

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
    System.out.println("running command");
    SmartDashboard.putString("DONE", "not done");
    SwerveModuleState[] states; 
    /* * * ALTERING VALUES * * */
    //Joystick values -> double 
    // double xSpeed = xSupplier.getAsDouble(); 
    // double ySpeed = ySupplier.getAsDouble(); 
    // double zSpeed = zSupplier.getAsDouble(); 

    //apply deadzone to speed values 
    // xSpeed = deadzone(xSpeed); 
    // ySpeed = deadzone(ySpeed); 
    // zSpeed = deadzone(zSpeed); 

    //square the speed values to make for smoother acceleration 
    // xSpeed = modifyAxis(xSpeed); 
    // ySpeed = modifyAxis(ySpeed); 
    // zSpeed = modifyAxis(zSpeed); 

    double forwardDriveSpeed = drivePid.calculate(swerveSubs.getDistanceFromTarget(), idealDist);
    if (ally.get() == Alliance.Red) {
      forwardDriveSpeed = -forwardDriveSpeed;
    }

    states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(0, forwardDriveSpeed, 0, swerveSubs.getRotation2d())
    );

    if(LimelightHelpers.getTV("limelight")){
      swerveSubs.setModuleStates(states);
    }
    else{
      swerveSubs.stopModules();
    }
    
    SmartDashboard.putNumber("drive speed", forwardDriveSpeed);
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