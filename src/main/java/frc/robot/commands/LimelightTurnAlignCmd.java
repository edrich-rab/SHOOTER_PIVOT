// make a command for each specific april tag and in each command, initialize to a specific pipeline that only detects certain april tags 
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightTurnAlignCmd extends Command {

  /* * * DECLARATION * * */
  SwerveSubsystem swerveSubs; 
  PIDController anglePID; 

  DoubleSupplier xSupp, ySupp, zSupp; 
  int pipeline;
  private boolean fieldOriented;

  public LimelightTurnAlignCmd(SwerveSubsystem swerveSubs, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp, boolean fieldOriented, int pipeline) {
    this.swerveSubs = swerveSubs; 

    this.xSupp = xSupp; 
    this.ySupp = ySupp; 
    this.zSupp = zSupp; 
    this.fieldOriented = fieldOriented;
    this.pipeline = pipeline;

    anglePID = new PIDController(0.001, 0, 0);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", pipeline);
    //swerveSubs.resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("pipeline", pipeline);
    SmartDashboard.putString("CURRENT CMD", getName());
    SwerveModuleState[] states; 

    // TURN SUPPLIERS INTO DOUBLES 
    double xSpeed = xSupp.getAsDouble(); 
    double ySpeed = ySupp.getAsDouble(); 
    double zSpeed = zSupp.getAsDouble();

    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 
    zSpeed = deadzone(zSpeed);

    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 
    zSpeed = modifyAxis(zSpeed); 

    
    double rotationSpeed = anglePID.calculate(LimelightHelpers.getTX("limelight"), 0);

    if(fieldOriented){
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(rotationSpeed, ySpeed, zSpeed, swerveSubs.getRotation2d())
      );
    }
    else{
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      new ChassisSpeeds(rotationSpeed, ySpeed, zSpeed)
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
    SmartDashboard.putString("CURRENT CMD", "");
    return false;
  }

   /* * * ADDED METHODS * * */
   public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
  }
  
  private static double modifyAxis(double num) {
    // Square the axis
    num = num * num * Math.signum(num);

    return num;
  }
}