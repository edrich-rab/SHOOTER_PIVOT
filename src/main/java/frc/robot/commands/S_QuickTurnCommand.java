package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class S_QuickTurnCommand extends CommandBase {

  /* * * DECLARATION * * */
  SwerveSubsystem swerveSubs; 
  PIDController anglePID; 

  DoubleSupplier xSupp, ySupp, zSupp; 

  public S_QuickTurnCommand(SwerveSubsystem swerveSubs, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp) {
    this.swerveSubs = swerveSubs; 

    this.xSupp = xSupp; 
    this.ySupp = ySupp; 
    this.zSupp = zSupp; 

    anglePID = new PIDController(SwerveConstants.KP_ANGLE, SwerveConstants.KI_ANGLE, SwerveConstants.KD_ANGLE);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //swerveSubs.resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    swerveSubs.drive(xSpeed, ySpeed, rotationSpeed, true);
  
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