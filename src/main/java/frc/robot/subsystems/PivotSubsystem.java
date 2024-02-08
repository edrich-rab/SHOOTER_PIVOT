// hardstop at the start position
// subwoofer position, amp position, feed position?

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.*;

public class PivotSubsystem extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private RelativeEncoder encoder;
  private DigitalInput limitSwitch;

  private PIDController pid;
  private double setpoint;
  private boolean pidOn;
  private double setpointTolerance;

  private double manualSpeed;
  private double maxPidSpeed;

  private NetworkTable limelight;
  private NetworkTableEntry botpos;

  private double[] botposeArray;

  private double[] converted;

  public PivotSubsystem(){
    pivotMotor = new CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    limitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT);
    encoder = pivotMotor.getEncoder();
    
    pid = new PIDController(0.01, 0, 0);
    setpoint = 0;
    setpointTolerance = 2.5;

    manualSpeed = 0;
    maxPidSpeed = 0.2;

    limelight = null;
    botpos = null;

    botposeArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double [6]);
  

  }

  private NetworkTable getLimelight(){
    if(limelight == null){
      limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }
    return limelight;
  }

  private double getBotPosition(){
    double distance;
    /* 
    if(getLimelight() == null){
      botpos = null;
    }
    else{
      botpos = getLimelight().getEntry("targetPose_RobotSpace");
    }
    return botpos; */
    distance = LimelightHelpers.getBotPose3d_TargetSpace("limelight").toPose2d().getTranslation().getNorm();
    return distance;
  }

  public void init(){
    pivotMotor.setIdleMode(IdleMode.kBrake);

    /* get Pose3D 
     * from the Pose3D object use .toPose2d() to get a Pose2D object 
     * from the Pose2D object use .getTranslation() to get a Translation2d object 
     * from Translation2d object use .getNorm() to get distance from limelight to april tag's position 
     *    Pose3dObj.toPose2d().getTranslation().getNorm(); it will be a double representing that dist ^^
     * if you want distance from robot to april tag, find out how to get a Pose3d object representing the robot on the field 
     * MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW MEOW
     */
  }

  //////////////////////
  // ENCODER METHODS  //
  //////////////////////

  public double returnEncoder(){
    return encoder.getPosition();
  }

  public void resetEnc(){
    encoder.setPosition(0);
  }

  ///////////////////
  //  PID METHODS //
  //////////////////

  public void enablePid(){
    pidOn = true;
    changeSetpoint(returnEncoder()); 
  }

  public void disablePid(){
    pidOn = false;
  }

  public void changeSetpoint(double newSetpoint){
    setpoint = newSetpoint;
  }

  /////////////////////
  //  CHECK METHODS  //
  ////////////////////

  public boolean topLimitSwitchPressed(){
    return limitSwitch.get();
  }


  public boolean atSetpoint(){ 
    double error1 = setpoint - returnEncoder();
    return Math.abs(error1) < setpointTolerance;
  }

  /////////////////////////
  //  MOVEMENT METHODS   //
  ////////////////////////

  public void stopMotor(){
    pivotMotor.set(0);
  }

  public double deadzone(double speed){
    if(speed < 0.1 && speed > -0.1 ){
      return 0;
    }
    else{
      return speed;
    }
  }

  public void autoShoot(){
    
  }

  public void setManualSpeed(double speed){
    manualSpeed = deadzone(speed);
  }

  public void adjustPidShoot(){
   
  }
 
  @Override
  public void periodic() {
    if(topLimitSwitchPressed()){
      resetEnc();
    }
  
    double pidSpeed = 0;

    if(pidOn){
      pidSpeed = -pid.calculate(setpoint, encoder.getPosition());
    }
    else{
      pidSpeed = manualSpeed;
    }

    
    if(topLimitSwitchPressed() && pidSpeed > 0){
      pidSpeed = 0;
    }
    else if(pidSpeed > maxPidSpeed){
      pidSpeed = maxPidSpeed;
    }
    else if(pidSpeed < -maxPidSpeed){
      pidSpeed = - maxPidSpeed;
    }
    
  
    pivotMotor.set(pidSpeed);

    SmartDashboard.putBoolean("Pid On?", pidOn);
    SmartDashboard.putNumber("Speed", pidSpeed);
    SmartDashboard.putBoolean("Limit switch pressed?", topLimitSwitchPressed());
    SmartDashboard.putNumber("Encoder values", returnEncoder());
    
    SmartDashboard.putNumber("distance from limelight", getBotPosition());

  }
}