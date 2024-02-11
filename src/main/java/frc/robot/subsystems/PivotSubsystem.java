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
  
  private double subwoofHeight;
  private double distance;

  private double tixInOneRotation; 
  private double tixInOneDeg;

  private double limelightAngle;
  private double aprilSubDis; //Distance between subwoofer opening & apriltag
  private double beta; 
  private double hypDist;
  private double finalAngle;

  // amount of ticks in one degree = 2.84 encoder tix

  public PivotSubsystem(){
    pivotMotor = new CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    limitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT);
    encoder = pivotMotor.getEncoder();
    
    pid = new PIDController(0.01, 0, 0);
    setpoint = 0;
    setpointTolerance = 0.5;

    manualSpeed = 0;
    maxPidSpeed = 0.01;

    limelight = null;
    subwoofHeight = 1.98; //height of subwoofer opening in meters
    tixInOneRotation = 6; // or 4096
    tixInOneDeg = tixInOneRotation/360;

    limelightAngle = 10;
    aprilSubDis = 0.43;
    beta = 90 - limelightAngle;
    hypDist = Math.pow(aprilSubDis, 2) + Math.pow(getDistanceFromTarget(), 2) - 2*(aprilSubDis* getDistanceFromTarget()* Math.cos(beta));
    finalAngle = Math.asin(aprilSubDis*Math.sin(beta)/ hypDist);

  }

  public void init(){
    pivotMotor.setIdleMode(IdleMode.kBrake);
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
    if(speed < 0.3 && speed > -0.3 ){
      return 0;
    }
    else{
      return speed;
    }
  }

  public void setManualSpeed(double speed){
    manualSpeed = deadzone(speed);
  }

  /////////////////////////
  //  LIMELIGHT METHODS  //
  ////////////////////////

  private double getDistanceFromTarget(){ //find out if it is horizontal distance
    distance = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getTranslation().getNorm();
    return distance;
  }

  // returns the encoder count of the angle shooter should go to
  public double angleSubwooferShot(){
    return finalAngle * tixInOneDeg ;
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
    
    SmartDashboard.putNumber("distance from limelight", getDistanceFromTarget());

  }
}