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
  private DigitalInput bottomLimitSwitch;

  private PIDController pid;
  private double setpoint;
  private boolean pidOn;
  private double setpointTolerance;

  private double manualSpeed;
  private double maxPidSpeed;
  
  private double subwoofHeight;
  //private double distance;

  private double encInOneDeg;

  private double limelightAngle;
  private double aprilSubDis; //Distance between subwoofer opening & apriltag
  private double beta; 
  private double hypDist;
  private double finalAngle;

  public PivotSubsystem(){
    pivotMotor = new CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    pivotMotor.setInverted(true);
    limitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT);
    bottomLimitSwitch = new DigitalInput(PivotConstants.PIVOT_BOTTOM_LIMIT);
    encoder = pivotMotor.getEncoder();
    
    pid = new PIDController(0.05, 0, 0);
    setpoint = 0;
    setpointTolerance = 0.5;

    manualSpeed = 0;
    maxPidSpeed = 0.2;

    subwoofHeight = 1.98; //height of subwoofer opening in meters
    encInOneDeg = 0.083;

    limelightAngle = 105;
    aprilSubDis = 0.43; //a
    beta = 90 - limelightAngle;
    hypDist = Math.sqrt(Math.pow(aprilSubDis, 2) + Math.pow(getDistanceFromTarget(), 2) - 2*(aprilSubDis* getDistanceFromTarget()* Math.cos(beta))); //b
    finalAngle = Math.asin(aprilSubDis*Math.sin(beta)/ hypDist); //A

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
    return !limitSwitch.get();
  }

  public boolean bottomLimitSwitchPressed(){
    return !bottomLimitSwitch.get();
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
    if(Math.abs(speed) < 0.1){
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

  // returns the encoder count of the angle shooter should go to
  public double angleSubwooferShot(){
    return -finalAngle * encInOneDeg;
  }

  public double getDistanceFromTarget(){
    double distance = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getTranslation().getNorm();
    return distance;
  }
 
  @Override
  public void periodic() {
    if(topLimitSwitchPressed()){
      resetEnc();
    }
  
    double pidSpeed = 0;

    if(pidOn){
      pidSpeed = pid.calculate(encoder.getPosition(), setpoint);
    }
    else{
      pidSpeed = manualSpeed;
    }

    if(topLimitSwitchPressed() && pidSpeed > 0){
      pidSpeed = 0;
    }

    if(bottomLimitSwitchPressed() && pidSpeed < 0){
      pidSpeed = 0;
    }
    
    pivotMotor.set(pidSpeed);
    SmartDashboard.putBoolean("Pid On?", pidOn);
    SmartDashboard.putNumber("Speed", pidSpeed);
    SmartDashboard.putBoolean("Top limit switch pressed?", topLimitSwitchPressed());
    SmartDashboard.putNumber("Encoder values", returnEncoder());
    SmartDashboard.putBoolean("Bottom limit switch pressed?", bottomLimitSwitchPressed());
    
    //SmartDashboard.putNumber("distance from limelight", getDistanceFromTarget());

  }
}