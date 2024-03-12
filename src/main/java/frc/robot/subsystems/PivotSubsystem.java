// hardstop at the start position
// subwoofer position, amp position, feed position?

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//import com.revrobotics.SparkPIDController;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public class PivotSubsystem extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private AbsoluteEncoder encoder;
  private DigitalInput limitSwitch;
  private DigitalInput bottomLimitSwitch;

  private PIDController pid;
  private double setpoint;
  private boolean pidOn;
  private double setpointTolerance;

  private double manualSpeed;
  private double maxPidSpeed;

  private double finalAngle;

  private double horizontalDist;
  private double statsAngleCalc; 

  private double statsCalcAngle;

  public PivotSubsystem(){
    pivotMotor = new CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    pivotMotor.setInverted(false);
    limitSwitch = new DigitalInput(PivotConstants.PIVOT_TOP_LIMIT);
    bottomLimitSwitch = new DigitalInput(PivotConstants.PIVOT_BOTTOM_LIMIT);
    encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  
    encoder.setZeroOffset(270);
    encoder.setPositionConversionFactor(360);
    
    pid = new PIDController(PivotConstants.PIVOT_KP, PivotConstants.PIVOT_KI, PivotConstants.PIVOT_KD);
    setpoint = 0; //PID setPoint
    setpointTolerance = 1.5;

    manualSpeed = 0;
    maxPidSpeed = PivotConstants.MAX_SPEED;

    pid.enableContinuousInput(0, 360); 
    pid.setTolerance(1.5);

  }

 // sets pivotMotor to brake mode
  public void init(){
    pivotMotor.setIdleMode(IdleMode.kBrake);
  }

  //////////////////////
  // ENCODER METHODS  //
  //////////////////////

  // returns current encoder
  public double returnEncoder(){
    return encoder.getPosition();
  }

  ///////////////////
  //  PID METHODS //
  //////////////////

  // enable pid for pid commands
  public void enablePid(){
    pidOn = true;
    changeSetpoint(returnEncoder()); 
  }
  // disables pid for pid commands
  public void disablePid(){
    pidOn = false;
  }

    // changes setPoint for the pid 
  public void changeSetpoint(double newSetpoint){
    setpoint = 360 - newSetpoint;
  }

  /////////////////////
  //  CHECK METHODS  //
  ////////////////////

  // checks if topLimitSwitch is pressed
  public boolean topLimitSwitchPressed(){
    return !limitSwitch.get();
  }

  // checks if bottomLimitSwitch is pressed
  public boolean bottomLimitSwitchPressed(){
    return !bottomLimitSwitch.get();
  }

   // checks if pivot reached the setPoint 
  public boolean atSetpoint(){ 
    double error1 = setpoint - returnEncoder();
    return Math.abs(error1) < setpointTolerance;
  }

  /////////////////////////
  //  MOVEMENT METHODS   //
  ////////////////////////

  // stops the pivotMotor
  public void stopMotor(){
    pivotMotor.set(0);
  }

  // deadzone for joyStick
  public double deadzone(double speed){
    if(Math.abs(speed) < 0.1){
      return 0;
    }
    else{
      return speed;
    }
  }

  //setting the speed manually to the joyStick
  public void setManualSpeed(double speed){
    manualSpeed = deadzone(speed);
  }

  /////////////////////////
  //  LIMELIGHT METHODS  //
  ////////////////////////

  // returns the encoder count of the angle shooter should go to
  public double angleSubwooferShot(){
    return finalAngle;
  }

  public double returnCalcAngle (){
    return statsCalcAngle;
  }
 
  @Override
  public void periodic() {
    horizontalDist = Units.inchesToMeters(43) / (Math.tan(Units.degreesToRadians(LimelightHelpers.getTY("limelight") + 15)));
    statsCalcAngle = 84.3 + (-9.18 * horizontalDist) + (0.369 * Math.pow(horizontalDist, 2));

    finalAngle = Units.radiansToDegrees(Math.atan((Units.inchesToMeters(43) + Units.inchesToMeters(21))/horizontalDist));
    statsAngleCalc = 86 + (-9.5 * horizontalDist) + (0.386 * Math.pow(horizontalDist, 2)); 
    statsAngleCalc /= 2;
  
    double pidSpeed = 0;

    if(pidOn){
      pidSpeed = pid.calculate(encoder.getPosition(), setpoint);
    }
    else{
      pidSpeed = manualSpeed;
    }

    if(topLimitSwitchPressed() && pidSpeed < 0){
      pidSpeed = 0;
    }
    else if(bottomLimitSwitchPressed() && pidSpeed > 0){
      pidSpeed = 0;
    }
    else if(pidSpeed > maxPidSpeed){
      pidSpeed = maxPidSpeed;
    }
    else if(pidSpeed < -maxPidSpeed){
      pidSpeed = -maxPidSpeed;
    }

    pivotMotor.set(pidSpeed);

    SmartDashboard.putBoolean("[P] Pid On?", pidOn);
    SmartDashboard.putNumber("[P] Speed", pidSpeed);
    SmartDashboard.putBoolean("[P] Top limit switch pressed?", topLimitSwitchPressed());
    SmartDashboard.putNumber("[P] Encoder values", returnEncoder());
    SmartDashboard.putBoolean("[P] Bottom limit switch pressed?", bottomLimitSwitchPressed());
    SmartDashboard.putNumber("[P] calculated angle", finalAngle);
    SmartDashboard.putNumber("[P] distance from limelight", horizontalDist);
    SmartDashboard.putBoolean("[P] at setpoint?", atSetpoint());
    SmartDashboard.putNumber("[P] pid setpoint", setpoint);

    SmartDashboard.putNumber("[P] stats calc angle", returnCalcAngle());
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight"));

  }
}