package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;
//import frc.robot.Constants.SwerveConstants.AutonomousConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
  /* * * INITIALIZATION * * */

  //initialize SwerveModules 
  private SwerveModule[] swerveModules; 

  //odometer 
  private SwerveDriveOdometry odometer; 
  private AHRS navx; 

  // private BooleanSupplier shouldFlipPath = () -> false;

  public SwerveSubsystem() {

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.FrontLeft.constants), 
      new SwerveModule(1, SwerveConstants.BackLeft.constants), 
      new SwerveModule(2, SwerveConstants.FrontRight.constants), 
      new SwerveModule(3, SwerveConstants.BackRight.constants)
    };



    //instantiate navx 
    navx = new AHRS();
    navx.zeroYaw();

    //instantiate odometer 
    odometer = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      navx.getRotation2d(), 
      getModulePositions()
    );

    // AutoBuilder.configureHolonomic(
    //   this::getPose, 
    //   this::resetOdometry, 
    //   this::getRobotRelativeSpeeds, 
    //   this::driveRobotRelative, 
    //   AutonomousConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG, 
    //   () -> false, 
    //   this //reference to this subsystem to set requirements 
    //   );

  }

    /* * * ODOMETRY * * */

  //returns the Rotation2d object 
  //a 2d coordinate represented by a point on the unit circle (the rotation of the robot)
  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public void resetNavx() {
    navx.reset();
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  // FIXME i dont think this works as intended,, resetPosition should reset everything to 0 
  public void setPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).vxMetersPerSecond, SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).vyMetersPerSecond, SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
  }

  public void driveRobotRelative(ChassisSpeeds chassis) {
    SwerveModuleState[] state = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassis);

    setModuleStates(state);
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented) { //for non field oriented drive
    SwerveModuleState[] swerveModuleStates; 

    if (fieldOriented) {
      swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d())
      );
    } else {
      swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed)
      );
    }

    setModuleStates(swerveModuleStates);
  }
  /* * * STATES * * */

  //SET STATES 
  //gets a SwerveModuleStates array from driver control and sets each module to the corresponding SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleID]);
    }
  }

  //GET STATES 
  //returns the states of the swerve modules in an array 
  //getState uses drive velocity and module rotation 
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      states[swerveMod.moduleID] = swerveMod.getState();
    }

    return states; 
  }

  //GET POSITIONS
  //returns the positions of the swerve modules in an array 
  //getPosition uses drive enc and module rotation 
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      positions[swerveMod.moduleID] = swerveMod.getPosition();
    }

    return positions;
  }

  //LOCK 
  public void lock() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setAngle(states[swerveMod.moduleID]);
    }
  }

  //STRAIGHTEN THE WHEELS 
  public void straightenWheels() { //set all wheels to 0 degrees 
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(states[swerveMod.moduleID]);
    }
  }

  //STOP 
  public void stopModules() {
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.stop();
    }
  }

  public double getDistanceFromTarget(){ //find out if it is horizontal distance
    double distance = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getTranslation().getNorm();
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(navx.getRotation2d(), getModulePositions());
    
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.print();
    }

    SmartDashboard.putNumber("NAVX", navx.getYaw());
    SmartDashboard.putString("POSE INFO", odometer.getPoseMeters().toString());
    // SmartDashboard.putString("WORKING DIR", System.getProperty("user.dir"));
    
  }
}