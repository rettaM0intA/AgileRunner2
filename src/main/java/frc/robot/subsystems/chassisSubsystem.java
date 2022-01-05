// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControllerInControl;
import frc.robot.RobotContainer;

public class chassisSubsystem extends SubsystemBase {

  //Init Analog Encoders to help reset the wheel rotation
  public AnalogEncoder fRAnalogEncoder = new AnalogEncoder(new AnalogInput(2));
  public AnalogEncoder fLAnalogEncoder = new AnalogEncoder(new AnalogInput(0));
  public AnalogEncoder bRAnalogEncoder = new AnalogEncoder(new AnalogInput(3));
  public AnalogEncoder bLAnalogEncoder = new AnalogEncoder(new AnalogInput(1));

  //The Falcon 500s are in charge of spinning the wheels
  WPI_TalonFX fRDriveMotor = new WPI_TalonFX(6);
  WPI_TalonFX fLDriveMotor = new WPI_TalonFX(21);
  WPI_TalonFX bRDriveMotor = new WPI_TalonFX(2);
  WPI_TalonFX bLDriveMotor = new WPI_TalonFX(11);

  
  //The Neo550s are in charge of rotating the wheels
  public CANSparkMax fRrotationMotor = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax fLrotationMotor = new CANSparkMax(20, MotorType.kBrushless);
  public CANSparkMax bRrotationMotor = new CANSparkMax(10, MotorType.kBrushless);
  public CANSparkMax bLrotationMotor = new CANSparkMax(1, MotorType.kBrushless);
  
  SlewRateLimiter frontLeftLimiter = new SlewRateLimiter(.72);
  SlewRateLimiter frontRightLimiter = new SlewRateLimiter(.72);
  SlewRateLimiter backLeftLimiter = new SlewRateLimiter(.72);
  SlewRateLimiter backRightLimiter = new SlewRateLimiter(.72);

  ControllerInControl driver = ControllerInControl.flightStick;

  public AHRS gyro = new AHRS(I2C.Port.kOnboard);


  int frontLeftOnPointCount = 0;
  int frontRightOnPointCount = 0;
  int backLeftOnPointCount = 0;
  int backRightOnPointCount = 0;
  
  //The angles I want the wheels to face in
  double fRAngle = 0;
  double fLAngle = 0;
  double bRAngle = 0;
  double bLAngle = 0;
  
  //This angle is used to test functions.
  double testAngle = 0;

  double lastSpeedfL = 0;
  double lastSpeedfR = 0;
  double lastSpeedbL = 0;
  double lastSpeedbR = 0;

  PIDController fLPidController = new PIDController(0.00013, 0.000028, 0);
  PIDController fRPidController = new PIDController(0.00013, 0.000028, 0);
  PIDController bLPidController = new PIDController(0.00013, 0.000028, 0);
  PIDController bRPidController = new PIDController(0.00013, 0.000028, 0);
  
  int currentRotationFl = 1;
  int currentRotationFr = 1;
  int currentRotationBl = 1;
  int currentRotationBr = 1;

  // makes sure that the setPid method is only run once 
  boolean setPid = true;


  // Declare the swervedrive math stuff from WPI
  SwerveDriveKinematics m_kinematics;

  /** Creates a new chassisSubsystem. */
  public chassisSubsystem() {

    wheelBrakes();
  
  // Locations for the swerve drive modules relative to the robot center.
  Translation2d frontLeftLocation = new Translation2d(0.292, 0.286);
  Translation2d frontRightLocation = new Translation2d(0.292, -0.286);
  Translation2d backLeftLocation = new Translation2d(-0.292, 0.286);
  Translation2d backRightLocation = new Translation2d(-0.292, -0.286);

    fLrotationMotor.setInverted(false);

  if(setPid){
    SetPIDController();
  }

  m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  }

  /**
   * This is the main driving function for the AgileRunner robot.
   * @param fwd Percent forward.  Used to decide which direction the robot goes in with fwd
   * @param strafe Percent strafe.  Used to decide which direction the robot goes in with fwd
   * @param rotation Percent for rotating.  Will combine with the direction given by fwd and strafe to let the robot turn.
   */
  public void driveTeleop(double fwd, double strafe, double rotation){

    if(RobotContainer.operator.getYButton()){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }else if(RobotContainer.driver.getRawButton(9)){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }

    //The following if statements are to set controller deadzones.
    if(fwd < Constants.kDirectionalDeadzone && fwd > -Constants.kDirectionalDeadzone){
      fwd = 0;
    }
    if(strafe < Constants.kDirectionalDeadzone && strafe > -Constants.kDirectionalDeadzone){
      strafe = 0;
    }

    // convert fwd from flight stick y of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double fwd_MpS = -fwd * Constants.kChassisMaxMetersPerSec; 
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("fwd_MpS", fwd_MpS);

    // convert strafe from flight stick x of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double strafe_MpS = -strafe * Constants.kChassisMaxMetersPerSec;
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("strafe_MpS", strafe_MpS);

    // convert rotation from flight stick twist of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double rotation_RpS = -rotation * Constants.kChassisMaxRadiansPerSec;
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("rotation_RpS", rotation_RpS);

    // Calculate the module speeds based on what the requested chassis speeds are.
    ChassisSpeeds speeds = new ChassisSpeeds(fwd_MpS,strafe_MpS,rotation_RpS);

    /**The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd_MpS, strafe_MpS, rotation_RpS, Rotation2d.fromDegrees(gyro.getAngle()));
    */
    // Get a reference to the module states for a swerve drive system
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize the speeds in case one wants to be higher that the max speed
    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.kChassisMaxMetersPerSec);

    // Get a reference to each module states
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    SwerveModuleState frontLeftOptimize = SwerveModuleState.optimize(frontLeft, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState frontRightOptimize = SwerveModuleState.optimize(frontRight, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState backLeftOptimize = SwerveModuleState.optimize(backLeft, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState backRightOptimize = SwerveModuleState.optimize(backRight, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));

    // Get the needed angle from the module state and convert it to the Cnts needed for the CanSparkMAx PID loop
    fLAngle = (frontLeftOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    fRAngle = (frontRightOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    bLAngle = (backLeftOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    bRAngle = (backRightOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;


    // Get the needed speed from the module state and convert it to the -1 to 1 value needed for percent output command of the CANTalon
    double frontLeftSpeed = frontLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double frontRightSpeed = frontRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backLeftSpeed = backLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backRightSpeed = backRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;

    //The goal of these four uses of rotationOverflow is to have the wheels avoid a 350+ degree rotation
    rotationOverflow(fLrotationMotor, 0);
    rotationOverflow(fRrotationMotor, 1);
    rotationOverflow(bLrotationMotor, 2);
    rotationOverflow(bRrotationMotor, 3);
    
    //these lines tell the motor controller what poisition to set the motor to
    fLrotationMotor.getPIDController().setReference(fLAngle, ControlType.kPosition);
    fRrotationMotor.getPIDController().setReference(fRAngle, ControlType.kPosition);
    bLrotationMotor.getPIDController().setReference(bLAngle, ControlType.kPosition);
    bRrotationMotor.getPIDController().setReference(bRAngle, ControlType.kPosition);
    

    //WIP for replacing the avoidance method of infinite rotation with real infinite rotation.
    //Uses the turn forever method of going past full rotation.  Make sure to comment out rotationOverflow calls and the previous inputs to the rotation motors.
    // fLrotationMotor.getPIDController().setReference(TurnForever(fLrotationMotor, fLAngle), ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(TurnForever(fRrotationMotor, fRAngle), ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(TurnForever(bLrotationMotor, bLAngle), ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(TurnForever(bRrotationMotor, bRAngle), ControlType.kPosition);
    
    
    // Set the speed in TalonFX to a percent output.
    fLDriveMotor.set(frontLeftLimiter.calculate(frontLeftSpeed));
    fRDriveMotor.set(-frontRightLimiter.calculate(frontRightSpeed));
    bLDriveMotor.set(backLeftLimiter.calculate(backLeftSpeed));
    bRDriveMotor.set(-backRightLimiter.calculate(backRightSpeed));

    // fLDriveMotor.set(0);
    // fRDriveMotor.set(0);
    // bLDriveMotor.set(0);
    // bRDriveMotor.set(0);

    //fLDriveMotor.set(fLPidController.calculate(fLDriveMotor.getSelectedSensorPosition(), 1000));

    lastSpeedfL = frontLeftSpeed;
    lastSpeedfR = frontRightSpeed;
    lastSpeedbL = backLeftSpeed;
    lastSpeedbR = backRightSpeed;

  }

  
  /**
   * This is the main driving function for the AgileRunner robot.
   * @param fwd Percent forward.  Used to decide which direction the robot goes in with fwd
   * @param strafe Percent strafe.  Used to decide which direction the robot goes in with fwd
   * @param rotation Percent for rotating.  Will combine with the direction given by fwd and strafe to let the robot turn.
   */
  public void driveAuton(double fwd, double strafe, double rotation){

    if(RobotContainer.operator.getYButton()){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }else if(RobotContainer.driver.getRawButton(9)){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }

    //The following if statements are to set controller deadzones.
    if(fwd < Constants.kDirectionalDeadzone && fwd > -Constants.kDirectionalDeadzone){
      fwd = 0;
    }
    if(strafe < Constants.kDirectionalDeadzone && strafe > -Constants.kDirectionalDeadzone){
      strafe = 0;
    }

    // convert fwd from flight stick y of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double fwd_MpS = -fwd * Constants.kChassisMaxMetersPerSec; 
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("fwd_MpS", fwd_MpS);

    // convert strafe from flight stick x of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double strafe_MpS = -strafe * Constants.kChassisMaxMetersPerSec;
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("strafe_MpS", strafe_MpS);

    // convert rotation from flight stick twist of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double rotation_RpS = -rotation * Constants.kChassisMaxRadiansPerSec;
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("rotation_RpS", rotation_RpS);

    // Calculate the module speeds based on what the requested chassis speeds are.
    ChassisSpeeds speeds = new ChassisSpeeds(fwd_MpS,strafe_MpS,rotation_RpS);

    /**The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd_MpS, strafe_MpS, rotation_RpS, Rotation2d.fromDegrees(gyro.getAngle()));
    */
    // Get a reference to the module states for a swerve drive system
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize the speeds in case one wants to be higher that the max speed
    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.kChassisMaxMetersPerSec);

    // Get a reference to each module states
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    SwerveModuleState frontLeftOptimize = SwerveModuleState.optimize(frontLeft, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState frontRightOptimize = SwerveModuleState.optimize(frontRight, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState backLeftOptimize = SwerveModuleState.optimize(backLeft, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState backRightOptimize = SwerveModuleState.optimize(backRight, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));

    // Get the needed angle from the module state and convert it to the Cnts needed for the CanSparkMAx PID loop
    fLAngle = (frontLeftOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    fRAngle = (frontRightOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    bLAngle = (backLeftOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    bRAngle = (backRightOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;


    // Get the needed speed from the module state and convert it to the -1 to 1 value needed for percent output command of the CANTalon
    double frontLeftSpeed = frontLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double frontRightSpeed = frontRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backLeftSpeed = backLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backRightSpeed = backRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;

    //The goal of these four uses of rotationOverflow is to have the wheels avoid a 350+ degree rotation
    rotationOverflow(fLrotationMotor, 0);
    rotationOverflow(fRrotationMotor, 1);
    rotationOverflow(bLrotationMotor, 2);
    rotationOverflow(bRrotationMotor, 3);
    
    //these lines tell the motor controller what poisition to set the motor to
    fLrotationMotor.getPIDController().setReference(fLAngle, ControlType.kPosition);
    fRrotationMotor.getPIDController().setReference(fRAngle, ControlType.kPosition);
    bLrotationMotor.getPIDController().setReference(bLAngle, ControlType.kPosition);
    bRrotationMotor.getPIDController().setReference(bRAngle, ControlType.kPosition);
    

    //WIP for replacing the avoidance method of infinite rotation with real infinite rotation.
    //Uses the turn forever method of going past full rotation.  Make sure to comment out rotationOverflow calls and the previous inputs to the rotation motors.
    // fLrotationMotor.getPIDController().setReference(TurnForever(fLrotationMotor, fLAngle), ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(TurnForever(fRrotationMotor, fRAngle), ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(TurnForever(bLrotationMotor, bLAngle), ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(TurnForever(bRrotationMotor, bRAngle), ControlType.kPosition);
    
    
    
    // Set the speed in TalonFX to a percent output.



    fLDriveMotor.set(frontLeftSpeed);
    fRDriveMotor.set(-frontRightSpeed);
    bLDriveMotor.set(backLeftSpeed);
    bRDriveMotor.set(-backRightSpeed);


    lastSpeedfL = frontLeftSpeed;
    lastSpeedfR = frontRightSpeed;
    lastSpeedbL = backLeftSpeed;
    lastSpeedbR = backRightSpeed;

  }

  
  /**
   * This is the main driving function for the AgileRunner robot.
   * @param fwd Percent forward.  Used to decide which direction the robot goes in with fwd
   * @param strafe Percent strafe.  Used to decide which direction the robot goes in with fwd
   * @param rotation Percent for rotating.  Will combine with the direction given by fwd and strafe to let the robot turn.
   */
  public void driveToPoint(double fwd, double strafe, double rotation, double fLgoalPosition, double fRgoalPosition, double bLgoalPosition, double bRgoalPosition){

    if(RobotContainer.operator.getYButton()){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }else if(RobotContainer.driver.getRawButton(9)){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }

    //The following if statements are to set controller deadzones.
    if(fwd < Constants.kDirectionalDeadzone && fwd > -Constants.kDirectionalDeadzone){
      fwd = 0;
    }
    if(strafe < Constants.kDirectionalDeadzone && strafe > -Constants.kDirectionalDeadzone){
      strafe = 0;
    }

    // convert fwd from flight stick y of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double fwd_MpS = -fwd * Constants.kChassisMaxMetersPerSec; 
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("fwd_MpS", fwd_MpS);

    // convert strafe from flight stick x of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double strafe_MpS = -strafe * Constants.kChassisMaxMetersPerSec;
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("strafe_MpS", strafe_MpS);

    // convert rotation from flight stick twist of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double rotation_RpS = -rotation * Constants.kChassisMaxRadiansPerSec;
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("rotation_RpS", rotation_RpS);

    // Calculate the module speeds based on what the requested chassis speeds are.
    ChassisSpeeds speeds = new ChassisSpeeds(fwd_MpS,strafe_MpS,rotation_RpS);

    /**The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd_MpS, strafe_MpS, rotation_RpS, Rotation2d.fromDegrees(gyro.getAngle()));
    */
    // Get a reference to the module states for a swerve drive system
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize the speeds in case one wants to be higher that the max speed
    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.kChassisMaxMetersPerSec);

    // Get a reference to each module states
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    SwerveModuleState frontLeftOptimize = SwerveModuleState.optimize(frontLeft, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState frontRightOptimize = SwerveModuleState.optimize(frontRight, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState backLeftOptimize = SwerveModuleState.optimize(backLeft, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));
    SwerveModuleState backRightOptimize = SwerveModuleState.optimize(backRight, new Rotation2d((fLrotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation) * 0.0174533));

    // Get the needed angle from the module state and convert it to the Cnts needed for the CanSparkMAx PID loop
    fLAngle = (frontLeftOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    fRAngle = (frontRightOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    bLAngle = (backLeftOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    bRAngle = (backRightOptimize.angle.getDegrees()) / Constants.kChassisSwerveOutputDegreeToNeoRotation;


    // Get the needed speed from the module state and convert it to the -1 to 1 value needed for percent output command of the CANTalon
    double frontLeftSpeed = frontLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double frontRightSpeed = frontRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backLeftSpeed = backLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backRightSpeed = backRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;

    //The goal of these four uses of rotationOverflow is to have the wheels avoid a 350+ degree rotation
    rotationOverflow(fLrotationMotor, 0);
    rotationOverflow(fRrotationMotor, 1);
    rotationOverflow(bLrotationMotor, 2);
    rotationOverflow(bRrotationMotor, 3);
    
    //these lines tell the motor controller what poisition to set the motor to
    fLrotationMotor.getPIDController().setReference(fLAngle, ControlType.kPosition);
    fRrotationMotor.getPIDController().setReference(fRAngle, ControlType.kPosition);
    bLrotationMotor.getPIDController().setReference(bLAngle, ControlType.kPosition);
    bRrotationMotor.getPIDController().setReference(bRAngle, ControlType.kPosition);
    

    //WIP for replacing the avoidance method of infinite rotation with real infinite rotation.
    //Uses the turn forever method of going past full rotation.  Make sure to comment out rotationOverflow calls and the previous inputs to the rotation motors.
    // fLrotationMotor.getPIDController().setReference(TurnForever(fLrotationMotor, fLAngle), ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(TurnForever(fRrotationMotor, fRAngle), ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(TurnForever(bLrotationMotor, bLAngle), ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(TurnForever(bRrotationMotor, bRAngle), ControlType.kPosition);
    
    
    // Set the speed in TalonFX to a percent output.
    // fLDriveMotor.set(frontLeftLimiter.calculate(frontLeftSpeed));
    // fRDriveMotor.set(-frontRightLimiter.calculate(frontRightSpeed));
    // bLDriveMotor.set(backLeftLimiter.calculate(backLeftSpeed));
    // bRDriveMotor.set(-backRightLimiter.calculate(backRightSpeed));

    fLDriveMotor.set(fLPidController.calculate(fLDriveMotor.getSelectedSensorPosition(), fLgoalPosition));
    fRDriveMotor.set(-fRPidController.calculate(fLDriveMotor.getSelectedSensorPosition(), -fRgoalPosition));
    bLDriveMotor.set(bLPidController.calculate(fLDriveMotor.getSelectedSensorPosition(), bLgoalPosition));
    bRDriveMotor.set(-bRPidController.calculate(fLDriveMotor.getSelectedSensorPosition(), -bRgoalPosition));

    // lastSpeedfL = frontLeftSpeed;
    // lastSpeedfR = frontRightSpeed;
    // lastSpeedbL = backLeftSpeed;
    // lastSpeedbR = backRightSpeed;



  }

  /**
   * A function made to avoid going to 0 or 360 degrees in rotation.
   * @param rotationMotor Used to check a motor's position to avoid doing a full rotation
   * @param angleNumber input an integer based on which motor is being used.  FL = 0, FR = 1, BL = 2, BR = 3
   */
  public void rotationOverflow(CANSparkMax rotationMotor, int angleNumber){
    double currentRotation = rotationMotor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    
    double goalAngle = 0;   //used as a reference of the Rotation Motor's goal rotation in degrees.
    
    if(angleNumber == 0){
      goalAngle = fLAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 1){
      goalAngle = fRAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 2){
      goalAngle = bLAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 3){
      goalAngle = bRAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }

    //Checks to see if the goal rotation is near +-180 degrees.  If it is the goal rotation is flipped to avoid having the wheel do a 360
    if(goalAngle > 170 || goalAngle < -170){
      if(currentRotation > 0){
        goalAngle = goalAngle - 180;
      }else{
        goalAngle = goalAngle + 180;
      }
    }

    if(angleNumber == 0){
      fLAngle = goalAngle / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 1){
      fRAngle = goalAngle / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 2){
      bLAngle = goalAngle / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 3){
      bRAngle = goalAngle / Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }
    
  }


  /**
   * Attempt at fixing the inablity to go past a full rotoation with the wheel
   * @param motor the rotation moter
   * @param goalAngle the angle that the rotoation moter should be at
   * @return returns the new goal angle that will allow the rotation motors to go past a full rotation
   */
  // public double TurnForever(CANSparkMax motor, double goalAngle){

  //   double convertedTo180s = motor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation;

  //   SmartDashboard.putNumber("LookATME!", goalAngle);
    
  //   if((goalAngle < 0) && goalAngle + (currentRotation * 360) - 180 > convertedTo180s){
  //     currentRotation = currentRotation - 1;
  //   }else if((goalAngle > 0) && goalAngle + (currentRotation * 360) + 180 < convertedTo180s){
  //       currentRotation = currentRotation + 1;
  //   }
    
  //   goalAngle = (currentRotation * 360) + goalAngle;
    


  //   return goalAngle / Constants.kChassisSwerveOutputDegreeToNeoRotation;

  // }

  public double TurnForever(CANSparkMax motor, double angleNumber){

    double motorAngle = motor.getEncoder().getPosition() * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    double goalPosition = 0;
    double goalAngle = 0;

    if(angleNumber == 0){
      goalAngle = fLAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 1){
      goalAngle = fRAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 2){
      goalAngle = bLAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }else if(angleNumber == 3){
      goalAngle = bRAngle * Constants.kChassisSwerveOutputDegreeToNeoRotation;
    }

    // if()


    return goalPosition;
  }

  
  // Creates the PID controllers for all 4 rotation motors.  Should only ever be called once
  public void SetPIDController(){
    fLrotationMotor.getEncoder().setPosition(0);
    fRrotationMotor.getEncoder().setPosition(0);
    bLrotationMotor.getEncoder().setPosition(0);
    bRrotationMotor.getEncoder().setPosition(0);
    
    fLrotationMotor.getPIDController().setP(0.105);
    fLrotationMotor.getPIDController().setI(0.0);
    fLrotationMotor.getPIDController().setFF(0);
    fLrotationMotor.getPIDController().setOutputRange(-0.5, 0.5);

    fRrotationMotor.getPIDController().setP(0.105);
    fRrotationMotor.getPIDController().setI(0.0);
    fRrotationMotor.getPIDController().setFF(0);
    fRrotationMotor.getPIDController().setOutputRange(-0.5, 0.5);

    bLrotationMotor.getPIDController().setP(0.105);
    bLrotationMotor.getPIDController().setI(0.0);
    bLrotationMotor.getPIDController().setFF(0);
    bLrotationMotor.getPIDController().setOutputRange(-0.5, 0.5);

    bRrotationMotor.getPIDController().setP(0.105);
    bRrotationMotor.getPIDController().setI(0.0);
    bRrotationMotor.getPIDController().setFF(0);
    bRrotationMotor.getPIDController().setOutputRange(-0.5, 0.5);

    fLPidController.setTolerance(100, 0.0000001);
    fRPidController.setTolerance(100, 0.0000001);
    bLPidController.setTolerance(100, 0.0000001);
    bRPidController.setTolerance(100, 0.0000001);
    
    

    setPid = false; //Since the if statement that calls this function requires this boolean to be true, this prevents it from being rerun
  }

  /**
   * Makes the drive motors go in coast mode
   */
  public void wheelBrakes(){
    fLDriveMotor.setNeutralMode(NeutralMode.Coast);
    fRDriveMotor.setNeutralMode(NeutralMode.Coast);
    bLDriveMotor.setNeutralMode(NeutralMode.Coast);
    bRDriveMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * @return Average position of all 4 drive motors
   */
  public double wheelMotorCountAverage(){
    return (
    -fRDriveMotor.getSelectedSensorPosition() +
     fLDriveMotor.getSelectedSensorPosition() +
    -bRDriveMotor.getSelectedSensorPosition() +
     bLDriveMotor.getSelectedSensorPosition())/ 4;
  }

  /**
   * Zeros all 4 drive motors 
   */
  public void zeroMotors(){
    fRDriveMotor.setSelectedSensorPosition(0);
    fLDriveMotor.setSelectedSensorPosition(0);
    bRDriveMotor.setSelectedSensorPosition(0);
    bLDriveMotor.setSelectedSensorPosition(0);
  }

  
  /**
   * Get the chassis angle
   */
  public double getChassisAngle(){
    return gyro.getAngle();
  }

  /**
   * Resets the gyro
   */
  public void resetGyro(){
    gyro.reset();
  }

  public boolean checkPIDlocation(){

    if(fRPidController.atSetpoint()){
      frontLeftOnPointCount += 1;
    }else{
      frontLeftOnPointCount = 0;
    }
    SmartDashboard.putNumber("frontleftPID", frontLeftOnPointCount);
    
    if(fRPidController.atSetpoint()){
      frontRightOnPointCount += 1;
    }else{
      frontRightOnPointCount = 0;
    }
    SmartDashboard.putNumber("frontRightPID", frontRightOnPointCount);
    
    if(bLPidController.atSetpoint()){
      backLeftOnPointCount += 1;
    }else{
      backLeftOnPointCount = 0;
    }
    SmartDashboard.putNumber("backleftPID", backLeftOnPointCount);
    
    if(bRPidController.atSetpoint()){
      backRightOnPointCount += 1;
    }else{
      backRightOnPointCount = 0;
    }
    SmartDashboard.putNumber("backrightPID", backRightOnPointCount);

    if(frontLeftOnPointCount >= 5 && frontRightOnPointCount >= 5 && backLeftOnPointCount >= 5 && backRightOnPointCount >= 5){
 
      frontLeftOnPointCount = 0;
      frontRightOnPointCount = 0;
      backLeftOnPointCount = 0;
      backRightOnPointCount = 0;
     return true;
    }

    return false;
  }

  public void disablePids(){
    fLPidController.close();
    fRPidController.close();
    bLPidController.close();
    bRPidController.close();
  }

  /**
   * Calls all smartdashboard data placements
   */
  public void smartDashboardCall(){

    // SmartDashboard.putNumber("FrontRightEncoder", fRAnalogEncoder.get());
    // SmartDashboard.putNumber("FrontLeftEncoder", fLAnalogEncoder.get());
    // SmartDashboard.putNumber("BackRightEncoder", bRAnalogEncoder.get());
    // SmartDashboard.putNumber("BackLeftEncoder", bLAnalogEncoder.get());
    
 
    SmartDashboard.putNumber("Gyro position", gyro.getAngle());
     SmartDashboard.putData(gyro);
     SmartDashboard.putBoolean("Gyro connected", gyro.isConnected());
 
     SmartDashboard.putNumber("fR Rotation", fRrotationMotor.getEncoder().getPosition() / Constants.kChassisNeoMotorRotationPerWheelRotation * 360);
     SmartDashboard.putNumber("fL Rotation", fLrotationMotor.getEncoder().getPosition() / Constants.kChassisNeoMotorRotationPerWheelRotation * 360);
     SmartDashboard.putNumber("bR Rotation", bRrotationMotor.getEncoder().getPosition() / Constants.kChassisNeoMotorRotationPerWheelRotation * 360);
     SmartDashboard.putNumber("bL Rotation", bLrotationMotor.getEncoder().getPosition() / Constants.kChassisNeoMotorRotationPerWheelRotation * 360);
 
     SmartDashboard.putNumber("FrontLeftEncoder", fLAnalogEncoder.get());   
     SmartDashboard.putNumber("FrontRightEncoder", fRAnalogEncoder.get());
     SmartDashboard.putNumber("BackLeftEncoder", bLAnalogEncoder.get());
     SmartDashboard.putNumber("BackRightEncoder", bRAnalogEncoder.get());
 
     SmartDashboard.putNumber("rotations traveled", (-fRDriveMotor.getSelectedSensorPosition() + fLDriveMotor.getSelectedSensorPosition() - bRDriveMotor.getSelectedSensorPosition() + bLDriveMotor.getSelectedSensorPosition()) / 4);
     SmartDashboard.putNumber("Fr", fRDriveMotor.getSelectedSensorPosition());
     SmartDashboard.putNumber("Fl", fLDriveMotor.getSelectedSensorPosition());
     SmartDashboard.putNumber("br", bRDriveMotor.getSelectedSensorPosition());
     SmartDashboard.putNumber("bl", bLDriveMotor.getSelectedSensorPosition());
     SmartDashboard.putNumber("Wheel power", fRDriveMotor.get());

  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run.  Even when disabled.
    smartDashboardCall(); 
  }

  
}
