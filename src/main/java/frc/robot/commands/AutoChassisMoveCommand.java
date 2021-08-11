// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoChassisMoveCommand extends CommandBase {

  double rotate = 0;
  double fwd;
  double strafe;
  double radians;
  double speed;
  double distance = 1;
  double travelDistance = 0;
  boolean isInit = false;
  boolean isFinished = false; //boolean used to end the command.  is necessary since the isFinished function is called immediatly.

  /**
   * 
   * @param m_degree What direction you want to go in degrees
   * @param m_speed How fast you want to move in percent
   * @param m_distance How far the robot will travel in feet
   * 
   * Makes the chassis move in a desired direction at a desired speed.
   */
  public AutoChassisMoveCommand(double m_degree, double m_speed, double m_distance) {
    addRequirements(RobotContainer.m_chassisSubsystem);

    radians = ((m_degree + 90) * Math.PI / 180); //The math requires radians, so ranslate degree input to radians
    speed = m_speed;
    distance = m_distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.m_chassisSubsystem.zeroMotors();
    isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    travelDistance = RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() / Constants.kChassisEstimatedRotationsToInches;

    //The following if statements are used to keep the robot facing a single direction
    if(fwd < -0.5 || fwd > 0.5 || strafe < -0.5 || strafe > 0.5){
      if(RobotContainer.m_chassisSubsystem.gyro.getAngle() < 0){
        if(rotate < 0){
          rotate = 0;
        }else if(rotate < 0.01){
          rotate = rotate + 0.002;
        }else{
          rotate = rotate + 0.0006;
        }
      }else if(RobotContainer.m_chassisSubsystem.gyro.getAngle() > 0){
        if(rotate > 0){
          rotate = 0;
        }else if(rotate > 0.01){
          rotate = rotate - 0.002;
        }else{
          rotate = rotate - 0.0006;
        }
      }
    }

    if(fwd > -0.5 && fwd < 0.5 && strafe > -0.5 && strafe < 0.5){
      if(RobotContainer.m_chassisSubsystem.gyro.getAngle() < 0){
        if(rotate < 0){
          rotate = 0;
        }else if(rotate < 0.01){
          rotate = rotate + 0.001;
        }else{
          rotate = rotate + 0.0003;
        }
      }else if(RobotContainer.m_chassisSubsystem.gyro.getAngle() > 0){
        if(rotate > 0){
          rotate = 0;
        }else if(rotate > 0.01){
          rotate = rotate - 0.001;
        }else{
          rotate = rotate - 0.0003;
        }
      }
    }

    //Math to translate given speed and direction into usable percentages.
    fwd = (Math.sin(radians) / 100) * speed;
    strafe = (Math.cos(radians) / 100) * speed;

    //input the numbers into the drive command so the robot moves as planned
    RobotContainer.m_chassisSubsystem.driveTeleop(fwd, strafe, rotate);

    //used to track the distance traveled so the user can verify accuracy.
    SmartDashboard.putNumber("distancetraveled", travelDistance); //distancetraveled = travelDistance.  Do you understand?
    
    //these if statements decide if the command is over.  if it is, then isFinished boolean becomes true
    if(distance != 0){
      if(Math.abs(travelDistance) > Math.abs(distance)){
        RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, 0);
        isFinished = true;
      }else{
        isFinished = false;
      }
    }else{
      RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, 0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      //this is done to ensure the next command doesn't immediatly end.
      isFinished = false;
      RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, 0);
      RobotContainer.m_chassisSubsystem.zeroMotors();
      return true;
    }
    return false;
  }
}
