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
  boolean isFinished = false;

  /** Creates a new AutoChassisCommand. */
  public AutoChassisMoveCommand(double m_degree, double m_speed, double m_distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);
    radians = ((m_degree + 90) * Math.PI / 180);
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

    fwd = (Math.sin(radians) / 100) * speed;
    strafe = (Math.cos(radians) / 100) * speed;

    RobotContainer.m_chassisSubsystem.driveTeleop(fwd, strafe, rotate);

    SmartDashboard.putNumber("distancetraveld", travelDistance);
    
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
    // if(isInit){
      if(isFinished){
        isFinished = false;
        RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, 0);
        RobotContainer.m_chassisSubsystem.zeroMotors();
        return true;
      }
    // }
    return false;
  }
}
