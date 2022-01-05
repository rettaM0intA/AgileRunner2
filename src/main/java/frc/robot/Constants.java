// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


	public static final double kDirectionalDeadzone = 0.02;
    public static final double kFwd_MetersPerSecPerNom = 1;
	public static final double kStrafe_MetersPerSecPerNom = 0;
	public static final double kRotation_RadiansPerSecPerNom = 0;
	public static final double kMaxMetersPerSecWheelSpeeds = 0;
	public static final double kCntsPerDeg = 12.8;
	public static final double kMetersPerSecPerNom = 0;


	// Straight ahead offset of wheel in degrees
	public static final double kChassisFLAbsOffsetDeg = 26.316;
	public static final double kChassisFRAbsOffsetDeg = 27.72;
	public static final double kChassisBLAbsOffsetDeg = 90.36;
	public static final double kChassisBRAbsOffsetDeg = 21.42;
	public static final double kChassisSteerMotorGearRatio = 1; // 37.5

	// Theoretical based off the Max speed of the motor of 5676 RPM
	public static final double kChassisMaxMetersPerSec = 5.67;
	public static final double kChassisMaxRadiansPerSec = 18.59;

	public static final double kChassisEstimatedRotationsToInches = 1146.43;

	// The gear ratios for the serve turning motor
	public static final double kChassisNeoToGearbox = 80;
	public static final double kChassisGearboxToOutputGear = 48;
	public static final double kChassisOutputDriveGearToInputGear = 40;
	public static final double kChassisNeoMotorRotationPerWheelRotation = kChassisNeoToGearbox / kChassisGearboxToOutputGear * kChassisOutputDriveGearToInputGear;
	public static final double kChassisNeoMotorRotationtoRadians = kChassisNeoMotorRotationPerWheelRotation * Math.PI;
	
	//This is used to lower the speed of the drive motors to reasonable values
	public static final double kChassisMotorSpeedLower = 5.67;
	
	public static final double kChassisSwerveOutputDegreeToNeoRotation = 360 / kChassisNeoMotorRotationPerWheelRotation;	//5.4000054000054

	public static final double kChassisAbsoluteZerofL = 3.118;	//Absolute reads 4.118	lowered for conversion
	public static final double kChassisAbsoluteZerofR = 1.97;	//Absolute reads 2.97	lowered for conversion
	public static final double kChassisAbsoluteZerobL = 2.52;	//Absolute reads 2.52	unchanged for conversion
	public static final double kChassisAbsoluteZerobR = 0.81;	//Absolute reads 0.81	unchanged for conversion
	public static final int kChassisAbsoluteToDegreeConversion = 90;

}
