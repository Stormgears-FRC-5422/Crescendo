/* This file was autogenerated at {12:47:57} - do not edit it - any edits will be overwritten! */ 
package frc.robot;

import frc.utils.configfile.StormProp;

public final class Constants {
	public static final int LogitechControllerPort = StormProp.getInt("LogitechControllerPort",0);
	public static final double bumperThickness = StormProp.getNumber("bumperThickness",0.0);
	public static final String debugProperties = StormProp.getString("debugProperties","");
	public static final String robotName = StormProp.getString("robotName","");
	public static final double robotWidth = StormProp.getNumber("robotWidth",0.0);
	public static final double StickNullSize = StormProp.getNumber("StickNullSize",0.0);
	public static final double robotLength = StormProp.getNumber("robotLength",0.0);
	public static final String override = StormProp.getString("override","");
	public static final String navXConnection = StormProp.getString("navXConnection","");

	public static final class Toggles {
		public static final boolean useDrive = StormProp.getBoolean("useDrive",false);
		public static final boolean useStormNet = StormProp.getBoolean("useStormNet",false);
		public static final boolean useNavX = StormProp.getBoolean("useNavX",false);
		public static final int udpListenerPort = StormProp.getInt("udpListenerPort",0);
		public static final boolean useShooter = StormProp.getBoolean("useShooter",false);
		public static final boolean useController = StormProp.getBoolean("useController",false);
		public static final boolean useStatusLights = StormProp.getBoolean("useStatusLights",false);
		public static final boolean useVision = StormProp.getBoolean("useVision",false);
	}

	public static final class SparkMax {
		public static final int Neo550CurrentLimit = StormProp.getInt("Neo550CurrentLimit",0);
		public static final int TemperatureRampThreshold = StormProp.getInt("TemperatureRampThreshold",0);
		public static final int FreeSpeedRPM = StormProp.getInt("FreeSpeedRPM",0);
		public static final int TemperatureRampLimit = StormProp.getInt("TemperatureRampLimit",0);
		public static final int Neo550FreeSpeedRPM = StormProp.getInt("Neo550FreeSpeedRPM",0);
		public static final int CurrentLimit = StormProp.getInt("CurrentLimit",0);
	}

	public static final class Shooter {
		public static final int shooterID = StormProp.getInt("shooterID",0);
		public static final int secondShooterID = StormProp.getInt("secondShooterID",0);
		public static final int firstShooterID = StormProp.getInt("firstShooterID",0);
	}

	public static final class StormNet {
		public static final int udpListenerPort = StormProp.getInt("udpListenerPort",0);
	}

	public static final class Drive {
		public static final int frontLeftEncoderID = StormProp.getInt("frontLeftEncoderID",0);
		public static final double turnki = StormProp.getNumber("turnki",0.0);
		public static final int frontRightDriveID = StormProp.getInt("frontRightDriveID",0);
		public static final int SteerEncoderTicksPerRotation = StormProp.getInt("SteerEncoderTicksPerRotation",0);
		public static final double precisionSpeedScale = StormProp.getNumber("precisionSpeedScale",0.0);
		public static final double driveSpeedScale = StormProp.getNumber("driveSpeedScale",0.0);
		public static final int backLeftSteerID = StormProp.getInt("backLeftSteerID",0);
		public static final double turnkd = StormProp.getNumber("turnkd",0.0);
		public static final int frontLeftOffsetTicks = StormProp.getInt("frontLeftOffsetTicks",0);
		public static final int frontLeftDriveID = StormProp.getInt("frontLeftDriveID",0);
		public static final String mk4iModuleKind = StormProp.getString("mk4iModuleKind","");
		public static final double wheelRadiusMeters = StormProp.getNumber("wheelRadiusMeters",0.0);
		public static final int backRightSteerID = StormProp.getInt("backRightSteerID",0);
		public static final int backRightOffsetTicks = StormProp.getInt("backRightOffsetTicks",0);
		public static final int backLeftEncoderID = StormProp.getInt("backLeftEncoderID",0);
		public static final int backRightEncoderID = StormProp.getInt("backRightEncoderID",0);
		public static final double drivetrainWheelbaseMeters = StormProp.getNumber("drivetrainWheelbaseMeters",0.0);
		public static final double driveYkp = StormProp.getNumber("driveYkp",0.0);
		public static final int frontRightOffsetTicks = StormProp.getInt("frontRightOffsetTicks",0);
		public static final double driveXkp = StormProp.getNumber("driveXkp",0.0);
		public static final String driveType = StormProp.getString("driveType","");
		public static final int frontRightEncoderID = StormProp.getInt("frontRightEncoderID",0);
		public static final int frontRightSteerID = StormProp.getInt("frontRightSteerID",0);
		public static final int backLeftOffsetTicks = StormProp.getInt("backLeftOffsetTicks",0);
		public static final double driveXkd = StormProp.getNumber("driveXkd",0.0);
		public static final double wheelMaxRPM = StormProp.getNumber("wheelMaxRPM",0.0);
		public static final int frontLeftSteerID = StormProp.getInt("frontLeftSteerID",0);
		public static final int backRightDriveID = StormProp.getInt("backRightDriveID",0);
		public static final double driveXki = StormProp.getNumber("driveXki",0.0);
		public static final int backLeftDriveID = StormProp.getInt("backLeftDriveID",0);
		public static final double turnkp = StormProp.getNumber("turnkp",0.0);
		public static final double drivetrainTrackwidthMeters = StormProp.getNumber("drivetrainTrackwidthMeters",0.0);
		public static final double driveYki = StormProp.getNumber("driveYki",0.0);
		public static final double driveYkd = StormProp.getNumber("driveYkd",0.0);
	}

	public static final class OperatorConstants {
		public static final int kDriverControllerPort = StormProp.getInt("kDriverControllerPort",0);
	}
}

