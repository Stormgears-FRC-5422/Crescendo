/* This file was autogenerated at {00:12:36} - do not edit it - any edits will be overwritten! */ 
package frc.robot;

import frc.utils.configfile.StormProp;

public final class Constants {
	public static final int LogitechControllerPort = StormProp.getInt("general","LogitechControllerPort",0);
	public static final double bumperThickness = StormProp.getNumber("general","bumperThickness",0.0);
	public static final String debugProperties = StormProp.getString("general","debugProperties","");
	public static final String robotName = StormProp.getString("general","robotName","");
	public static final double robotWidth = StormProp.getNumber("general","robotWidth",0.0);
	public static final double StickNullSize = StormProp.getNumber("general","StickNullSize",0.0);
	public static final double robotLength = StormProp.getNumber("general","robotLength",0.0);
	public static final String override = StormProp.getString("general","override","");
	public static final String navXConnection = StormProp.getString("general","navXConnection","");

	public static final class Toggles {
		public static final boolean useDrive = StormProp.getBoolean("toggles","useDrive",false);
		public static final boolean useStormNet = StormProp.getBoolean("toggles","useStormNet",false);
		public static final boolean useNavX = StormProp.getBoolean("toggles","useNavX",false);
		public static final int udpListenerPort = StormProp.getInt("toggles","udpListenerPort",0);
		public static final boolean useController = StormProp.getBoolean("toggles","useController",false);
		public static final boolean useStatusLights = StormProp.getBoolean("toggles","useStatusLights",false);
		public static final boolean useVision = StormProp.getBoolean("toggles","useVision",false);
	}

	public static final class SparkMax {
		public static final int Neo550CurrentLimit = StormProp.getInt("sparkMax","Neo550CurrentLimit",0);
		public static final int TemperatureRampThreshold = StormProp.getInt("sparkMax","TemperatureRampThreshold",0);
		public static final int FreeSpeedRPM = StormProp.getInt("sparkMax","FreeSpeedRPM",0);
		public static final int TemperatureRampLimit = StormProp.getInt("sparkMax","TemperatureRampLimit",0);
		public static final int Neo550FreeSpeedRPM = StormProp.getInt("sparkMax","Neo550FreeSpeedRPM",0);
		public static final int CurrentLimit = StormProp.getInt("sparkMax","CurrentLimit",0);
	}

	public static final class Shooter {
		public static final int shooterID = StormProp.getInt("shooter","shooterID",0);
		public static final boolean hack = StormProp.getBoolean("shooter","hack",false);
	}

	public static final class Drive {
		public static final int frontLeftEncoderID = StormProp.getInt("drive","frontLeftEncoderID",0);
		public static final double turnki = StormProp.getNumber("drive","turnki",0.0);
		public static final int frontRightDriveID = StormProp.getInt("drive","frontRightDriveID",0);
		public static final double precisionSpeedScale = StormProp.getNumber("drive","precisionSpeedScale",0.0);
		public static final int steerEncoderTicksPerRotation = StormProp.getInt("drive","steerEncoderTicksPerRotation",0);
		public static final double driveSpeedScale = StormProp.getNumber("drive","driveSpeedScale",0.0);
		public static final int backLeftSteerID = StormProp.getInt("drive","backLeftSteerID",0);
		public static final double turnkd = StormProp.getNumber("drive","turnkd",0.0);
		public static final int frontLeftOffsetTicks = StormProp.getInt("drive","frontLeftOffsetTicks",0);
		public static final int frontLeftDriveID = StormProp.getInt("drive","frontLeftDriveID",0);
		public static final String mk4iModuleKind = StormProp.getString("drive","mk4iModuleKind","");
		public static final double wheelRadiusMeters = StormProp.getNumber("drive","wheelRadiusMeters",0.0);
		public static final int backRightSteerID = StormProp.getInt("drive","backRightSteerID",0);
		public static final int backRightOffsetTicks = StormProp.getInt("drive","backRightOffsetTicks",0);
		public static final int backLeftEncoderID = StormProp.getInt("drive","backLeftEncoderID",0);
		public static final int backRightEncoderID = StormProp.getInt("drive","backRightEncoderID",0);
		public static final double drivetrainWheelbaseMeters = StormProp.getNumber("drive","drivetrainWheelbaseMeters",0.0);
		public static final double driveYkp = StormProp.getNumber("drive","driveYkp",0.0);
		public static final int frontRightOffsetTicks = StormProp.getInt("drive","frontRightOffsetTicks",0);
		public static final double driveXkp = StormProp.getNumber("drive","driveXkp",0.0);
		public static final String driveType = StormProp.getString("drive","driveType","");
		public static final int frontRightEncoderID = StormProp.getInt("drive","frontRightEncoderID",0);
		public static final int frontRightSteerID = StormProp.getInt("drive","frontRightSteerID",0);
		public static final int backLeftOffsetTicks = StormProp.getInt("drive","backLeftOffsetTicks",0);
		public static final double maxMotorVoltage = StormProp.getNumber("drive","maxMotorVoltage",0.0);
		public static final double driveXkd = StormProp.getNumber("drive","driveXkd",0.0);
		public static final double wheelMaxRPM = StormProp.getNumber("drive","wheelMaxRPM",0.0);
		public static final int frontLeftSteerID = StormProp.getInt("drive","frontLeftSteerID",0);
		public static final int backRightDriveID = StormProp.getInt("drive","backRightDriveID",0);
		public static final double driveXki = StormProp.getNumber("drive","driveXki",0.0);
		public static final int backLeftDriveID = StormProp.getInt("drive","backLeftDriveID",0);
		public static final double turnkp = StormProp.getNumber("drive","turnkp",0.0);
		public static final double drivetrainTrackwidthMeters = StormProp.getNumber("drive","drivetrainTrackwidthMeters",0.0);
		public static final double driveYki = StormProp.getNumber("drive","driveYki",0.0);
		public static final double driveYkd = StormProp.getNumber("drive","driveYkd",0.0);
	}

	public static final class OperatorConstants {
		public static final int kDriverControllerPort = StormProp.getInt("operatorConstants","kDriverControllerPort",0);
	}
}

