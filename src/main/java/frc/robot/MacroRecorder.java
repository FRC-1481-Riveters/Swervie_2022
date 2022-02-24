package frc.robot;

// lifted from https://github.com/DennisMelamed/FRC-Play-Record-Macro/blob/master/FRC2220-Play-Record-Macro-DM//src/BTMacroRecord.java

import java.io.FileWriter;
import java.io.IOException;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.lang.String;

/*
*This macro records all the movements you make in teleop and saves them to the file you specify.
*make sure you record every variable you need, if you dont record the value from a motor or a solenoid,
*you won't be able to play it back. It records it in "frames" that contain a value for each output 
* you want to use during teleop
*BE AWARE: write into the same file as you do in the Play macro
*BE AWARE: Only write/read the motors/other things that you actually have fully created in 
*your code. Otherwise you'll lose robot code randomly with no reason
*In main, the try/catch structure catches any IOExceptions or FileNotFoundExceptions. Necessary to play back
*the recorded routine during autonomous
*Dennis Melamed, Melanie Quick
*22 March, 2015
*/


public class MacroRecorder {
	
	//this object writes values into the file we specify
	private FileWriter writer;
	
	private long startTime;
	private DrivetrainSubsystem m_drivetrain;

	
	public MacroRecorder( String filename, DrivetrainSubsystem drivetrain ) throws IOException
	{
			m_drivetrain = drivetrain;

			//record the time we started recording
			startTime = System.currentTimeMillis();
			
			//put the filesystem location you are supposed to write to as a string 
			//as the argument in this method, as of 2015 it is /home/lvuser/recordedAuto.csv
			writer = new FileWriter( filename );
	}
	

	public void record() throws IOException
	{
		double convert_voltage;
		
		convert_voltage = m_drivetrain.MAX_VOLTAGE * m_drivetrain.MAX_VELOCITY_INCHES_PER_SECOND;

		if(writer != null)
		{
			//start each "frame" with the elapsed time since we started recording
			writer.append("" + (System.currentTimeMillis()-startTime));
			
			//in this chunk, use writer.append to add each type of data you want to record to the frame
			//the 2015 robot used the following motors during auto
			
			//drive motors
			writer.append( 
				"," + m_drivetrain.m_forward +
				"," + m_drivetrain.m_strafe +
				"," + m_drivetrain.m_rotation
			);
			
			/*
			* THE LAST ENTRY OF THINGS YOU RECORD NEEDS TO HAVE A DELIMITER CONCATENATED TO 
			* THE STRING AT THE END. OTHERWISE GIVES NOSUCHELEMENTEXCEPTION
			* CAREFUL. REMEMBER TO APPEND THE DELIMITER
			*/
			writer.append( "\n");
		}
	}
	
	
	//this method closes the writer and makes sure that all the data you recorded makes it into the file
	public void end() throws IOException
	{
		if(writer !=null)
		{
			writer.flush();
			writer.close();
		}
	}

	private String safe_write( double velocity, double convert_voltage )
	{
		String s;

		s = String.valueOf( velocity / convert_voltage );

		return s;
	}
}
