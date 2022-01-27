package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import frc.robot.subsystems.DrivetrainSubsystem;

/*Code outline to implement playing back a macro recorded in BTMacroRecord
*Be sure to read out of the same file created in BTMacroRecord
*BEWARE OF: setting your motors in a different order than in BTMacroRecord and changing motor values before
*time is up. Both issues are dealt with and explained below. Also only read/write from/to the motors 
*you have fully coded for, otherwise your code will cut out for no reason. 
*In main, the try/catch structure catches any IOExceptions or FileNotFoundExceptions. Necessary to play back
*the recorded routine during autonomous
*Dennis Melamed and Melanie (sorta, she slept)
*March 22nd, 2015
*/


public class MacroPlayback {
	Scanner scanner;
	long startTime;
    DrivetrainSubsystem m_drivetrain;
	boolean onTime = true;
    double nextDouble;
    double current_time;
    double current_front_left_angle;
    double current_front_left_drive;
    double current_front_right_angle;
    double current_front_right_drive;
    double current_back_left_angle;
    double current_back_left_drive;
    double current_back_right_angle;
    double current_back_right_drive;
    double next_time;
    double next_front_left_angle;
    double next_front_left_drive;
    double next_front_right_angle;
    double next_front_right_drive;
    double next_back_left_angle;
    double next_back_left_drive;
    double next_back_right_angle;
    double next_back_right_drive;
    boolean m_finished;

	public MacroPlayback( String filename, DrivetrainSubsystem drivetrain ) throws FileNotFoundException
	{
		//create a scanner to read the file created during BTMacroRecord
		//scanner is able to read out the doubles recorded into recordedAuto.csv (as of 2015)
        
        scanner = new Scanner(new File( filename ));
        
        m_drivetrain = drivetrain;
        next_time  = -99999;

		//let scanner know that the numbers are separated by a comma or a newline, as it is a .csv file
		scanner.useDelimiter(",|\\n");
		
		//lets set start time to the current time you begin autonomous
        startTime = System.currentTimeMillis();	
        
        m_finished =  false;
	}
	
	public void play()
	{
        double elapsed_time;

        elapsed_time = System.currentTimeMillis()-startTime;

		//if recordedAuto.csv has a double to read next, then read it
		if ((scanner != null) && (scanner.hasNextDouble()))
		{
            while( next_time < elapsed_time)
            {
                current_time  = next_time;
                current_front_left_drive = next_front_left_drive;
                current_front_left_angle = next_front_left_angle;
                current_front_right_drive = next_front_right_drive;
                current_front_right_angle = next_front_right_angle;
                current_back_left_drive = next_back_left_drive;
                current_back_left_angle = next_back_left_angle;
                current_back_right_drive = next_back_right_drive;
                current_back_right_angle = next_back_right_angle;

                next_time = scanner.nextDouble();
                next_front_left_drive = scanner.nextDouble();
                next_front_left_angle = scanner.nextDouble();
                next_front_right_drive = scanner.nextDouble();
                next_front_right_angle = scanner.nextDouble();
                next_back_left_drive = scanner.nextDouble();
                next_back_left_angle = scanner.nextDouble();
                next_back_right_drive = scanner.nextDouble();
                next_back_right_angle = scanner.nextDouble();
            }

            if( current_time <= elapsed_time)
            {
                m_drivetrain.frontLeftModule.set( current_front_left_angle, current_front_left_drive );
                m_drivetrain.frontRightModule.set( current_front_right_drive, current_front_right_angle );
                m_drivetrain.backLeftModule.set( current_back_left_drive, current_back_left_angle );
                m_drivetrain.backRightModule.set( current_back_right_drive, current_back_right_angle );
            }
			
		}
		//end play, there are no more values to find
		else
		{
			this.end();
			if (scanner != null) 
			{
				scanner.close();
				scanner = null;
			}
		}
		
	}
	
	//stop motors and end playing the recorded file
	public void end()
	{
        if (scanner != null)
		{
			scanner.close();
		}

        m_finished = true;
    }
    
    public boolean isFinished()
    {
        return m_finished;
    }
	
}