package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import frc.robot.subsystems.DrivetrainSubsystem;
import common.math.Vector2;

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
	private Scanner scanner;
	private long startTime;
    private DrivetrainSubsystem m_drivetrain;
	private boolean onTime = true;
    private double nextDouble;
    private double current_time;
    private double current_forward;
    private double current_strafe;
    private double current_rotation;
    private double next_time;
    private double next_forward;
    private double next_strafe;
    private double next_rotation;
    private boolean m_finished;

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
                current_forward = next_forward;
                current_strafe = next_strafe;
                current_rotation = next_rotation;

                next_time = scanner.nextDouble();
                next_forward = scanner.nextDouble();
                next_strafe = scanner.nextDouble();
                next_rotation = scanner.nextDouble();
            }

            if( current_time <= elapsed_time)
            {
                m_drivetrain.drive( new Vector2(current_forward,current_strafe), current_rotation, true );
            }
			
		}
		//end play, there are no more values to find
		else
		{
			if (scanner != null) 
			{
				scanner.close();
				scanner = null;
                m_finished = true;
			}
		}
		
	}
	
    public boolean isFinished()
    {
        return m_finished;
    }
	
}