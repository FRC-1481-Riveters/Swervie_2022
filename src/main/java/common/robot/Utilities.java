package common.robot;

public class Utilities {
	public static double deadband(double input) {
		return deadband(input, 0.1);
	}

	public static double deadband(double input, double buffer) {
		double result=0;
		if( input > buffer )
		{
			input = input - buffer;
			result = input *  (1 - buffer);
		}
		else if( input < -buffer )
		{
			input = input + buffer;
			result = input *  (1 - buffer);
		}
		return result;
	}
}
