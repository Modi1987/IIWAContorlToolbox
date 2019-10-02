package IIWAControlToolbox;

/* By Mohammad SAFEEA: Coimbra University-Portugal, 
 * Ensam University-France
 * 
 * IIWAControlToolbox
 * 
 * 02-October-2019
 * 
 * This is a multi-threaded server program that is meant to be used with both
 *    KUKA iiwa 7 R 800
 * or KUKA iiwa 14 R 820.
 * The server of this application listens on the port 30001.
 * 
 * */

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.StringTokenizer;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;

//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
 

class BackgroundTask implements Runnable {

	
	
	private Controller kuka_Sunrise_Cabinet_1;
	//private MediaFlangeIOGroup daIO;
	public boolean terminateBool;
	private LBR _lbr;
	private int _port;
	private int _timeOut;
	private ServerSocket ss;
	private Socket soc;
	
    //private static final String stopCharacter="\n"+Character.toString((char)(10));
    private static final String stopCharacter=Character.toString((char)(10));
    private static final String ack="done"+stopCharacter;
    
	
	BackgroundTask(int daport, int timeOut,Controller kuka_Sunrise_Cabinet_1,LBR _lbr )
	{
		_timeOut=timeOut;
		_port=daport;
		this.kuka_Sunrise_Cabinet_1 = kuka_Sunrise_Cabinet_1;
		this._lbr = _lbr;
		//daIO=new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		terminateBool=false;
		Thread t= new Thread(this);
		t.setDaemon(true);
		t.start();

	}

	
	public void run() {
		// TODO Auto-generated method stub
		
		try {
			ss= new ServerSocket(_port);
			try
			{
			ss.setSoTimeout(_timeOut);
			soc= ss.accept();
			}
			catch (Exception e) {
				// TODO: handle exception
				ss.close();
				ICTServer.terminateFlag=true;
				return;
			}
			Scanner scan= new Scanner(soc.getInputStream());
			// In this loop you shall check the input signal
			while((soc.isConnected()))
			{
				if(scan.hasNextLine())
				{			
					ICTServer.daCommand=scan.nextLine();
					 
					if(ICTServer.daCommand.startsWith("jf_"))
		        	{
		        		boolean tempBool=getTheJointsf(ICTServer.daCommand);
		        		ICTServer.daCommand="";
		        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		        		if(tempBool==false)
		        		{
		        			ICTServer.directSmart_ServoMotionFlag=false;
		        		}
		        		// this.sendCommand(ack); no acknowledgement in fast execution mode
		        	}
					// If the signal is equal to end, you shall turn off the server.
					else if(ICTServer.daCommand.startsWith("end"))
					{
						/* Close all existing loops:
						/  1- The BackgroundTask loop.
						 * 2- The main class, MatlabToolboxServer loops:
						 * 		a- The while loop in run, using the flag: MatlabToolboxServer.terminateFlag.
						 * 		b- The direct servo loop, using the flag: MatlabToolboxServer.directServoMotionFlag.
						*/
						ICTServer.directSmart_ServoMotionFlag=false;
						ICTServer.terminateFlag=true;
						break;						
					}
					// Put the direct_servo joint angles command in the joint variable
					else if(ICTServer.daCommand.startsWith("jp"))
		        	{
		        		updateJointsPositionArray();
		        	}
					else if(ICTServer.daCommand.startsWith("cArtixanPosition"))
		        	{
						if(ICTServer.daCommand.startsWith("cArtixanPositionCirc1"))
						{
			        		boolean tempBool=getEEFposCirc1(ICTServer.daCommand);
			        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			        		if(tempBool==false)
			        		{
			        			//MatlabToolboxServer.directServoMotionFlag=false;
			        		}
			        		this.sendCommand(ack);
			        		ICTServer.daCommand="";
						}
						else if(ICTServer.daCommand.startsWith("cArtixanPositionCirc2"))
						{
			        		boolean tempBool=getEEFposCirc2(ICTServer.daCommand);
			        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			        		if(tempBool==false)
			        		{
			        			//MatlabToolboxServer.directServoMotionFlag=false;
			        		}
			        		this.sendCommand(ack);
			        		ICTServer.daCommand="";
						}
						else
						{
			        		boolean tempBool=getEEFpos(ICTServer.daCommand);
			        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			        		if(tempBool==false)
			        		{
			        			//MatlabToolboxServer.directServoMotionFlag=false;
			        		}
			        		this.sendCommand(ack);
			        		ICTServer.daCommand="";
						}
		        	}
					
					// This insturction is used to turn_off the direct_servo controller
		        	else if(ICTServer.daCommand.startsWith("stopDirectServoJoints"))
		        	{
		        		ICTServer.directSmart_ServoMotionFlag=false;
		        		this.sendCommand(ack);
		        		ICTServer.daCommand="";
		        	}
					else if(ICTServer.daCommand.startsWith("DcSe"))
		        	{
		        		updateEEFPositionArray();
		        	}
		        	else
		        	{
		        		// inquiring data from server
		        		dataAqcuisitionRequest();
		        	}
					
				}				
			}
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		
		try {
			ICTServer.terminateFlag=true;
			soc.close();
			ss.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

	
	private void updateEEFPositionArray()
	{
		//////////////////////////////////////////////////
		//Start of server update functions
		/////////////////////////////////////////////////////						
		
		if(ICTServer.daCommand.startsWith("DcSeCar_"))
		{
			boolean tempBool=getThePositions(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
			ICTServer.directSmart_ServoMotionFlag=false;
			}
			// this.sendCommand(ack);
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("DcSeCarExT_"))
		{
			boolean tempBool=getThePositions(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
			ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendJointsExternalTorquesToClient();
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("DcSeCarMT_"))
		{
			boolean tempBool=getThePositions(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
			ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendJointsMeasuredTorquesToClient();
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("DcSeCarEEfP_"))
		{
			boolean tempBool=getThePositions(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
			ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendEEfPositionToClient(); // "modified 12th-May-2019"
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("DcSeCarJP_"))
		{
			boolean tempBool=getThePositions(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
			ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendJointsPositionsToClient();
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("DcSeCarEEfFrelEEF_")) 
		{
			// "newly-added 12th-May-2019"
			// Update Cartesian position of EEF and returns force force feedback described in EEF reference frame	
			boolean tempBool=getThePositions(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
				ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendEEFforcesToClient();
			ICTServer.daCommand="";
		}
		//////////////////////////////////////////////////
		//End of Servo joints update functions
		//////////////////////////////////////////////////////
	}
	
	private boolean getThePositions(String thestring) {
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
				int j=0;
				while(st.hasMoreTokens())
				{
					if(j<6)
					{
						//getLogger().warn(jointString);
						try
						{
							ICTServer.EEfServoPos[j]=Double.parseDouble(st.nextToken());
						}
						catch(Exception e)
						{
							return false;
						}						
					}					
					j++;
				}
				ICTServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
	}


	
	private void updateJointsPositionArray()
	{
		//////////////////////////////////////////////////
		//Start of server update functions
		/////////////////////////////////////////////////////								
		if(ICTServer.daCommand.startsWith("jp_"))
		{
			boolean tempBool=getTheJoints(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
				ICTServer.directSmart_ServoMotionFlag=false;
			}
			this.sendCommand(ack);
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("jpExT_"))
		{
			boolean tempBool=getTheJoints(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
				ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendJointsExternalTorquesToClient();
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("jpMT_"))
		{
			boolean tempBool=getTheJoints(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
				ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendJointsMeasuredTorquesToClient();
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("jpEEfP_"))
		{
			boolean tempBool=getTheJoints(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
				ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendEEfPositionToClient(); // "modified 12th-May-2019"
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("jpJP_"))
		{
			boolean tempBool=getTheJoints(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
				ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendJointsPositionsToClient();
			ICTServer.daCommand="";
		}
		else if(ICTServer.daCommand.startsWith("jpEEfFrelEEF_"))
		{
			// "newly-added 12th-May-2019"
			boolean tempBool=getTheJoints(ICTServer.daCommand);
			// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
			if(tempBool==false)
			{
				ICTServer.directSmart_ServoMotionFlag=false;
			}
			ICTServer.svr.sendEEFforcesToClient();
			ICTServer.daCommand="";
		}

		
		//////////////////////////////////////////////////
		//End of Servo joints update functions
		//////////////////////////////////////////////////////
	}
	
	// respond to a data Acquisition Request
	private void dataAqcuisitionRequest()
	{
		// Inquiring data from server
    	if(ICTServer.daCommand.startsWith("getJointsPositions"))
    	{
    		ICTServer.svr.sendJointsPositionsToClient();
    		ICTServer.daCommand="";
    	}        	
    	// Write output of Mediaflange
    	else if(ICTServer.daCommand.startsWith("blueOn"))
    	{
    		ICTServer.mff.blueOn();
    		sendCommand(ack);
    		ICTServer.daCommand="";
    	}
    	else if(ICTServer.daCommand.startsWith("blueOff"))
    	{
    		ICTServer.mff.blueOff();
    		sendCommand(ack);
    		ICTServer.daCommand="";
    	}
    	else if(ICTServer.daCommand.startsWith("pin"))
    	{
        	if(ICTServer.daCommand.startsWith("pin1on"))
			{
        		ICTServer.mff.pin1On();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("pin1off"))
			{
				ICTServer.mff.pin1Off();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("pin11on"))
			{
				ICTServer.mff.pin11On();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("pin11off"))
			{
				ICTServer.mff.pin11Off();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("pin2on"))
			{
				ICTServer.mff.pin2On();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("pin2off"))
			{
				ICTServer.mff.pin2Off();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("pin12on"))
			{
				ICTServer.mff.pin12On();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("pin12off"))
			{
				ICTServer.mff.pin12Off();
				sendCommand(ack);
				ICTServer.daCommand="";
			}
    	}
    	// Read input of Mediaflange
    	if(ICTServer.daCommand.startsWith("getPin"))
    	{
			if(ICTServer.daCommand.startsWith("getPin10"))
			{
				ICTServer.mff.getPin10();
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("getPin16"))
			{
				ICTServer.mff.getPin16();
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("getPin13"))
			{
				ICTServer.mff.getPin13();
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("getPin3"))
			{
				ICTServer.mff.getPin3();
				ICTServer.daCommand="";
			}
			else if(ICTServer.daCommand.startsWith("getPin4"))
			{
				ICTServer.mff.getPin4();
				ICTServer.daCommand="";
			}
    	}
	}
	
    
	/* The following function is used to extract the 
	 joint angles from the command
	 */
	private boolean getTheJoints(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("jp"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<7)
					{
						//getLogger().warn(jointString);
						ICTServer.jpos[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ICTServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}

	/* The following function is used to extract the 
	 joint angles from the command
	 */
	private boolean getTheJointsf(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("jf"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<7)
					{
						//getLogger().warn(jointString);
						ICTServer.jpos[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}
	
	private boolean getEEFpos(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("cArtixanPosition"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<6)
					{
						//getLogger().warn(jointString);
						ICTServer.EEFpos[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ICTServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}

	
	private boolean getEEFposCirc2(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("cArtixanPosition"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<6)
					{
						//getLogger().warn(jointString);
						ICTServer.EEFposCirc2[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ICTServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}
	
	
	private boolean getEEFposCirc1(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("cArtixanPosition"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<6)
					{
						//getLogger().warn(jointString);
						ICTServer.EEFposCirc1[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ICTServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}
	/* The following function is used to sent a string message
	 * to the server
	 */
	public boolean sendCommand(String s)
	{
		if(ss==null)return false;
		if(soc==null)return false;
		if(soc.isClosed())return false;
		
		try {
			soc.getOutputStream().write(s.getBytes("US-ASCII"));
			return true;
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return false;
		
	}
	

	
}
	
