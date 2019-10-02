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
 * the server of this application listens on the port 30001.
 * 
 * */

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.StringTokenizer;

//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

 
public class ICTServer extends RoboticsAPIApplication
{
    private LBR _lbr; 
	private Controller kuka_Sunrise_Cabinet_1;
	// Frame of the EEF
	/////////////////////////////////////
	// Tool Data
	public static Tool _toolAttachedToLBR;
	/////////////////////////////////////
	// Utility classes
	public static MediaFlangeFunctions mff; // utility functions to read Write mediaflange ports 
    public static StateVariablesOfRobot svr; // state variables publisher
    private BackgroundTask dabak; // server function.
    
    private PTPmotionClass ptpm;
    //-----------------------------
    
    //PTP variables
    public static double jRelVel;
    
	/* Public variables used by the ((BackgroundTask)) class
	 * to dumb the instructions into them
	 */
    public static String daCommand="";
    public static double jpos[];
    public static double EEFpos[];
    public static double EEFposCirc1[];
    public static double EEFposCirc2[];
    public static boolean terminateFlag=false;
    //---------------------------------------------------
    int _port;
    //private static final String stopCharacter="\n"+Character.toString((char)(10));
    private static final String stopCharacter=Character.toString((char)(10));
    private static final String ack="done"+stopCharacter;
    private static final String nak="nak"+stopCharacter;
    // unused variable
    public static boolean directSmart_ServoMotionFlag=false;
    // probably unused (have to check later)
    public static double EEfServoPos[];
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);
        kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		// INitialize the variables
        jpos=new double[7];
        EEFpos=new double[7];
        EEFposCirc1=new double[7];
        EEFposCirc2=new double[7];
        EEfServoPos=new double[7];
        for(int i=0;i<7;i++)
        {
        	jpos[i]=0;
        	EEFpos[i]=0;
        	EEFposCirc1[i]=0;
            EEFposCirc2[i]=0;
            EEfServoPos[i]=0;
        }
        terminateFlag=false;
        daCommand="";
        

        // Start the server
        _port=30001;
		int timeout=60*1000;  // milli seconds
		dabak=new BackgroundTask(_port,timeout,kuka_Sunrise_Cabinet_1,_lbr);
		// Call instructors of utility classes
		mff= new MediaFlangeFunctions(kuka_Sunrise_Cabinet_1,_lbr,  dabak);
		svr=new StateVariablesOfRobot( _lbr, dabak);
		ptpm=new PTPmotionClass(_lbr,dabak,kuka_Sunrise_Cabinet_1);
			
		
		
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    private void moveToInitialPosition()
    {
        _lbr.move(
        		ptp(0., Math.PI / 180 * 20., 0., -Math.PI / 180 * 70., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.15));
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        try
        {
        	
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }
    
    

    public void moveToSomePosition()
    {
        _lbr.move(
                ptp(0., Math.PI / 180 * 20., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.));
    }

    /**
     * Main Application Routine
     */
    @Override
    public void run()
    {
        //moveToInitialPosition();
        // You shall send an acknowledgment message to the server 
        // for each
        // instruction that do not sent something back.
        // This loop will be deactivated when the direct_servo function is being used
    	while(terminateFlag==false)
        {
    		try
    		{
    			////////////////////////////////
    			// Attach default tool equal to flange
    			double[] tempx1={0,0,0,0,0,0,0,0,0,0};
    			attachTheToolToFlange("defaultToolEqualFlange",tempx1);
    			///////////////////////////////////////////////
        while(terminateFlag==false)
        {
			/////////////////////////////////////
			// Attach tool
			if(daCommand.startsWith("TFtrans"))
			{
				String st=daCommand;
				double[] tempx2=getTransformDataFromCommand(st);
				if(tempx2.length==10)
				{
					attachTheToolToFlange("KST17Tool",tempx2);
			        dabak.sendCommand(ack);
					daCommand="";
					getLogger().info("The tool is attached successfully");
					String message;
					message="X tool (mm): "+Double.toString(tempx2[0]);
					getLogger().info(message) ;		
					message="Y tool (mm): "+Double.toString(tempx2[1]);
					getLogger().info(message) ;	
					message="Z tool (mm): "+Double.toString(tempx2[2]);
					getLogger().info(message) ;	
					message="A tool (rad): "+Double.toString(tempx2[3]);
					getLogger().info(message) ;		
					message="B tool (rad): "+Double.toString(tempx2[4]);
					getLogger().info(message) ;	
					message="C tool (rad): "+Double.toString(tempx2[5]);
					getLogger().info(message) ;	
					message="Mass of tool (kg): "+Double.toString(tempx2[6]);
					getLogger().info(message) ;
					message="COMx (mm): "+Double.toString(tempx2[7]);
					getLogger().info(message) ;
					message="COMy (mm): "+Double.toString(tempx2[8]);
					getLogger().info(message) ;
					message="COMz (mm): "+Double.toString(tempx2[9]);
					getLogger().info(message) ;
				}
				else
				{
			        dabak.sendCommand(nak);
					daCommand="";
					getLogger().info("Could not attach tool to robot, App terminated");
					terminateFlag=true;
					break;
				}
			
			}
        	// Start the hand guiding mode
        	else if(daCommand.startsWith("handGuiding"))
        	{
        		//FastHandGUiding.handGUiding(_lbr, kuka_Sunrise_Cabinet_1);
        		dabak.sendCommand(ack);
        		daCommand="";
        	}
        	// Start the precise hand guiding mode
        	else if(daCommand.startsWith("preciseHandGuiding"))
        	{
        		/*
    			double weightOfTool;
    			double[] toolsCOMCoordiantes=
    					new double[3];
        		try
        		{
        			// pre-processing step
        			String tempstring=daCommand;
        			double[] toolData=new double[4];
        			toolData=PreciseHandGuidingForUpload2.
        			getWightAndCoordiantesOfCOMofTool
        			(daCommand);
        			tempstring=PreciseHandGuidingForUpload2.LBRiiwa7R800;
        			// separate data of tool to weight+COM
        			weightOfTool=toolData[0];
        			for(int kj=0;kj<3;kj++)
        			{
        				toolsCOMCoordiantes[kj]=
        						toolData[kj+1];	
        			}
        			// start the precise hand guiding
        			PreciseHandGuidingForUpload2.
        			HandGuiding(_lbr, kuka_Sunrise_Cabinet_1
        			,tempstring,weightOfTool,toolsCOMCoordiantes);
        		
        		}
        		catch (Exception e) {
        			// TODO: handle exception
        			getLogger().error(e.toString());
        		}*/
        		dabak.sendCommand(ack);
        		daCommand="";
        	} 
        	
        	// PTP instructions
        	if(daCommand.startsWith("doPTPin"))
        	{
				if(daCommand.startsWith("doPTPinJS"))
				{
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionJointSpace();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCSRelEEF"))
				{
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionCartizianSpaceRelEEf();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCSRelBase"))
				{
					dabak.sendCommand(ack);
					//getLogger().info("move end effector relative, in base frame");
					PTPmotionClass.PTPmotionCartizianSpaceRelWorld();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCSCircle1"))
				{
					
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionCartizianSpaceCircle();
					daCommand="";
				}
				else if(daCommand.startsWith("doPTPinCS"))
				{
					
					dabak.sendCommand(ack);
					PTPmotionClass.PTPmotionCartizianSpace();
					daCommand="";
				}
			}
        	// PTP motion with condition instructions
        	else if(daCommand.startsWith("doPTP!"))
        	{
        		if(daCommand.startsWith("doPTP!inJS"))
				{
        			double[] indices=new double[7];
        			double[] maxTorque=new double[7];
        			double[] minTorque=new double[7];
        			int n=StringManipulationFunctions.get_Indexes_ValBoundaries(daCommand,indices,minTorque,maxTorque);
        			dabak.sendCommand(ack);
    				daCommand="";
        			for(int i=0;i<n;i++)
        			{
        				String strInfo="[minTorque,maxTorque] for joint "+
        			Double.toString(indices[i])+" is "+Double.toString(minTorque[i])
        			+" , "+Double.toString(maxTorque[i]);
        	        	getLogger().info(strInfo);
        			}
        			PTPmotionClass.PTPmotionJointSpaceTorquesConditional( n, indices, minTorque, maxTorque);
				}
        		else if(daCommand.startsWith("doPTP!inCS"))
				{
        			double[] indices=new double[7];
        			double[] maxTorque=new double[7];
        			double[] minTorque=new double[7];
        			int n=StringManipulationFunctions.get_Indexes_ValBoundaries(daCommand,indices,minTorque,maxTorque);
        			dabak.sendCommand(ack);
    				daCommand="";
        			for(int i=0;i<n;i++)
        			{
        				String strInfo="[minTorque,maxTorque] for joint "+
        			Double.toString(indices[i])+" is "+Double.toString(minTorque[i])
        			+" , "+Double.toString(maxTorque[i]);
        	        	getLogger().info(strInfo);
        			}
        			PTPmotionClass.PTPmotionLineCartizianSpaceTorquesConditional(n, indices, minTorque, maxTorque);
				}
        		else if(daCommand.startsWith("doPTP!CSCircle1"))
				{
        			double[] indices=new double[7];
        			double[] maxTorque=new double[7];
        			double[] minTorque=new double[7];
        			int n=StringManipulationFunctions.get_Indexes_ValBoundaries(daCommand,indices,minTorque,maxTorque);
        			dabak.sendCommand(ack);
    				daCommand="";
        			for(int i=0;i<n;i++)
        			{
        				String strInfo="[minTorque,maxTorque] for joint "+
        			Double.toString(indices[i])+" is "+Double.toString(minTorque[i])
        			+" , "+Double.toString(maxTorque[i]);
        	        	getLogger().info(strInfo);
        			}
        			PTPmotionClass.PTPmotionJointSpaceTorquesConditional( n, indices, minTorque, maxTorque);
				}
        		
        	}
			else if(daCommand.startsWith("jRelVel"))
			{
				getLogger().info(daCommand);
				jRelVel=StringManipulationFunctions.jointRelativeVelocity(daCommand);
				dabak.sendCommand(ack);
				daCommand="";
			}
        	// Get torques of joints
			else if(daCommand.startsWith("Torques"))
			{
				if(daCommand.startsWith("Torques_ext_J"))
				{
					svr.sendJointsExternalTorquesToClient();
					daCommand="";
				}
				else if(daCommand.startsWith("Torques_m_J"))
				{
					svr.sendJointsMeasuredTorquesToClient();
					daCommand="";
				}		
			}
        	// Get parameters of end effector
			else if(daCommand.startsWith("Eef"))
			{			
				if(daCommand.equals("Eef_force"))
				{
					svr.sendEEFforcesToClient();
					daCommand="";
				}
				else if(daCommand.equals("Eef_moment"))
				{
					svr.sendEEFMomentsToClient();
					daCommand="";
				}	
				else if(daCommand.equals("Eef_pos"))
				{
					svr.sendEEfPositionToClient();
					daCommand="";
				}
			}
        	
			/*else if(daCommand.length()>0)
			{
				getLogger().warn("Unrecognized instruction: "+daCommand);
				try {
					Thread.sleep(100);
					// daCommand=""; // do not use
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
			}*/
        	
        	
        	}
    		}
    		catch(Exception e)
    		{
    			getLogger().error(e.toString());
    			daCommand="";
    			dabak.sendCommand(nak);
    		}
    		
        }
    }

	////////////////////////////////
	// Attach the tool to the flange functions (inertial data of the tool are also added)
	private double[] getTransformDataFromCommand(String cmd)
	{
	
		double[] val=null;
		String thestring=cmd;
		int numericDataCount=0;
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			// First token is the instruction, should be deleted
			String temp=st.nextToken();
			// Following tokens are data of transform from TCP to frame of flange
			val=new double[10];
			int j=0;
			while(st.hasMoreTokens())
			{
				String stringData=st.nextToken();
				if(j<10)
				{
				//getLogger().warn(jointString);
				val[j]=
						Double.parseDouble(stringData);
				numericDataCount=numericDataCount+1;
				}
				
				j++;
			}				
		}
		
		if (numericDataCount==10)
		{
			return val;
		}
		else
		{
			double[] ret={1,2};
			return ret;
		}
	}
	
	private void attachTheToolToFlange(String string,double[] x) {
		// TODO Apêndice de método gerado automaticamente
		String TOOL_FRAME = string;
		double[] TOOL_DATA = x; 
		double[] CENTER_OF_MASS_IN_MILLIMETER = { x[7], x[8], x[9] }; 
		LoadData _loadData = new LoadData();
		double MASS=x[6];
		_loadData.setMass(MASS);
		_loadData.setCenterOfMass(
		CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
		CENTER_OF_MASS_IN_MILLIMETER[2]);
		
		_toolAttachedToLBR = new Tool(TOOL_FRAME, _loadData);
		XyzAbcTransformation trans = XyzAbcTransformation.ofRad(
		TOOL_DATA[0], TOOL_DATA[1],
		TOOL_DATA[2], TOOL_DATA[3],
		TOOL_DATA[4], TOOL_DATA[5]);
		
		ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
		+ "(TCP)", trans);
		_toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
		// Attach tool to the robot
		_toolAttachedToLBR.attachTo(_lbr.getFlange());
	}
	
	
	double getTheDisplacment(double dj)
    {
		/*
    	double   a=0.07; 
    	double b=a*0.75; 
		double exponenet=-Math.pow(dj/b, 2);
		return Math.signum(dj)*a*(1-Math.exp(exponenet));
		*/
		return dj;
    }


    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        ICTServer app = new ICTServer();
        app.runApplication();
    }

}
