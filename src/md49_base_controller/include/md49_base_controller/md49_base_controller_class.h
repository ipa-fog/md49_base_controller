#ifndef MD49_BASE_CONTROLLER_CLASS_H_
#define MD49_BASE_CONTROLLER_CLASS_H_

#define TIMEOUT 1000                                                                            /**<  timeout for reading serialport in ms */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <math.h>
#define wheel_diameter 0.125
#define wheel_baseline 0.515

class BaseController
{
public:

    ros::NodeHandle n;

    /**
     * @brief Constructor for class BaseController
     */
    BaseController()
      {
        // Topics to publish
        md49_encoders_pub = n.advertise<md49_messages::md49_encoders>("md49_encoders",1);
        md49_odom_pub = n.advertise<nav_msgs::Odometry>("odom",1);
        md49_data_pub = n.advertise<md49_messages::md49_data>("md49_data",1);
        // Topic to subscribe
        sub_cmd_vel = n.subscribe("/cmd_vel", 1, &BaseController::cmd_vel_callback, this);
        // Read initial parameters from parameter service
        n.param<std::string>("serialport/name", serialport, "/dev/ttyS0");                      // Get serialportname from ROS Parameter sevice, default is ttyS0 (pcDuinos GPIO UART)
        n.param("serialport/bps", serialport_bps, 38400);                                       // Get serialport bps from ROS Parameter sevice, default is 38400Bps
        n.param("md49/mode", initial_md49_mode, 1);                                             // Get MD49 Mode from ROS Parameter sevice, default is Mode=0
        n.param("md49/acceleration", initial_md49_acceleration, 5);                             // Get MD49 Acceleration from ROS Parameter sevice, default is Acceleration=0
        n.param("md49/regulator", initial_md49_regulator, true);                                // Get MD49 Regulator from ROS Parameter sevice, default is Regulator=ON
        n.param("md49/timeout", initial_md49_timeout, true);                                    // Get MD49 Timeout from ROS Parameter sevice, default is Timeout=ON
        n.param("md49/speed_l", requested_speed_l, 128);                                        // Get MD49 speed_l from ROS Parameter sevice, default is speed_l=128
        n.param("md49/speed_r",  requested_speed_r, 128);                                       // Get MD49 speed_r from ROS Parameter sevice, default is speed_r=128
        actual_speed_l=requested_speed_l;
        actual_speed_r=requested_speed_r;
      }

    /**
     * @brief This is the callback-function for topic /cmd_vel
     * @param vel_cmd
     */
    void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){
	// translation in m/s
	// angular in radian/s
	twist_time = ros::Time::now();
   	requested_speed_l = 128 + (int)(wheel_diameter * (1 * vel_cmd.linear.x - 0.5 * wheel_baseline * vel_cmd.angular.z) * (1380.0)); // 1380 calibration factor
      	requested_speed_r = 128 + (int)(wheel_diameter * (1 * vel_cmd.linear.x + 0.5 * wheel_baseline * vel_cmd.angular.z) * (1380.0));

    }


    /**
     * @brief This function publish odometry based on encoder values
     */
    void publish_odom()
    {
	// fist get encoder values which are the base of wheel odometry
        //get_encoders();

	// set current time
	current_time = ros::Time::now();

	double dl, dr;	
	
	// 1024 encoder steps = 1 turn
	l1_ = md49_encoders.encoder_l * 0.001024 * M_PI * wheel_diameter;
	r1_ = md49_encoders.encoder_r * 0.001024 * M_PI * wheel_diameter;

	dl = l1_ - l0_;
	dr = r1_ - r0_;

	// moved between last and current loop/state
	double dth, dx, dy;
	dth = (dr - dl) / wheel_baseline;
	dx = 0.5 * (dl + dr) * cos(dth + th0_);
	dy = 0.5 * (dl + dr) * sin(dth + th0_);

	// calculate new position
	th1_ = th0_ + dth;
	x1_ = x0_ + dx;	
	y1_ = y0_ + dy;


	// calculate velocity
	double dt = (current_time - last_time).toSec();

	vx_ = dx / dt;
	vy_ = dy / dt;
	vth_ = dth / dt;

	if(dl > 2.0 || dr > 2.0) // sometimes the encoder bits return wrong values e.g. at EMCY
	{
		
		ROS_WARN("huge jump detected, ignore last delta");
		th1_ = th0_;
		x1_ = x0_;	
		y1_ = y0_;
		vx_ = 0.0;
		vy_ = 0.0;
		vth_ = 0.0;
		l1_ = l0_;
		r1_ = r0_;
	}

	geometry_msgs::Quaternion odom_quat;
	odom_quat = tf::createQuaternionMsgFromYaw(th1_);

	// create new tf
	static tf::TransformBroadcaster odom_broadcaster;
    	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom_combined";
	odom_trans.child_frame_id = "base_footprint";
	odom_trans.transform.translation.x = x1_;
	odom_trans.transform.translation.y = y1_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);	

	// create odometry msgs
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom_combined";
	odom.child_frame_id = "base_footprint";
	odom.pose.pose.position.x = x1_;
	odom.pose.pose.position.y = y1_;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	odom.twist.twist.linear.x = vx_;
	odom.twist.twist.linear.y = vy_;
	odom.twist.twist.angular.z = vth_;

	 
	md49_odom_pub.publish(odom);
	odom_broadcaster.sendTransform(odom_trans);
	

	// set last time
	last_time = ros::Time::now();

	// set last to new positon for next loop
	th0_ = th1_;
	x0_ = x1_;
	y0_ = y1_;

	l0_ = l1_;
	r0_ = r1_;

	double dt_twist = (ros::Time::now() - twist_time).toSec();

	// stop if we get no new velocity (timeout)
	if(dt_twist > 0.5) 
	{
		requested_speed_l = 128;
	      	requested_speed_r = 128;
	}
	
    }


    /**
     * @brief This function opens serial port MD49 is connected to
     */
    void open_serialport()
    {
        try{ device.open(serialport.c_str(), serialport_bps); }
        catch(cereal::Exception& e)
        {
            ROS_FATAL("base_controller: Failed to open serialport %s!",serialport.c_str());
            ROS_BREAK();
        }
        ROS_INFO("base_controller: Opened Serialport at %s with %i bps.",serialport.c_str(),serialport_bps);
    }
    /**
     * @brief This function reads encodervalues from MD49 and publishes them as topic /md49_encoders
     */
    void publish_encoders()
    {
	get_encoders();
        md49_encoders_pub.publish(md49_encoders);
    }

    /**
     * @brief This function reads data and parameters from MD49 and publishes them as topic /md49_data
     */
    void publish_md49_data()
    {
        md49_data.speed_l=get_speed_l();
        md49_data.speed_r=get_speed_r();
        md49_data.volts=get_volts();
        md49_data.current_l=get_current_l();
        md49_data.current_r=get_current_r();
        md49_data.acceleration=get_acceleration();
        md49_data.mode=get_mode();
        md49_data.error=get_error();
        md49_data_pub.publish(md49_data);
    }

    /**
     * @brief This function sets parameters for MD49 as read from config file or as set as defaults
     * @param speed_l
     * @param speed_r
     * @param mode
     * @param acceleration
     * @param timeout
     * @param regulator
     */
    void init_md49(int speed_l, int speed_r, int mode, int acceleration, bool timeout, bool regulator)
    {
	reset_encoders();
        set_speed(speed_l,speed_r);
        set_mode(mode);
        set_acceleration(acceleration);
        if (timeout==true)
        {
            enable_timeout();
        }
        else if (timeout==false)
        {
            disable_timeout();
        }
        if (regulator==true)
        {
            enable_regulator();
        }
        else if (regulator==false)
        {
            disable_regulator();
        }
    }
    /**
     * @brief This function sets speed for left and right drive on MD49
     * @param speed_l
     * @param speed_r
     */
    void set_speed(int speed_l, int speed_r)
    {
        // set and send serial command for speed_l
        const char md49_set_speed_l[]={0x00,0x31,speed_l};
        device.write(md49_set_speed_l,3);
        // set and send serial command for speed_r
        const char md49_set_speed_r[]={0x00,0x32,speed_r};
        device.write(md49_set_speed_r,3);

    }
    /**
     * @brief This function sets mode on MD49
     * @param mode
     */
    void set_mode(int mode)
    {
        const char md49_set_mode[]={0x00,0x34,mode};
        device.write(md49_set_mode,3);
        ROS_INFO("base_controller: Set mode=%i on MD49", mode);
    }
    /**
     * @brief This function sets acceleration on MD49
     * @param acceleration
     */
    void set_acceleration(int acceleration)
    {
        const char md49_set_acceleration[]={0x00,0x33,acceleration};
        device.write(md49_set_acceleration,3);
        ROS_INFO("base_controller: Set acceleration=%i on MD49", acceleration);
    }
    /**
     * @brief This function enables timeout on MD49
     */
    void enable_timeout(void){
        const char md49_enable_timeout[] = {0x00,0x39};             // put together command to enable md49 timeout
        device.write(md49_enable_timeout,2);
        md49_data.timeout=1;
        ROS_INFO("base_controller: Enabled timeout on MD49");
    }
    /**
     * @brief This function enables regulator on MD49
     */
    void enable_regulator(void){
        const char md49_enable_regulator[] = {0x00,0x37};           // put together command to enable md49 regulator
        device.write(md49_enable_regulator,2);
        md49_data.regulator=1;
        ROS_INFO("base_controller: Enabled regulator on MD49");
    }
    /**
     * @brief This function disables Timeout on MD49
     */
    void disable_timeout(void){
        const char md49_disable_timeout[] = {0x00,0x38};            // put together command to enable md49 regulator
        device.write(md49_disable_timeout,2);
        md49_data.timeout=0;
        ROS_INFO("base_controller: Disabled timeout on MD49");
    }
    /**
     * @brief This function disables regulator on MD49
     */
    void disable_regulator(void){
        const char md49_disable_regulator[] = {0x00,0x36};          // put together command to enable md49 timeout
        device.write(md49_disable_regulator,2);
        md49_data.regulator=0;
        ROS_INFO("base_controller: Disabled regulator on MD49");
    }
    /**
     * @brief This function reads the acceleration that is set from MD49
     * @return
     */
    int get_acceleration(){
        const char md49_get_acceleration[] = {0x00,0x2A};           // put together command to read md49 set acceleration
        device.write(md49_get_acceleration,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 acceleration!");
        }
        //ROS_INFO("base_controller: MD49 Acceleration= %i", reply[0]);
        return reply[0];
    }
    /**
     * @brief This function reads the mode that is set from MD49
     * @return
     */
    int get_mode(){
        const char md49_get_mode[] = {0x00,0x2B};                   // put together command to read md49 set acceleration
        device.write(md49_get_mode,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 Mode!");
        }
        //ROS_INFO("base_controller: MD49 Mode= %i", reply[0]);
        return reply[0];
    }
    /**
     * @brief This function reads left drives speed from MD49
     * @return
     */
    int get_speed_l(){
        const char md49_get_speed_l[] = {0x00,0x21};                // put together command to read md49 set acceleration
        device.write(md49_get_speed_l,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 speed_l!");
        }
        return reply[0];
        //ROS_INFO("base_controller: MD49 speed_l= %i, speed_r= %i", speed_l,speed_r);
    }
    /**
     * @brief This function reads right drives speed from MD49
     * @return
     */
    int get_speed_r(){
        const char md49_get_speed_r[] = {0x00,0x22};                // put together command to read md49 set acceleration
        device.write(md49_get_speed_r,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, acceleration
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 speed_r!");
        }
        return reply[0];
        //ROS_INFO("base_controller: MD49 speed_l= %i, speed_r= %i", speed_l,speed_r);
    }
    /**
     * @brief This function reads encodervalues from MD49
     */
    void get_encoders(void){
        const char md49_get_encoders[] = {0x00,0x25};               // put together command to read md49 encoders
        device.write(md49_get_encoders,2);
        // ******************************************************
        // * Get the reply, the last value is the timeout in ms *
        // ******************************************************
        try{ device.read(reply, 8, TIMEOUT); }
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 encodervalues!");
        }


        // ***************************************************
        // * Set all values of custom message /md49_encoders *
        // ***************************************************
	
	uint8_t ints[8];
	for(int i=0; i<8; i++)
	{
		ints[i] = (uint8_t) reply[i];
	}
	md49_encoders.encoder_l = ((ints[0] << 24) + (ints[1] << 16) + (ints[2] << 8) + (ints[3] << 0));
	md49_encoders.encoder_r = ((ints[4] << 24) + (ints[5] << 16) + (ints[6] << 8) + (ints[7] << 0));

        md49_encoders.encoderbyte1l=reply[0];
        md49_encoders.encoderbyte2l=reply[1];
        md49_encoders.encoderbyte3l=reply[2];
        md49_encoders.encoderbyte4l=reply[3];
        md49_encoders.encoderbyte1r=reply[4];
        md49_encoders.encoderbyte2r=reply[5];
        md49_encoders.encoderbyte3r=reply[6];
        md49_encoders.encoderbyte4r=reply[7];


	publish_odom();


    }
    /**
     * @brief This function reads supply voltage from MD49
     * @return
     */
    int get_volts(){
        const char md49_get_volts[] = {0x00,0x26};                  // put together command to read md49 volts
        device.write(md49_get_volts,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, volts
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 volts!");
        }
        //ROS_INFO("Got this reply (volts): %i", reply[0]);
        return reply[0];
    }
    /**
     * @brief This function reads left drives current from MD49
     * @return
     */
    int get_current_l(){
        const char md49_get_current_l[] = {0x00,0x27};              // put together command to read md49 current_l
        device.write(md49_get_current_l,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, current_l
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 current_l!");
        }
        //ROS_INFO("Got this reply (current_l): %i", reply[0]);
        return reply[0];
    }
    /**
     * @brief This function reads right drives current from MD49
     * @return
     */
    int get_current_r(){
        const char md49_get_current_r[] = {0x00,0x28};              // put together command to read md49 current_r
        device.write(md49_get_current_r,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, current_r
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 current_r!");
        }
        //ROS_INFO("Got this reply (current_r): %i", reply[0]);
        return reply[0];
    }
    /**
     * @brief This function reads error code from MD49
     * @return
     */
    int get_error(){
        // ************************
        const char md49_get_error[] = {0x00,0x2D};                  // put together command to read md49 error
        device.write(md49_get_error,2);
        try{ device.read(reply, 1, TIMEOUT); }                      // get answer, error
        catch(cereal::TimeoutException& e){
            ROS_ERROR("base_controller: Timeout reading MD49 errorbyte!");
        }
        //ROS_INFO("Got this reply (error): %i", reply[0]);
        return reply[0];
    }
    /**
     * @brief This function resets encoders on MD49
     */
    void reset_encoders(void){
        const char md49_reset_encoders[] = {0x00,0x35};             // put together command to reset md49 encoders
        device.write(md49_reset_encoders,2);
        ROS_INFO("base_controller: Reset encoders on MD49");
    }

    /**
     * @brief get_requested_speed_l
     * @return
     */
    int get_requested_speed_l()
    {
        return requested_speed_l;
    }
    /**
     * @brief get_requested_speed_r
     * @return
     */
    int get_requested_speed_r()
    {
        return requested_speed_r;
    }
    /**
     * @brief set_requested_speed_l
     * @param speed_l
     */
    void set_requested_speed_l(int speed_l)
    {
        requested_speed_l=speed_l;
    }
    /**
     * @brief set_requested_speed_r
     * @param speed_r
     */
    void set_requested_speed_r(int speed_r)
    {
        requested_speed_r=speed_r;
    }
    /**
     * @brief get_actual_speed_l
     * @return
     */
    int get_actual_speed_l()
    {
        return actual_speed_l;
    }
    /**
     * @brief get_actual_speed_r
     * @return
     */
    int get_actual_speed_r()
    {
        return actual_speed_r;
    }
    /**
     * @brief set_actual_speed_l
     * @param speed_l
     */
    void set_actual_speed_l(int speed_l)
    {
        actual_speed_l=speed_l;
    }
    /**
     * @brief set_actual_speed_r
     * @param speed_r
     */
    void set_actual_speed_r(int speed_r)
    {
        actual_speed_r=speed_r;
    }
    /**
     * @brief get_initial_md49_mode
     * @return
     */
    int get_initial_md49_mode()
    {
        return initial_md49_mode;
    }/**
     * @brief get_initial_md49_acceleration
     * @return
     */
    int get_initial_md49_acceleration()
    {
        return initial_md49_acceleration;
    }
    /**
     * @brief get_initial_md49_timeout
     * @return
     */
    int get_initial_md49_timeout()
    {
        return initial_md49_timeout;
    }
    /**
     * @brief get_initial_md49_regulator
     * @return
     */
    int get_initial_md49_regulator()
    {
        return initial_md49_regulator;
    }

private:

	cereal::CerealPort device;                                                                      /**<  serialport */
	char reply[8];
	int requested_speed_l, requested_speed_r;                                                       /**<  requested speed_l and speed_r for MD49 */
	int actual_speed_l, actual_speed_r;                                                             /**<  buffers actual set speed_l and speed_r */
	int initial_md49_mode;                                                                          /**<  MD49 Mode, is read from parameters server */
	int initial_md49_acceleration;                                                                  /**<  MD49 Acceleration,  is read from parameters server */
	bool initial_md49_timeout;                                                                      /**<  MD49 Timeout-Mode, is read from parameters server */
	bool initial_md49_regulator;                                                                    /**<  MD40 Regulator-Mode , is read from parameters server */
	std::string serialport;                                                                         /**<  used serialport on pcDuino, is read from parameters server */
	int serialport_bps;                                                                             /**<  used baudrate, is read from parameters server */

	double l0_ = 0.0;
	double r0_ = 0.0;
	double l1_ = 0.0;
	double r1_ = 0.0;

	// robot velocity
	double vx_ = 0.0;
	double vy_ = 0.0;
	double vth_ = 0.0;

	// last robot position
	double x0_= 0.0;
	double y0_ = 0.0;
	double th0_ = 0.0;

	// new robot position
	double x1_= 0.0;
	double y1_ = 0.0;
	double th1_ = 0.0;

	ros::Time twist_time;
	ros::Time current_time;
	ros::Time last_time;
	ros::Subscriber sub_cmd_vel;

	md49_messages::md49_data md49_data;                                                           /**<  topic /md49_data */
	md49_messages::md49_encoders md49_encoders;                                                   /**<  topic /md49_encoders */

	ros::Publisher md49_odom_pub;
	ros::Publisher md49_encoders_pub;
	ros::Publisher md49_data_pub;
}; //End of class BaseController

#endif
