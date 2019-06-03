#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <stdio.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex;
nav_msgs::Odometry g_odom;
float pre_dAngleTurned;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // receive a '/odom' message with the mutex
    mutex.lock(); {
        g_odom = msg;
    } mutex.unlock();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// odomÀž·ÎºÎÅÍ ÇöÀçÀÇ º¯È¯Çà·Ä Á€ºžžŠ ž®ÅÏ!
tf::Transform
getCurrentTransformation(void)
{
    // transformation ¹öÆÛ
    tf::Transform transformation;

    // odom ¹öÆÛ
    nav_msgs::Odometry odom;

    // copy a global '/odom' message with the mutex
    mutex.lock(); {
        odom = g_odom;
    } mutex.unlock();

    // À§Ä¡ ÀúÀå
    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    // ÈžÀü ÀúÀå
    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    // ž®ÅÏ
    return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ·Îº¿ÀÌ žØÃçÀÖŽÂ »óÅÂ(Ã³Àœ »óÅÂ)ÀÇ À§Ä¡žŠ ÀúÀå!
tf::Transform
getInitialTransformation(void)
{
    // tf º¯È¯Çà·Ä
    tf::Transform transformation;

    // Ã³ÀœÀ§Ä¡¿¡ ŽëÇÑ odometry žÞœÃÁö ¹Þ±â
    ros::Rate loopRate(1000.0);

    while(ros::ok()) {
        // ÀÏŽÜ callback žÞœÃÁöžŠ ¹Þ°í!
        ros::spinOnce();

        // get current transformationreturn;
        transformation = getCurrentTransformation();

        // žÞœÃÁöžŠ ¹ÞŸÒÀžžé break!
        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }

    // ž®ÅÏ
    return transformation;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// do rotation 
bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
    //the command will be to turn at 'rotationSpeed' rad/s
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;
    
    if(dRotation < 0.) {
        baseCmd.angular.z = -dRotationSpeed;
    } else {
        baseCmd.angular.z = dRotationSpeed;
    }

    // as moving , receive odometry message;
    bool bDone = false;
    ros::Rate loopRate(1000.0);



    while(ros::ok() && !bDone) {
        // wait receive callback message.
        ros::spinOnce();

        // get current transformation
        tf::Transform currentTransformation = getCurrentTransformation();

        //see how far we've traveled
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        tf::Quaternion rotationQuat = relativeTransformation.getRotation();
	
	 
         
	
       // double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));
	
	double dAngleTurned = rotationQuat.getAngle();
	// getangle method return 
	//ROS_INFO("%f/%f",dAngleTurned,temp);
	// comparing using getangle() and using atan 
   	

  // check break comdition

    if( fabs(dAngleTurned) > fabs(dRotation) || (abs(pre_dAngleTurned - dRotation) <  abs(dAngleTurned - dRotation)) || (dRotation == 0)) 
	{
            bDone = true;
            break;
        } else {
	    pre_dAngleTurned = dAngleTurned;
            //send the drive command
            pubTeleop.publish(baseCmd);

            // sleep!
            loopRate.sleep();
        }
    }

    // base value 
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ÀÌµ¿
bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
{
    //the command will be to go forward at 'translationSpeed' m/s
    geometry_msgs::Twist baseCmd;

    if(dTranslation < 0) {
        baseCmd.linear.x = -dTranslationSpeed;
    } else {
        baseCmd.linear.x = dTranslationSpeed;
    }

    baseCmd.linear.y = 0;
    baseCmd.angular.z = 0;

    // ÀÌµ¿ÇÏžéŒ­ ÇöÀçÀ§Ä¡¿¡ ŽëÇÑ odometry žÞœÃÁö ¹Þ±â
    bool bDone = false;
    ros::Rate loopRate(1000.0);

    while(ros::ok() && !bDone) {
        // ÀÏŽÜ callback žÞœÃÁöžŠ ¹Þ°í!
        ros::spinOnce();

        // get current transformation
        tf::Transform currentTransformation = getCurrentTransformation();

        //see how far we've traveled
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        double dDistMoved = relativeTransformation.getOrigin().length();

        // ÁŸ·áÁ¶°Ç ÃŒÅ©

        if(fabs(dDistMoved) >= fabs(dTranslation)) {
            bDone = true;
            break;
        } else {
            //send the drive command
            pubTeleop.publish(baseCmd);

            // sleep!
            loopRate.sleep();
        }
    }

    //  ÃÊ±âÈ­

    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{

	FILE *f;
	if ( ! (f = fopen("/home/hang/catkin_ws/input.txt","r")) )
		{printf("잘못된 경로입니다\n");
		return 1;
	}
	
	int n;	
	double x[10];
	double y[10];
	
	fscanf(f,"%d\n",&n);
	

	for(int i = 0 ; i <n ;i++ )    
 		fscanf(f,"%lf %lf",&x[i],&y[i]);

	for(int i = 0 ; i <n;i++ )  
		ROS_INFO(" destination %d : (%lf,%lf)",i+1,x[i],y[i]);
	

	for(int i = 0 ; i <n ;i++ )    
 		printf("%lf %lf\n",x[i],y[i]);

    // ROS ÃÊ±âÈ­
    ros::init(argc, argv, "turtle_position_move");

    // exception
    if(argc != 1) {
        printf(">> rosrun knu_ros_lecture turtle_position_move \n");
        return 1;
    }

    // 처음 위치 갱신 (상대 tf 를 구하기 위해서.)

   
    
    //(1,2)로 테스트
    
   for(int i = 0 ; i < n ; i++)
    {

    // Ros initialization
    ros::NodeHandle nhp, nhs;

    // Decleation of subscriber
    ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);

    // Create a publisher object
    ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    
    tf::Transform initialTransformation = getInitialTransformation();
    
    double temp_x = x[i];
    double temp_y = y[i];

    
    
    //int temp_x = 0;
    //int temp_y = 0;
	
    double start_x = initialTransformation.getOrigin().getX();
    double start_y = initialTransformation.getOrigin().getY();
    // -20 ?% 11 = -9 
    printf("start point : x, y: %lf %lf\n",start_x,start_y);
    printf("destinateion : (x, y): %lf %lf\n",temp_x,temp_y);
    // 상대 위치 구하기.
     
    double relative_x, relative_y;
    
    relative_x = temp_x - start_x;
    relative_y = temp_y - start_y;

    double dRotation1 = atan((temp_y - start_y)/(temp_x-start_x));
    // 상대 위치의 길이 구하기. 
    double dTranslation1 = sqrt(relative_x*relative_x + relative_y*relative_y );
    // 길이는 맞는 거 같애 
    // 각도만 찾으면 되는데, 
    
    tf::Quaternion currentInfo = initialTransformation.getRotation();
    //이걸 어떻게 구한현거지 ?
    //현재의 각도 구하기 
    double currentRotate = atan2((2 * currentInfo[2] *currentInfo[3]) , (1-(2 * (currentInfo[2] * currentInfo[2]) ) ))* 180/3.14159265;
    //움직여야한 각도 구하기.
    double willrotate =  (atan2( relative_y, relative_x ) * 180/3.14159265);
    
    
   
    
    double dRotation = willrotate - currentRotate; 
   // float _dRatation = (float)((int)dRotation % 360);
    
	
    if (abs(dRotation)> 180 ){
if(dRotation>0){
dRotation=(dRotation-360);
}
else{
dRotation=(dRotation+360);
}
}
	
    double dTranslation = dTranslation1; 

    printf("터틀봇의 현재 방향 : %lf\n", currentRotate); 
    printf("터틀봇의 원하는 각도 : %lf\n", willrotate); 
    printf("돌아가야하는 각도 %lf \n", dRotation);
    printf("터틀봇의 움직여야 하는 거리 : %lf\n",dTranslation1); 

    //속도에 따라서 오차가 생길수 밖에 없음 // 느리게 할수도 오차가 줄어듬.
    doRotation(pub, initialTransformation, toRadian(dRotation), 0.2);
    doTranslation(pub, initialTransformation, dTranslation, 0.2);
    //멈추게 하는 코드
    ros::spinOnce();
    geometry_msgs::Twist base;
    base.linear.x = 0.0;
    base.linear.y = 0.0;
    base.linear.z = 0.0;
    base.angular.x = 0.0;
    base.angular.y = 0.0;
    base.angular.z = 0.0;
    pub.publish(base);        

   printf("---------------멈추고 기다리는 중------------\n"); 

    ////////값을 초기화 해서 멈추게 하자.
    sleep(5);  
   
 
    }

    

    return 0;
}
