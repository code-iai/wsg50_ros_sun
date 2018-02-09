/*
    ROS node to calculate wrenches

    Copyright 2018 Università della Campania Luigi Vanvitelli

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>

#include <stdio.h>
#include <iostream>

#include <wsg_50_common_sun/Tactile.h>
#include <geometry_msgs/WrenchStamped.h>

#include <ros/package.h>

#include <std_srvs/Empty.h>


//------new
#include <Helper1.0.h>
#include <ANN1.0.h>

using namespace std;

//==========GLOBAL VARS========//
const int N_MEAN = 60; //Number of seamples used in the bias computation
const int NUM_V = 25; //Length of voltage Vector
Matrix<6,NUM_V> K; //Linear Calibration Matrix
Vector<NUM_V> voltages = Zeros, bias = Zeros, voltages_rect = Zeros;
Vector<6> wrench = Zeros;
bool voltageMessageArrived = false; //Used in the service RemoveBias
bool b_linearCalib = true;
string fingerCode("");
ANN * myANN;

//*******GLOBAL ROS VARS******//
string name_space;
ros::NodeHandle* n;
//----------------------------//
//*******PUBLISHERS**********//
ros::Publisher pubWrench;
ros::Publisher pubVoltagesRect;
//----------------------------//
//**********MSGS*************//
geometry_msgs::WrenchStamped msgWrench;
wsg_50_common_sun::Tactile msgVoltageRect;
//====================================//


//=========LOCAL FCNs INTERFACEs=======//

//Model init, NB: Call it before Voltage-Subscriber definition and after Wrench-Publisher definition
void init_model(string path);
void init_ANN_model( string path );

//Calculate voltage bias
void _removeBias();

//====================================//


//==========SERVICES=========//

//Service to remove bias of voltages
bool removeBias(std_srvs::Empty::Request  &req, 
   		 		std_srvs::Empty::Response &res){
	_removeBias();
	return true;	
}

//====================================//


//==========TOPICs CALLBKs=========//
void readV( const wsg_50_common_sun::Tactile::ConstPtr& msg  ){
	for(int i = 0; i<NUM_V; i++){
		voltages[i] = msg->voltages.data[i];
	}
    voltageMessageArrived = true;
    voltages_rect = voltages - bias;
    
    //Calculate wrench model
    if(b_linearCalib)
        wrench = K*voltages_rect;
    else
        wrench = myANN->compute( voltages_rect );

    //Fill ROS msgs
    msgWrench.header.stamp = msg->header.stamp;
    msgWrench.header.frame_id = msg->header.frame_id;
    msgWrench.wrench.force.x = wrench[0];
	msgWrench.wrench.force.y = wrench[1];
	msgWrench.wrench.force.z = wrench[2];

	msgWrench.wrench.torque.x = wrench[3];
	msgWrench.wrench.torque.y = wrench[4];
	msgWrench.wrench.torque.z = wrench[5];

    msgVoltageRect.voltages.data.resize(25);
	msgVoltageRect.header.stamp = msg->header.stamp;
    msgVoltageRect.header.frame_id = msg->header.frame_id;
	msgVoltageRect.voltages.layout.dim.resize(1);
    msgVoltageRect.voltages.layout.dim[0].label = msg->voltages.layout.dim[0].label;
    msgVoltageRect.voltages.layout.dim[0].size = msg->voltages.layout.dim[0].size;
    msgVoltageRect.voltages.layout.dim[0].stride = msg->voltages.layout.dim[0].stride;
    msgVoltageRect.voltages.layout.data_offset = msg->voltages.layout.data_offset;

    for(int i=0; i<NUM_V ; i++)
        msgVoltageRect.voltages.data[i] = voltages_rect[i];

    //Publish
    pubWrench.publish( msgWrench );
	pubVoltagesRect.publish( msgVoltageRect );  

}

//====================================//


//==============MAIN================//

int main(int argc, char const *argv[]){

    int _argc = 0;
    char** _argv = NULL;
	ros::init(_argc,_argv,"read_wrench");

	n = new ros::NodeHandle("~");

    
    /**** CHECK PARAMS ****/
    string path("");
    path = ros::package::getPath("wsg_50_driver_sun");
    path = path + "/Finger_files/";
    int fingerPosition = 0;
    //sleep(4);
    n->param("fingerCode" , fingerCode, string("null") );
    n->param("fingerPosition" , fingerPosition, 0 );
    n->param("linearCalib" , b_linearCalib, true );

    if(fingerCode == string("null")){
        cout << BOLDRED << "Error! - No params for 'fingerCode' - stopping node... " << CRESET << endl;
        return -1; 
    }

    if(fingerPosition != 0 /*&& fingerPosition!=1*/ ){ //For future implementations
        cout << BOLDRED << "Error! - invalid fingerPosition [" << fingerPosition << "] " << CRESET << endl;
        return -1; 
    }
		sleep(1);
    path = path + fingerCode;

    if(b_linearCalib){
        path =  path + "/LinearCalib";
    } else {
        path = path + "/ANNCalib";
    }

    //path = path + fingerCode + "/LinearCalib/K_" + fingerCode  + ".txt";
    
    name_space = ros::this_node::getNamespace();

    cout << BOLDBLUE << "Finger " << fingerCode << endl; // /*For future implementation*/ << "  | Position " << fingerPosition << CRESET << endl;
    /*************************************************/

    /******INIT ROS MSGS**********/
	msgVoltageRect.voltages.data.resize(NUM_V);
	/********************/

    /*******INIT ROS PUB**********/
    //Force pub
	pubWrench = n->advertise<geometry_msgs::WrenchStamped>( name_space + "/wrench",1);
	//Voltage_rect pub
	pubVoltagesRect = n->advertise<wsg_50_common_sun::Tactile>( name_space + "/tactile_voltage/rect",1);
    /***************************/

    /*******INIT MODEL**********/
    init_model(path); //Don't move from here!
    /***************************/
	

    /*******INIT ROS SUB**********/
	//Status subscriber
	ros::Subscriber subVoltage = n->subscribe(name_space + "/tactile_voltage",1,readV);
	//Remove bias subscriber
	ros::ServiceServer serviceRemoveBias = n->advertiseService(name_space + "/removeBias", removeBias);
    /***************************/



/*============LOOP==============*/
    ros::spin();
/*==============================*/

	
	return 0;
}



/*===========LOCAL FCNs IMPL==========*/

//Calculate bias
void _removeBias(){

    cout << BOLDYELLOW << "REMOVING BIAS... ";

    Matrix<N_MEAN,NUM_V> lastVReads = Zeros; //buffer

    //redefinition of subscriber cause you can't use previouse subscriber in a service
    ros::Subscriber subVoltage = n->subscribe(name_space + "/tactile_voltage",1,readV);

    //*****Fill lastVReads Matrix****//
    int _index = 0; //I don't use first 10 samples
    for(int i=0; i < N_MEAN + 10 ; i++){

        //Wait for Voltages
        while(!voltageMessageArrived){
		    ros::spinOnce();
            if(!voltageMessageArrived)
                sleep(0.01); //10ms
        }
        voltageMessageArrived = false;

        //Fill lastVReads Row
        lastVReads[_index] = voltages;

        //Update column index
	    _index = (_index+1)%N_MEAN;

	}
    //**************************************//

    //*****Calculate bias****//
	bias = Zeros;
	for(int i = 0; i<N_MEAN ; i++)
        bias = bias + lastVReads[i];

    bias = bias/N_MEAN;
    //**************************************//

    cout << "DONE!" << endl;
	
    cout << "-----------------" << endl << "NEW VOLTAGE BIAS =" << endl << bias << endl << "-----------------" << CRESET << endl;
}

void init_model(string path){

    if(b_linearCalib){
        path = path + "/K_" + fingerCode  + ".txt";
        //*****Read K from file****//
        K = readFileM(path.c_str(), 6, NUM_V);//.T();
				cout << "K = " << endl << K << endl;
				cout << BOLDBLUE << "Linear Model" << CRESET << endl;
        //**************************************//
    } else {
        init_ANN_model( path );
				cout << BOLDBLUE << "ANN Model" << CRESET << endl;
				//myANN->display(); //debug line
    }

    //*****Remove initial bias****//
    _removeBias();
    //**************************************//

    cout << BOLDGREEN << "Model initialized!" << CRESET << endl;

}

void init_ANN_model( string path ){
 /* old Version
    string WH = path + "/W_h.txt";
    string bh = path + "/b_h.txt";

    string WL = path + "/W_l.txt";
    string bl = path + "/b_l.txt";

    string mMi = path + "/mM_i.txt";
    string mMo = path + "/mM_o.txt";

    string meta = path + "/meta.txt";

    Vector<1> numHiddenNeurons= readFileV(meta.c_str(),1);

    myANN = new ANN ( ANN2( NUM_V, numHiddenNeurons[0], 6, WH.c_str(), bh.c_str(), WL.c_str(), bl.c_str(), mMi.c_str(), mMo.c_str()) );
*/
		myANN = new ANN ( path );

}
