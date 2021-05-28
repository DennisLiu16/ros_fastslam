// option
#define LOCALIZATION
#define CANPRTPOS

#include <stdio.h>
#include "Particle.h"
#include "VehicleModel.h"
#include <math.h>
#include "MatrixCal.h"
#include "QuadrantAngle.h"
#include "resample_weight.h"
#include <random>
#include "fastSLAM_localization.h"
#include "ConeMap.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"


double v_cmd = 0.0f;						// velocity cmd, unit: (m/s)
double w_cmd = 0.0f;						// omega cmd, unit: (rad/s)
int data_size;
double sensor_dataX[40];		// sensor data distance
double sensor_dataP[40];		// sensor data head angle
int sensor_dataC[40];		// sensor data color
bool zt_in = false;


void CommandCallback(const geometry_msgs::Twist& msg){
	v_cmd = msg.linear.x;
	w_cmd = msg.angular.z;
	
}

void coneCallback(const std_msgs::Float64MultiArray& msg){
	data_size = msg.data.size()/3;
	for(int i=0;i<msg.data.size()/3;i++){
		sensor_dataX[i] = msg.data[3*i];
		sensor_dataP[i] = msg.data[3*i+1];
		sensor_dataC[i] = msg.data[3*i+2];
	}
	zt_in = true;
	if(data_size == 0){
		zt_in = false;
	}
	
}

int main(int argc, char **argv) {

	//Files open
/*
	FILE* fp_St;
	fopen_s(&fp_St, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\St_Data.txt", "w");
	FILE* fp_Nt;
	fopen_s(&fp_Nt, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\Nt_Data.txt", "w");
	FILE* fp_mu;
	fopen_s(&fp_mu, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\Cone_mu_Data.txt", "w");
	FILE* fp_cor;
	fopen_s(&fp_cor, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\Cone_cor_Data.txt", "w");
	FILE* fp_target;
	fopen_s(&fp_target, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\Cone_tar_Data.txt", "w");
	FILE* fp_uvr;
	fopen_s(&fp_uvr, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\uvr_Data.txt", "w");
	FILE* fp_zt;
	fopen_s(&fp_zt, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\zt_Data.txt", "w");
	FILE* fp_ztnum;
	fopen_s(&fp_ztnum, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\ztnum_Data.txt", "w");
	FILE* fp_ztset;
	fopen_s(&fp_ztset, "C:\\Users\\open4\\Desktop\\FastSLAM_draw\\ztset_Data.txt", "w");
*/
	ros::init(argc, argv, "localization");	//node name
	
	ros::NodeHandle n;
	ros::Publisher 	pub = n.advertise<std_msgs::Float64MultiArray>("status", 1000);
 	//ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, CommandCallback);	//topic name and callback function
	ros::Subscriber cmd_sub = n.subscribe("FeedBack_Vel", 1000, CommandCallback);
	ros::Subscriber sensor_sub = n.subscribe("coneXP", 1000, coneCallback);
	
	ros::Rate loop_rate(20);

	ROS_INFO("Initialize Finished!");

	//////// Parameter Init
	const int M = 100;							// Particle number
//	const int landmark_number = 60;				// expext cone number
	double del_t = 0.05;						// unit : (s)
	double Init_Size = 0.5f;					// the init range of vehicle X Y, unit: (m)
	double Init_way = 2.5;						// the init range of vehicle Theta, unit: (degree)
	double Init_position[2] = { 9*0.395+12.0,7*0.395+12.0 };	// unit: (m)
	double Init_head_angle = PI / 2;			// unit: (rad)
//	int file_times = 10;						// sample time if number = 10, sample time = del_t*10 = 0.1;
	int weightest_index = 0;					// plot the probability highest particle
	//////// Parameter Init
	std::default_random_engine normal_seed;
	std::normal_distribution<double> gaussian(0.0, 1.0);

	//////// Init the vehicle state of the particle
	struct node Particle[M];
	int localFlag[M] = { 0 };
	struct ConeSet* conePtr = NULL;

	// IF use init map, run this two shit
#ifdef LOCALIZATION
	const int coneNumber = 40;//30	// Case ellipse
	ConeInit(&conePtr);
#else
	// ELSE just
	const int coneNumber = 0;
#endif // LOCALIZATION

	for (int j = 0; j < M; j++) {
		Particle[j].St[0] = Init_position[0] + Init_Size * gaussian(normal_seed);
		Particle[j].St[1] = Init_position[1] + Init_Size * gaussian(normal_seed);
		Particle[j].St[2] = Init_head_angle + Init_way * PI / 180 * gaussian(normal_seed);
		Particle[j].Nt = coneNumber;						// cone number
		Particle[j].next = conePtr;
	}
	double zt[2];					// distance, head angle
	int cone_color = -1;

	double ut[3] = { 0.0 };				// [u v r]
	double wt[M];
	double St_1[3];

	// POOPOO add for statistical output
	double VX_avg = 0.0;
	double VY_avg = 0.0;
	double VPhi_avg = 0.0;

	// variable init
	/*
	int ffw = 0;
	double zt_set[landmark_number][2] = { 0 };
	int zt_number = 0;
	int zt_buffer_flag = 0;
	*/
	////////////////////////////////////////////////////////
	while (ros::ok()) {
		// For each particle, run pure pridiction (MOTION MODEL)
		for (int m = 0; m < M; m++) {
			for (int i = 0; i < 3; i++)	St_1[i] = Particle[m].St[i];
//#ifdef LOCALIZATION
			VehicleModel(v_cmd, w_cmd, St_1, del_t, Particle[m].St);//vehiclemodel
//#else
//#endif // LOCALIZATION
		}

		// Check if CDTDRDY appear, thEn do "SLAM update"
		//stat = canReadSpecific(hnd, CDTDEND, ConeMsg, &dlc, &flags, &timestamp);
		// If data exist or error (other than nomsg)
		if (zt_in == true) {

			// Run threw sensed landmarks
			for (int jk = 0; jk < data_size; jk++) {//no cones looked

				// If simply nomsg (left in buffer)
				//if (stat == canERR_NOMSG) {	
				//	if (zt_number != 0)zt_buffer_flag = 1;
				//	break;
				//}

				//int16_t tmp_dis = (ConeMsg[0] | ConeMsg[1] << 8);
				//int16_t tmp_angle = (ConeMsg[2] | ConeMsg[3] << 8);
				//zt[0] = tmp_dis / canGain + 0.15;	// 0.15 is roughly camera position bias
				//zt[1] = tmp_angle / canGain;
				//cone_color = ConeMsg[4] / canGain;
				zt[0] = sensor_dataX[jk];
				zt[1] = sensor_dataP[jk];
				cone_color = sensor_dataC[jk];

					// skip if too far
				if (zt[0] > 3.0) continue;	//t

/*
				if (!zt_buffer_flag) {
					zt_number = zt_number + 1;
					zt_set[jk][0] = zt[0];
					zt_set[jk][1] = zt[1];
				}
*/
				for (int m = 0; m < M; m++) {
					int Nt_1 = 0;
					struct ConeSet* Cone_set = NULL;

						// Just pushing shit to shit
					for (int i = 0; i < 3; i++)	St_1[i] = Particle[m].St[i];
					Nt_1 = Particle[m].Nt;
					Cone_set = Particle[m].next;

//#ifdef LOCALIZATION
					// Actual slam part ()
					fast_slam_loc(zt, ut, St_1, Nt_1, Particle[m].next, Particle[m].St, &wt[m], &localFlag[m], cone_color);
//#else
//#endif //LOCALIZATION
				}

					// Resample as follow
				int re_sample_index[M];
				resample_weight(wt, re_sample_index, M);
				struct node up_Particle[M];
				double weight_tmp = 0;
				for (int i = 0; i < M; i++) {
					up_Particle[i] = Particle[i];

						// Search max likelyhood id
					if (wt[i] > weight_tmp) {
						weight_tmp = wt[i];
						weightest_index = i; // for matlab plot the most likehood particle
					}
				}
				for (int i = 0; i < M; i++) {
					Particle[i] = up_Particle[re_sample_index[i]];
				}

					//int flagSum = 0;
					//for (int i = 0; i < M; i++) {
					//	flagSum = flagSum + localFlag[i];
					//}
					//printf("%d\r\n", flagSum);

					// End of resample

			}	// End of CDTRDY scanning "for loop"

			//POOPOO
			//break;
		}
		zt_in = false;
		// Else just simply NOMSG

		// POOPOO calculate average output
		VX_avg = 0.0;
		VY_avg = 0.0;
		VPhi_avg = 0.0;
		for (int i = 0; i < M; i++) {
			VX_avg += Particle[i].St[0];
			VY_avg += Particle[i].St[1];
			VPhi_avg += Particle[i].St[2];
			// Case statistical output
		}
		VX_avg = VX_avg / (double)M;
		VY_avg = VY_avg / (double)M;
		VPhi_avg = VPhi_avg / (double)M;

		//send position to plot.py 
		std_msgs::Float64MultiArray status;
		
		status.data.push_back((double)VX_avg);
		status.data.push_back((double)VY_avg);
		status.data.push_back((double)VPhi_avg);

		pub.publish(status);
    	ros::spinOnce();
    	loop_rate.sleep();

	}

	return 0;
}
/*
		//Predict forward output due to DLYN
		double Xpred[3];
		Xpred[0] = VX_avg;
		Xpred[1] = VY_avg;
		Xpred[2] = VPhi_avg;

		// Threw out localization result //output ros
		msgtmp = Xpred[0] * 100.0f;
		msg[0] = msgtmp & 0xFF;
		msg[1] = (msgtmp >> 8) & 0xFF;
		msgtmp = Xpred[1] * 100.0f;
		msg[2] = msgtmp & 0xFF;
		msg[3] = (msgtmp >> 8) & 0xFF;
		msgtmp = Xpred[2] * 100.0f;
		msg[4] = msgtmp & 0xFF;
		msg[5] = (msgtmp >> 8) & 0xFF;
*/
/*

	if (fp_uvr)fclose(fp_uvr);
	if (fp_St)fclose(fp_St);
	if (fp_Nt)fclose(fp_Nt);
	if (fp_mu)fclose(fp_mu);
	if (fp_cor)fclose(fp_cor);
	if (fp_target)fclose(fp_target);
*/
/*
		if (ffw >= file_times) {
			struct ConeSet* fpt_cone;
			fprintf(fp_uvr, "%f\t%f\t%f\n", ut[0], ut[1], ut[2]);
			//double VX = Particle[weightest_index].St[0];
			//double VY = Particle[weightest_index].St[1];
			//double VPhi = Particle[weightest_index].St[2];
			double VX = VX_avg;
			double VY = VY_avg;
			double VPhi = VPhi_avg;
			fprintf(fp_zt, "%f\t%f\n", VX + zt[0] * cos(zt[1] + VPhi), VY + zt[0] * sin(zt[1] + VPhi));
			fprintf(fp_St, "%f\t", VX);
			fprintf(fp_St, "%f\t", VY);
			fprintf(fp_St, "%f\n", VPhi);
			fprintf(fp_Nt, "%d\n", Particle[weightest_index].Nt);
			fpt_cone = Particle[weightest_index].next;
			for (int i = 0; i < Particle[weightest_index].Nt; i++) {
				if (fpt_cone) {
					fprintf(fp_mu, "%f\t", fpt_cone->mu[0]);
					fprintf(fp_mu, "%lf\n", fpt_cone->mu[1]);
					fprintf(fp_cor, "%f\t", fpt_cone->cor[0]);
					fprintf(fp_cor, "%f\t", fpt_cone->cor[1]);
					fprintf(fp_cor, "%f\t", fpt_cone->cor[2]);
					fprintf(fp_cor, "%f\n", fpt_cone->cor[3]);
					fprintf(fp_target, "%d\n", fpt_cone->credible);
					fpt_cone = fpt_cone->next;
				}
				else {
					printf("txt error\n");
				}

			}
			fprintf(fp_ztnum, "%d\n", zt_number);
			for (int i = 0; i < zt_number; i++) {
				double sensor_coneX = VX + zt_set[i][0] * cos(zt_set[i][1] + VPhi);
				double sensor_coneY = VY + zt_set[i][0] * sin(zt_set[i][1] + VPhi);
				fprintf(fp_ztset, "%f\t%f\n", zt_set[i][0], zt_set[i][1]);
				//printf("%f\t%f\n", sensor_coneX, sensor_coneY);
			}
			ffw = 0;
			zt_number = 0;
			for (int i = 0; i < landmark_number; i++) {
				zt_set[i][0] = 0;
				zt_set[i][1] = 0;
			}
			zt_buffer_flag = 0;
		}
		else {
			ffw = ffw + 1;
		}
*/
