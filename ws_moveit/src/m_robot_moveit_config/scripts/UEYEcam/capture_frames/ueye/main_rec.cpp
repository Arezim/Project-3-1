#include <iostream>
#include <vector>
#include <typeinfo>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include <tuple>
//#include <chrono>
#include <cstring>
#include <iterator>
#include "ueye.h"
#include "ueye_camera.h"
#include "console.h"


// const std::string parameters_file_cam1="./parameters/camera1.ini";
// const std::string parameters_file_cam2="./parameters/camera2.ini";
const std::string parameters_file="/home/moveit/corosect_ws/src/corosect/corosect/m_robot_moveit_config/scripts/UEYEcam/capture_frames/ueye/parameters/presetParams.ini";


inline std::tuple<int,std::string,int,int,int> parse_arguments(int argc, char *argv[]){

  InputParser input(argc, argv);

  const int &camdev_id=std::stoi(input.getCmdOption("-camdev"));

  std::cout<<"THE camdev_id camdev_id camdev_id camdev_id camdev_id camdev_id "<<camdev_id<<std::endl;

  //AUTOFOCUS
  //if(input.cmdOptionExists("--autofocus")){
  const int default_fmax=1023;
  const int default_fmin=0;
  int af_max,af_min;
  const std::string &temp_fmax = input.getCmdOption("-fmax");
  const std::string &temp_fmin = input.getCmdOption("-fmin");
  if(temp_fmax.empty() | temp_fmin.empty()){ 
    std::cout<<"UEYE_MAIN argument passing BUGG :: please provide min and max value for autofocus"<<std::endl;
  }
  else {
      af_max=std::stoi(temp_fmax);
      af_min=std::stoi(temp_fmin);
  }
  std::cout<<"temp_fmax : "<<temp_fmax<<",temp_fmin : "<<temp_fmin<<std::endl;
  std::cout<<"AFmax : "<<af_max<<", AF_min : "<<af_min<<std::endl;

  const std::string imdir=input.getCmdOption("-temp_capture_dir");
  std::cout <<"imdir = "<<imdir<<std::endl;

  const std::string &cs = input.getCmdOption("-cs");
  int capture_seconds=std::stoi(cs);

  return {camdev_id,imdir,af_min,af_max,capture_seconds};
} 


int main(int argc, char *argv[]){

  std::cout<<"Hello Ueye"<<std::endl;






  auto [camdev_id,imdir, afmin, afmax, capture_seconds] = parse_arguments(argc,argv);
  // int camdev_id=1;
  // std::string imdir="/home/melissap/Desktop/CoRoSect/10.code/CF2-capturing_scripts/v9/captures";
  // int afmin=330;
  // int afmax=360;
  // int capture_seconds=1;






  std::cout<<"Main "<<imdir<<", "<<capture_seconds<<std::endl;

  Ueye_camera ueye=Ueye_camera(camdev_id);

  // std::string parameters_file;
  // if (camdev_id==1){parameters_file=parameters_file_cam1;}else if(camdev_id==2){parameters_file=parameters_file_cam2;}

  ueye.load_parameters(parameters_file);
  ueye.camera_info();
  ueye.set_parameters(1); //(float diplay_to_max_ratio, double newfps)

  // //apply autofocus
  // std::cout<<"Executing automatic focus"<<std::endl;
  // ueye.apply_automatic_focus(afmin,afmax);
  // ueye.prepare_video(5*capture_seconds);
  // ueye.capture_video(imdir,false); 
  // ueye.stop_video();

  //capture video
  std::cout<<"Executing capture"<<std::endl;
  ueye.prepare_video(capture_seconds);
  ueye.capture_video(imdir,true);
  ueye.stop_video();

  ueye.~Ueye_camera();  
  std::cout<<"Exiting application"<<std::endl;

  return 0;
}
