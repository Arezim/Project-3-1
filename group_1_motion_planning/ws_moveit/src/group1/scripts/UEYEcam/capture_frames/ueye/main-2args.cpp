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

const std::string parameters_filepath="./parameters/file.ini";

inline std::tuple<int,int,std::string,int,int,int> parse_arguments(int argc, char *argv[]){

  InputParser input(argc, argv);

  //defining process
  int process;
  if(input.cmdOptionExists("capture")){
    process=1;
  }
  else if(input.cmdOptionExists("whitebalance")){
    process=2;
  }

  const int &camdev_id=std::stoi(input.getCmdOption("-camdev"));

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

  return {process,camdev_id,imdir,af_min,af_max,capture_seconds};
} 


int main(int argc, char *argv[]){

  std::cout<<"Hello Ueye"<<std::endl;

  auto [process,camdev_id,imdir, afmin, afmax, capture_seconds] = parse_arguments(argc,argv);
  
  Ueye_camera ueye=Ueye_camera(camdev_id);
  ueye.camera_info();
  ueye.set_parameters(30); //(float diplay_to_max_ratio, double newfps)

  if(process==1){

    bool filex=file_exists(parameters_file);
    if (filex){
      std::cout<<"Parameter file " << parameters_file << " exists." <<std::endl;
      
      ueye.load_parameters(parameters_file);
    
      std::cout<<"Executing capture"<<std::endl;

      //aply autofocus
      std::cout<<"Executing automatic focus"<<std::endl;
      ueye.apply_automatic_focus(afmin,afmax);
      ueye.prepare_video(5*capture_seconds);
      ueye.capture_video(imdir,false); 
      ueye.stop_video();
    
      //capture video
      std::cout<<"Executing capture"<<std::endl;
      ueye.prepare_video(capture_seconds);
      ueye.capture_video(imdir,true);
      ueye.stop_video();
    }else std::cout<<"Parameter file " << parameters_file << " DOES NOT exist. RUN WHITE BALANCE FIRST!" <<std::endl;
    

  }elif(process==2){
    //aply autofocus
    std::cout<<"Executing automatic focus"<<std::endl;
    ueye.apply_automatic_focus(afmin,afmax);
    ueye.prepare_video(5*capture_seconds);
    ueye.capture_video(imdir,false); 
    ueye.stop_video();
    
    //apply image control
    std::cout<<"Executing white balance"<<std::endl;
    ueye.apply_automatic_image_control();
    ueye.disable_automatic_image_control();
    //capture to apply image control
    ueye.prepare_video(5*capture_seconds);
    ueye.capture_video(imdir,false);
    ueye.stop_video();
    
    ueye.save_parameters(parameters_file);
  }
  ueye.~Ueye_camera();  
  std::cout<<"Exiting application"<<std::endl;

  return 0;
}
