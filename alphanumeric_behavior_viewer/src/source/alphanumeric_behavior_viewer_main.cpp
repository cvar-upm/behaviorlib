/*!*******************************************************************************************
 * \brief     Alphanumeric behavior viewer implementation file.
 * \authors   Javier Melero Deza
 * \copyright Copyright (c) 2021 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "../include/alphanumeric_behavior_viewer.h"
void spinnerThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ALPHANUMERIC BEHAVIOR VIEWER");
  ros::NodeHandle n("~");
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  n.param<std::string>("catalog_path", config_file, "behavior_catalog.yaml");
  std::thread thr(&spinnerThread);
  
  //test_take_off = n.serviceClient<behavior_execution_manager_msgs::ActivateBehavior>("/" + drone_id_namespace +  "/" + "quadrotor_motion_with_pid_control/behavior_follow_path" + "/"+ "activate_behavior");
  //LOOP
    // ncurses initialization
  setlocale(LC_ALL, "");
  std::setlocale(LC_NUMERIC, "C");
  initscr();
  start_color();
  scrollok(stdscr, TRUE);
  use_default_colors();  
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_BLUE, -1);
  init_pair(2, COLOR_GREEN, -1);
  init_pair(3, COLOR_CYAN, -1);
  init_pair(4, COLOR_RED, -1);
  init_pair(5, COLOR_YELLOW, -1);


  //Input variable
  char command = 0;
  pointer_c = 0;

  //check_situation = n.serviceClient<behavior_execution_manager_msgs::CheckSituation>("/" + drone_id_namespace +  "/" + "basic_quadrotor_behaviors/behavior_take_off" + "/"+ "check_activation_condition");
  move(0,0);clrtoeol();
  printw("                        - ALPHANUMERIC BEHAVIOR VIEWER -");
  std::string path = config_file;
  YAML::Node config = YAML::LoadFile(path);
  for (YAML::const_iterator behaviorsIterator=config["behaviors"].begin();
  behaviorsIterator!=config["behaviors"].end();++behaviorsIterator) {
    
    behaviors.push_back((*behaviorsIterator)["behavior"].as<std::string>());
    std::string behavior = (*behaviorsIterator)["behavior"].as<std::string>();
    std::for_each(behavior.begin(), behavior.end(), [](char & c){
      c = ::tolower(c);
    });
    std::string aux = "behavior_";
    std::string node = aux + behavior;
    names.push_back(node);
    packages.push_back((*behaviorsIterator)["package"].as<std::string>());
  }
  printoutControls();
  ros::Rate loop_rate(FREQ_INTERFACE);

 
  while (ros::ok()){
    // Read messages
    ros::spinOnce();
    for (int i = 0; i< names.size(); i++){
      path =  "/" + drone_id_namespace + "/" + packages.at(i) + "/" + names.at(i) + "/check_activation";
      check_behavior_situation_srv = n.serviceClient<behavior_execution_manager_msgs::CheckActivation>(path);
      behavior_execution_manager_msgs::CheckActivation check_activation_msg;
      check_behavior_situation_srv.call(check_activation_msg);
      auto it = find(active_behaviors.begin(), active_behaviors.end(), behaviors.at(i).c_str());
      if (check_activation_msg.response.is_active && it == active_behaviors.end()){
        //printw(check_activation_msg.response.parameters.c_str());
        active_behaviors.push_back(behaviors.at(i));
        std::string str = "& ";
        std::string str2 = "\n";
        size_t found = check_activation_msg.response.parameters.find(str2);
        if (found != std::string::npos){            
          check_activation_msg.response.parameters.replace(found, 1, str);           
        }        
        active_behaviors_parameters.push_back(check_activation_msg.response.parameters);
      }
      else if (!check_activation_msg.response.is_active && it != active_behaviors.end()){
        active_behaviors.erase(it);
        int index = it - active_behaviors.begin();
        active_behaviors_parameters.erase(active_behaviors_parameters.begin() + index);
        for (int j = 0; j < active_behaviors.size(); j++){
          move (pointer_d + index +1 + j, 0);clrtoeol();
        }    
      }
      else if (check_activation_msg.response.is_active && it != active_behaviors.end()){
        std::string str = "& ";
        std::string str2 = "\n";
        size_t found = check_activation_msg.response.parameters.find(str2);
        if (found != std::string::npos){            
          check_activation_msg.response.parameters.replace(found, 1, str);           
        }
        int index = it - active_behaviors.begin();
        if (active_behaviors_parameters.at(index) != check_activation_msg.response.parameters){
          active_behaviors.erase(it);
          active_behaviors_parameters.erase(active_behaviors_parameters.begin() + index);

          for (int j = 0; j < active_behaviors.size(); j++){
            move (pointer_d + index +1 + j, 0);clrtoeol();
          } 
          active_behaviors.push_back(behaviors.at(i));
          active_behaviors_parameters.push_back(check_activation_msg.response.parameters);                
        }        
      }
    }
    pointer_c = 4;
    move (pointer_c, 0);clrtoeol(); 
    for (int i = 0; i< active_behaviors.size(); i++){
      
      printw(" ");printw(active_behaviors.at(i).c_str()); printw(" "); printw (active_behaviors_parameters.at(i).c_str());
      pointer_c++;
      move(pointer_c, 0);clrtoeol();

    }

    refresh();
    loop_rate.sleep();
    }
    endwin();
    return 0;
}

void printoutControls(){
  pointer_c = 2;
  move(pointer_c,0);clrtoeol();
  printw("List of active behaviors:");
  pointer_c = pointer_c + 2;
  move(pointer_c,0);clrtoeol();
  pointer_d = pointer_c;
  refresh();
}



//High level command


