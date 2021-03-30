/*!*******************************************************************************************
 * \brief     Keyboard activation with behaviors implementation file.
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

#include "../include/keyboard_behavior_activation_main.h"
void spinnerThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "KEYBOARD BEHAVIOR ACTIVATION");
  ros::NodeHandle n("~");
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  n.param<std::string>("config_file", config_file, "keyboard_behavior_activation_config.yaml");
  std::thread thr(&spinnerThread);

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

  
  move(0,0);clrtoeol();
  printw("                        - KEYBOARD BEHAVIOR ACTIVATION -");
  //Print controls
  std::string path = config_file;
  YAML::Node config = YAML::LoadFile(path);
  for(YAML::const_iterator it=config.begin(); it!=config.end(); ++it){
    const std::string &behavior=it->first.as<std::string>();
    behaviors.push_back(behavior);

    YAML::Node list = it->second;
    names.push_back(list["node"].as<std::string>());
    parameters.push_back(list["parameters"].as<std::string>());
    packages.push_back(list["package"].as<std::string>());
    mappings.push_back(list["key"].as<std::string>());

  } 
  printoutControls();
  ros::Rate loop_rate(FREQ_INTERFACE);

 
  while (ros::ok()){
    // Read messages
    ros::spinOnce();
    int uppercase = 0;
    //Read command

    command = getch();
    if (isupper(command)){
      uppercase = 1;
      command = command + 32;
    }
    std::string s(1, command);
    
    move (pointer_c, 0);
    auto it = find(mappings.begin(), mappings.end(), s);
    if (uppercase == 1)
      command = command - 32;
      s = command;
      command = command + 32;
    // If element was found
    while (it != mappings.end()) 
    {
      int index = it - mappings.begin();
      std::string behavior_path;
      std::string check_path;
      move (pointer_c, 0);
      printw("Last key pressed: ");
      printw(s.c_str());
      move (pointer_c +2, 0);
      clrtoeol();      
      if (uppercase == 0){
        behavior_path =  "/" + drone_id_namespace + "/" + packages.at(index) + "/" + names.at(index) + "/activate_behavior";
        check_path =  "/" + drone_id_namespace + "/" + packages.at(index) + "/" + names.at(index) + "/check_activation_conditions";
        manage_behavior_srv = n.serviceClient<behavior_execution_manager_msgs::ActivateBehavior>(behavior_path);
        check_situation_srv = n.serviceClient<behavior_execution_manager_msgs::CheckSituation>(check_path);
        behavior_execution_manager_msgs::CheckSituation check_situation_msg;
        check_situation_srv.call(check_situation_msg);
        if (check_situation_msg.response.situation_occurs){
          behavior_execution_manager_msgs::ActivateBehavior activate_behavior_msg;
          activate_behavior_msg.request.arguments = parameters.at(index);
          activate_behavior_msg.request.timeout = 1000;
          manage_behavior_srv.call(activate_behavior_msg);       

          if(!activate_behavior_msg.response.ack){
            move (pointer_c, 0);
            printw("Last key pressed: ");
            printw(s.c_str());                    
            attron(COLOR_PAIR(4));
            pointer_c = pointer_c +2;
            move (pointer_c, 0);
            printw("Error: Activation failure of behavior "); printw(behaviors.at(index).c_str()); attroff(COLOR_PAIR(4));
            pointer_c = pointer_c -2;
            break;                    
          }
        }
        else {
            move (pointer_c, 0);
            printw("Last key pressed: ");
            printw(s.c_str());                    
            attron(COLOR_PAIR(4));
            pointer_c = pointer_c +2;
            move (pointer_c, 0);          
            printw("Error: Unsatisfied pre-conditions of behavior "); printw(behaviors.at(index).c_str()); attroff(COLOR_PAIR(4));
            pointer_c = pointer_c -2;            
            break;
        }       
      }
      else {
        behavior_path =  "/" + drone_id_namespace + "/" + packages.at(index) + "/" + names.at(index) + "/deactivate_behavior";
        manage_behavior_srv = n.serviceClient<behavior_execution_manager_msgs::DeactivateBehavior>(behavior_path);
        behavior_execution_manager_msgs::DeactivateBehavior deactivate_behavior_msg;
        manage_behavior_srv.call(deactivate_behavior_msg);     

        if(!deactivate_behavior_msg.response.ack){
          move (pointer_c, 0);
          printw("Last key pressed: ");
          printw(s.c_str()); 
          attron(COLOR_PAIR(4)); printw(" Error: Deactivation failure of behavior "); printw(behaviors.at(index).c_str()); attroff(COLOR_PAIR(4));
          break;
        }
      }

      it++;
      if (uppercase){
        std::string str(1,command);
        it = find(it, mappings.end(),str);
        }
      else
        it = find(it, mappings.end(), s);
      }
    uppercase = 0;  
     
    refresh();
    loop_rate.sleep();
    }
    endwin();
    return 0;
}

void printoutControls(){
  pointer_c = 2;
  move(pointer_c,0);clrtoeol();
  printw("Key assignment to activate behaviors:");
  int index = 0;
  int j = mappings.size();
  pointer_c = pointer_c + 2;
  for(auto it = mappings.begin(); it != mappings.end(); it++, index++)    {
    // found nth element..print and break.
    int pointer_y = 1;

    move(pointer_c,pointer_y);clrtoeol();
    std::string map = *it;
    if (map == "ASCII_KEY_UP"){
      map = "\u2191";
    }
    else if (map == "ASCII_KEY_DOWN"){
      map = "\u2193";
    }
    else if (map == "ASCII_KEY_RIGHT"){
      map = "\u2190";
    }
    else if (map == "ASCII_KEY_LEFT"){
      map = "\u2192";
    }
    
    if (find(key_printed.begin(), key_printed.end(), map.c_str()) == key_printed.end()){
      attron(COLOR_PAIR(5));printw(map.c_str()); attroff(COLOR_PAIR(5)); 
      pointer_y = pointer_y + 3; 
      move(pointer_c,pointer_y++);
      printw("{");
           
      for (int i = 0; i<j; i++){
        if (map.c_str() == mappings.at(i)){
          move(pointer_c++,pointer_y);
          printw(behaviors.at(i).c_str());
          for (int k = (j-i)-1; k>i; k--){
            if (map.c_str() == mappings.at(k)){
              if (parameters.at(i) == ""){
              printw(",");
              }
              else {
              printw(" ");
              }
              if (parameters.at(i) != ""){
                printw(",");
              }
              break;
            }
          }
          if (parameters.at(i) != ""){
            printw(" ");
          }          
          printw(parameters.at(i).c_str());
                     
          std::string str = "& ";
          std::string str2 = "\n";
          size_t found = parameters.at(i).find(str);
          if (found != std::string::npos){
            
            parameters.at(i).replace(found, 2, str2);
            
          }    
        }
        
        if (i == j-1){
          
          addch('}');
        }
      }
      //move(--pointer_c,pointer_y);clrtoeol();
      
      key_printed.push_back(map.c_str());
    }  
  }
  pointer_c = pointer_c + 1;    
  move (pointer_c, 0);
  printw("Use Shift key to deactivate behaviors");
  pointer_c = pointer_c + 2; 
  refresh();
}



//High level command


