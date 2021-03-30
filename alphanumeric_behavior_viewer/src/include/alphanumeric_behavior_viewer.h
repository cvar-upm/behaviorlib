/*!*******************************************************************************************
 * \brief     Keyboard teleoperation with behaviors implementation file.
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

#ifndef KEYBOARD_BEHAVIOR_TELEOPERATION_MAIN_H
#define KEYBOARD_BEHAVIOR_TELEOPERATION_MAIN_H

#include <string>
#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <cctype>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <behavior_execution_manager_msgs/CheckActivation.h>
#include <curses.h>
#include <thread>
#include <locale.h>
#include "yaml-cpp/yaml.h"

//Loop rate
#define FREQ_INTERFACE 50.0

ros::ServiceClient check_behavior_situation_srv;

void printoutControls();

//Publishers

//Topics
std::string drone_id_namespace;
std::string config_file;

//variables 
std::vector <std::string> names;
std::vector <std::string> packages;
std::vector <std::string> behaviors;
std::vector <std::string> active_behaviors;
std::vector <std::string> active_behaviors_parameters;
std::string path;

int pointer_c;
int pointer_d;



#endif
