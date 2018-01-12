#ifndef _CMDAPPBASE_H_
#define _CMDAPPBASE_H_
// Copyright (c) 2013-2014 "Yohei Matsumoto,"
// Command line application utility library

// This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.


//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

//        About class c_cmd_app_base
// You can make command line application easily by inhering class c_cmd_app_base
// and implementing the virtual methd "main". 
// In the true main function of your application, you only need to make an 
// instance of the class, and execute the run method. 
// Arguments for the application are to be enumerated in your constrcutor by 
// using add_arg and add_val methods. I do not give the information about 
// how to add arguments in your application but give a sample application
// instead of that.

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

bool parse_str_opt(const char * opt, char ** argv, int argc, 
		   int & iarg, const char ** str);
bool parse_bool_opt(const char * opt, char ** argv, int argc, 
		    int & iarg, bool & val);
bool parse_int_opt(const char * opt, char ** argv, int argc, 
		   int & iarg, int & val);
bool parse_real_opt(const char * opt, char ** argv, int argc, 
		    int & iarg, double & val);

class c_cmd_app_exception{
protected:
  int code;
 public:
 c_cmd_app_exception(): code(0)
    {
    }
  
 c_cmd_app_exception(int a_code): code(a_code){
  }
};

class CmdAppBase
{
 protected:
  const char * name_app;
  const char * name_coder;
  const char * contact;
  short year_copy;
  short ver_main;
  short ver_sub;
  
  int argc;
  char ** argv;
  
  int num_fixed_args;
  
  enum e_val_type{
    EV_BL, EV_INT, EV_FLT, EV_DBL, EV_STR
  };
  
  struct s_val_desc{
    s_val_desc()
    {}
    
  s_val_desc(bool * pbl, const char * a_explanation = NULL): vtype(EV_BL), pvbl(pbl), explanation(a_explanation)
    {
    }
    
  s_val_desc(char ** pstr, const char * a_explanation = NULL): vtype(EV_STR), pvstr(pstr), explanation(a_explanation)
    {
    }
    
  s_val_desc(int * pint, const char * a_explanation = NULL): vtype(EV_INT), pvint(pint), explanation(a_explanation)
		{
		}
    
  s_val_desc(float * pflt, const char * a_explanation = NULL): vtype(EV_FLT), pvflt(pflt), explanation(a_explanation)
    {
    }
    
  s_val_desc(double * pdbl, const char * a_explanation = NULL): vtype(EV_DBL), pvdbl(pdbl), explanation(a_explanation)
    {
    }
    
    
    ~s_val_desc()
    {}
    
    e_val_type vtype;
    const char * explanation;
    union {
      bool * pvbl;
      char ** pvstr;
      int * pvint;
      unsigned int * pvuint;
      float * pvflt;
      double * pvdbl;
    };
  };
  
  struct s_arg_desc{
    s_arg_desc()
    {
    }
  s_arg_desc(int a_iarg, const char * a_explanation = NULL):str(NULL), iarg(a_iarg), explanation(a_explanation)
      
    {}

  s_arg_desc(const char * a_str, const char * a_explanation = NULL):str(a_str), iarg(-1), explanation(a_explanation)
    {}
    
    ~s_arg_desc(){
    }
    
    const char * str;
    int iarg;
    const char * explanation;
    vector<s_val_desc> vals;
  };
  
  vector<s_arg_desc> args;
  
  void add_arg(int a_iarg, const char * a_explanation = NULL){
    args.push_back(s_arg_desc(a_iarg, a_explanation));
  }
  
  void add_arg(const char * a_str, const char * a_explanation = NULL){
    args.push_back(s_arg_desc(a_str, a_explanation));
  }
  void add_val(bool * pbl, const char * a_explanation = NULL){
    (args.end()-1)->vals.push_back(s_val_desc(pbl, a_explanation));
  }
  void add_val(char ** pstr, const char * a_explanation = NULL){
    (args.end()-1)->vals.push_back(s_val_desc(pstr, a_explanation));
  }
  void add_val(int * pint, const char * a_explanation = NULL){
    (args.end()-1)->vals.push_back(s_val_desc(pint, a_explanation));
  }
  void add_val(float * pflt, const char * a_explanation = NULL){
    (args.end()-1)->vals.push_back(s_val_desc(pflt, a_explanation));
  }
  void add_val(double * pdbl, const char * a_explanation = NULL){
    (args.end()-1)->vals.push_back(s_val_desc(pdbl, a_explanation));
  }

 public:
 CmdAppBase(int a_argc, char ** a_argv):argc(a_argc), argv(a_argv), num_fixed_args(1),
    name_app(NULL), year_copy(2012), ver_main(0), ver_sub(0), contact(NULL)
  {
  }
  
  virtual ~CmdAppBase()
    {
    }
  
  void set_name_app(const char * a_name_app){
    name_app = a_name_app;
  }
  
  void set_contact(const char * a_contact)
  {
    contact = a_contact;
  }
  
  void set_year_copy(const short a_year_copy){
    year_copy = a_year_copy;
  }
  
  void set_name_coder(const char * a_name_coder){
    name_coder = a_name_coder;
  }
  
  void set_version(const short a_ver_main, const short a_ver_sub)
  {
    ver_main = a_ver_main;
    ver_sub = a_ver_sub;
  }
  
  virtual void print_title(){
    if(name_app)
      cout << name_app;
    else
      cout<< "nanashi";
    
    cout << " Ver." << ver_main << "." << ver_sub;
    cout << " (built " << __DATE__ << " " << __TIME__ << ")" << endl;
    cout << "Copyright (c) " << year_copy << " " << name_coder << " All Rights Reserved" << endl;
    if(contact)
      cout << contact << endl;
  }
  
  virtual bool parse_args()
  {
    // counting number of fixed arguments
    num_fixed_args = 1;
    for(int i = 0; i < (int) args.size(); i++){
      if(args[i].str != NULL)
	continue;
      if(args[i].iarg != num_fixed_args){
	throw c_cmd_app_exception();
      }
      num_fixed_args++;
    }
    
    if(num_fixed_args > argc){
      cerr << "Few arguments" << endl;
      return false;
    }
    
    // processinig fixed arguments
    for(int i = 0; i < (int) args.size(); i++){
      if(args[i].str != NULL)
	continue;
      int iarg = args[i].iarg;
      set_val(argv[iarg], args[i].vals[0]);
    }
    
    // processing optional arguments
    int iarg = num_fixed_args; 
    while(iarg < argc){
      bool valid = false;
      for(int i = 0; i < (int) args.size(); i++){
	if(args[i].str == NULL)
	  continue;
	
	if(strcmp(args[i].str, argv[iarg]) == 0){
	  iarg++;
	  int num_vals = (int) args[i].vals.size();
	  if(num_vals + iarg > argc){
	    cerr << "Few values for " << args[i].str << "." << endl;
	    return false;
	  }
	  for(int ival = 0; ival < num_vals; ival++){
	    set_val(argv[iarg], args[i].vals[ival]);
	    iarg++;
	  }
	  valid = true;
	  break;
	}
      }
      
      if(!valid){
	cerr << "Unknown argument " << argv[iarg] << "." << endl;
	return false;
      }
    }
    
    return true;
  }
  
  void set_val(char * arg, s_val_desc & vd){
    switch(vd.vtype){
    case EV_BL:
      *vd.pvbl = (strcmp("on", arg)==0 ? true : false);
      break;
    case EV_STR:
      *vd.pvstr = arg;
      break;
    case EV_INT:
      *vd.pvint = atoi(arg);
      break;
    case EV_FLT:
      *vd.pvflt = (float) atof(arg);
      break;
    case EV_DBL:
      *vd.pvdbl = (float) atof(arg);
      break;
    }
  }
  
  virtual void print_usage()
  {
    cout << "Usage: " << name_app;
    for (int i = 0; i < (int) args.size(); i++){
      if(args[i].str != NULL)
	continue;
      cout << " <" << args[i].vals[0].explanation << ">";
    }
    
    cout << " [Options]" << endl;
    cout << "Duty arguments:" << endl;
    for (int i = 0; i < (int) args.size(); i++){
      if(args[i].str != NULL)
	continue;
      cout << "\t<" << args[i].vals[0].explanation << ">\t" << args[i].explanation << endl;
    }
    cout << endl;
    
    cout << "Options:" << endl;
    for(int i = 0; i < (int) args.size(); i++){
      if(args[i].str == NULL)
	continue;
      cout << "\t" << args[i].str << " ";
      int num_vals = (int) args[i].vals.size();
      for(int ival = 0; ival < num_vals; ival++){
	cout << " <" << args[i].vals[ival].explanation <<">";
			}
      cout << endl;
      cout << "\t\t" << args[i].explanation << endl;
    }
  }
  
  bool run()
  {
    print_title();

    if(!parse_args()){
      print_usage();
      return false;
    }
    
    return main();
  }
  
  virtual bool main() =0;
};

#endif
