#include "stdafx.h"
#include "CmdAppBase.h"

bool parse_str_opt(const char * opt, char ** argv, int argc, 
		   int & iarg, const char ** str)
{
  if(strcmp(opt, argv[iarg]) == 0){
    iarg++;
    if(iarg >= argc){
      throw argv[iarg-1];
    }
    
    *str = argv[iarg];
    iarg++;
    return true;
  }
  return false;
}

bool parse_bool_opt(const char * opt, char ** argv, int argc, 
		    int & iarg, bool & val)
{
  if(strcmp(opt, argv[iarg]) == 0){
    val = true;
    iarg++;
    return true;
  }
  return false;
}

bool parse_int_opt(const char * opt, char ** argv, int argc, 
		   int & iarg, int & val)
{
  if(strcmp(opt, argv[iarg]) == 0){
    iarg++;
    if(iarg >= argc){
      throw argv[iarg-1];
    }
    
    val = atoi(argv[iarg]);
    iarg++;
    return true;
  }
  return false;
}

bool parse_real_opt(const char * opt, char ** argv, 
		    int argc, int & iarg, double & val)
{
  if(strcmp(opt, argv[iarg]) == 0){
    iarg++;
    if(iarg >= argc){
      throw argv[iarg-1];
    }
    
    val = atof(argv[iarg]);
    iarg++;
    return true;
  }
  return false;
}
