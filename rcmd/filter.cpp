#include <iostream>
#include "aws_cmd.h"


#define AWS_CMD "filter"
#define AWS_CMD_USAGE "filter <type> <name> -i <input list> -o <output list>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
