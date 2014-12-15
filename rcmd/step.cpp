#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "step"
#define AWS_CMD_USAGE "step [<absolute time> | c <cycles>]"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
