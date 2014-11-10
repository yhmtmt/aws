#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "stop"
#define AWS_CMD_USAGE "stop"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
