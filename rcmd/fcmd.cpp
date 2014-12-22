#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "fcmd"
#define AWS_CMD_USAGE "fcmd <filter name> <filter command>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
