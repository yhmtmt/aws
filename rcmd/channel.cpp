#include <iostream>
#include "aws_cmd.h"


#define AWS_CMD "channel"
#define AWS_CMD_USAGE "channel <type> <name>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
