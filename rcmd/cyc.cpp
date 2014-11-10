#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "cyc"
#define AWS_CMD_USAGE "cyc <cycle time in second>"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
